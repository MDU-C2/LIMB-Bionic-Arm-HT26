"""Check session continuity and illustrations against the recorder's timed movements."""

from __future__ import annotations

import json
import tempfile
import tkinter as tk
import unittest
from pathlib import Path

from aurora.gui.guidance import GuidanceViews, display_sign, illustration_key
from aurora.gui.session import ApplicationSession, CupViewState
from aurora.recording.cup_protocol import REFERENCE_STEPS, CupReferenceGuide, CupTrialGuide


class ProtocolViewTests(unittest.TestCase):
    def test_every_reference_movement_has_a_stable_visual_id_and_timed_progress(self):
        now = [0]
        guide = CupReferenceGuide(clock=lambda: now[0])
        for movement, seconds, instruction in REFERENCE_STEPS:
            self.assertEqual(guide.view()["movement"], movement)
            self.assertEqual(illustration_key(guide.view()), movement)
            self.assertEqual(guide.view()["progress"], 0)
            guide.start_step()
            now[0] += 1_500_000_000
            self.assertEqual(guide.view()["progress"], 0.5)
            now[0] += 1_500_000_000
            guide.tick()
            self.assertIn("GO", guide.view()["cue"])
            self.assertEqual(guide.view()["instruction"], instruction)
            now[0] += int(seconds * 0.5e9)
            self.assertEqual(guide.view()["progress"], 0.5)
            now[0] += int(seconds * 0.5e9)
            guide.tick()
        self.assertTrue(guide.completed)

    def test_capture_and_rest_progress_use_their_own_backend_start_times(self):
        now = [0]
        guide = CupTrialGuide(clock=lambda: now[0])
        guide.begin("slow")
        now[0] = 3_000_000_000
        guide.tick()
        now[0] += 4_000_000_000
        self.assertEqual(guide.view()["progress"], 0.5)
        self.assertEqual(guide.view()["movement"], "CUP")
        guide.finish()
        self.assertEqual(guide.view()["progress"], 0)
        now[0] += 10_000_000_000
        self.assertEqual(guide.view()["progress"], 0.5)

    def test_status_is_a_view_of_backend_flags_without_inferring_readiness_from_words(self):
        state = CupViewState()
        state.phase = "arming"
        state.update({"ready": False, "message": "Ready", "protocol": {"phase": "reference"}})
        self.assertFalse(state.status["can_record"])
        self.assertEqual(state.phase, "reference")
        state.update({"can_record": True, "protocol": {"phase": "ready"}})
        self.assertTrue(state.status["can_record"])

    def test_camera_reflection_is_applied_once(self):
        for side in ("left", "right"):
            own = display_sign(side)
            self.assertEqual(display_sign(side, camera_view=True), -own)
            self.assertEqual(display_sign(side, camera_view=True, mirrored=True), own)


class SessionTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Native Tk unavailable: {error}")
        self.root.withdraw()
        self.addCleanup(self.root.destroy)
        self.temporary = tempfile.TemporaryDirectory(prefix="aurora session Å ")
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "preferences.json"
        self.session = ApplicationSession(self.root, self.path)
        self.addCleanup(self.session.close)

    def test_preferences_restore_inputs_but_never_live_validity_or_previous_trial(self):
        self.session.variables["recording_subject"].set("S07")
        self.session.variables["recording_camera_side"].set("right")
        self.session.variables["recording_output"].set(str(self.path.parent / "captures Å"))
        self.session.calibration_path = self.path.parent / "reference.json"
        self.session.selected_trial = self.path.parent / "trial.json"
        self.session.cup.update({"can_record": True})
        self.session.save()
        restored = ApplicationSession(self.root, self.path)
        self.addCleanup(restored.close)
        self.assertEqual(restored.variables["recording_subject"].get(), "S07")
        self.assertEqual(restored.variables["recording_camera_side"].get(), "right")
        self.assertEqual(
            restored.variables["recording_output"].get(), str(self.path.parent / "captures Å")
        )
        self.assertIsNone(restored.calibration_path)
        self.assertIsNone(restored.selected_trial)
        self.assertFalse(restored.cup.status["can_record"])

    def test_active_session_rejects_conflicting_identity_and_allows_next_trial_settings(self):
        subject = self.session.variables["recording_subject"]
        self.session.context_locked = True
        subject.set("other")
        self.assertEqual(subject.get(), "S01")
        self.assertIn("Finish", self.session.notice.get())
        duration = self.session.variables["cup_duration"]
        duration.set("12")
        self.assertEqual(duration.get(), "12")
        self.session.cup.phase = "movement"
        duration.set("4")
        self.assertEqual(duration.get(), "12")
        self.session.context_locked = False
        self.session.calibration_path = Path("reference.json")
        subject.set("S08")
        self.assertIsNone(self.session.calibration_path)

    def test_corrupted_preferences_have_visible_error_and_invalid_choices_use_defaults(self):
        self.path.write_text("{broken", encoding="utf-8")
        values, error = ApplicationSession.load(self.path)
        self.assertIn("defaults", error)
        self.assertEqual(values["recording_camera_side"], "left")
        self.path.write_text(
            json.dumps(
                {
                    "format_version": 1,
                    "preferences": {
                        "recording_camera_side": "neither",
                        "cup_duration": 9,
                        "calibration_valid": True,
                    },
                }
            )
        )
        values, error = ApplicationSession.load(self.path)
        self.assertEqual(error, "")
        self.assertEqual(values["cup_duration"], "8")
        self.assertNotIn("calibration_valid", values)

    def test_both_views_follow_one_snapshot_and_markers_keep_physical_right_for_either_arm(self):
        views = GuidanceViews(self.root)
        views.operator.pack()
        views.open_participant()
        now = [0]
        guide = CupReferenceGuide(clock=lambda: now[0])
        views.window.destroy()
        self.assertEqual(guide.step, 0)
        self.assertTrue(guide.active)
        views.open_participant()
        for index, (movement, seconds, _) in enumerate(REFERENCE_STEPS):
            guide.start_step()
            now[0] += 3_000_000_000
            guide.tick()
            view = guide.view()
            for side in ("left", "right"):
                views.update(view, side, "Checking movement")
                self.root.update()
                self.assertEqual(views.operator.snapshot, views.participant.snapshot)
                self.assertIn(movement, views.operator.snapshot["cue"])
                self.assertIn(side, views.participant.title.cget("text"))
                self.assertEqual(guide.step, index)
                if movement in {"STILL", "RAISE", "BEND"}:
                    x1, y1, x2, y2, x3, y3 = views.participant.canvas.coords("active_arm")
                    dot = (x1 - x2) * (x3 - x2) + (y1 - y2) * (y3 - y2)
                    if movement == "BEND":
                        self.assertGreater(dot, 0)
                    else:
                        self.assertAlmostEqual(dot, 0)
            now[0] += int(seconds * 1e9)
            guide.tick()
        for side in ("left", "right"):
            views.update({"movement": "CUP", "phase": "movement", "progress": 0.4}, side)
            for pane in (views.operator, views.participant):
                c = pane.canvas
                self.assertGreater(c.coords("marker3")[0], c.coords("marker2")[0])
        views.operator.mirrored = False
        views.operator.draw()
        self.assertLess(
            views.operator.canvas.coords("marker3")[0], views.operator.canvas.coords("marker2")[0]
        )
        views.window.destroy()
        self.assertTrue(guide.completed)


if __name__ == "__main__":
    unittest.main()
