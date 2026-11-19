"""Exercise generated sensor physics, the real processing path and desktop rehearsal."""

from __future__ import annotations

import csv
import gc
import json
import math
import struct
import tempfile
import time
import tkinter as tk
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

from aurora.gui.sensor_learning import MOVEMENT_NAMES, SensorLearning
from aurora.gui.sensor_plot import AXIS_COLORS, draw_traces
from aurora.sensors.emg_packets import HIGH_RATE_EMG_HEADER
from aurora.sensors.sensor_lesson import EMG_RATE, STEP_S, SensorLesson, mounting


def advance(lesson, seconds):
    for _ in range(round(seconds / STEP_S)):
        lesson.advance()
    return lesson.latest


class SensorLearningTests(unittest.TestCase):
    def test_generated_emg_uses_the_firmware_header_and_sample_intervals(self):
        lesson = self.lesson()
        with patch.object(
            lesson.reader, "notification", wraps=lesson.reader.notification
        ) as receive:
            lesson.advance()
        payload = next(call.args[1] for call in receive.call_args_list if call.args[0] == "emg")
        flags, sequence, first_us, period_us, count, offset = HIGH_RATE_EMG_HEADER.unpack_from(
            payload
        )
        self.assertEqual(
            (flags, sequence, first_us, period_us, count, offset), (0x100, 1, 1_000_000, 250, 40, 0)
        )
        self.assertEqual(
            struct.unpack_from(f"<{count}H", payload, HIGH_RATE_EMG_HEADER.size),
            tuple(lesson.latest["emg"]),
        )

    def test_processing_does_not_require_optional_hardware_or_neural_modules(self):
        with patch.dict(
            "sys.modules",
            {name: None for name in ("bleak", "depthai", "cv2", "mediapipe", "pybullet", "torch")},
        ):
            lesson = self.lesson()
            row = advance(lesson, 0.1)
            self.assertEqual(row["packets"], {"imu": 10, "emg": 10})
            self.assertEqual(len(row["emg"]), 40)
            self.assertIsNone(lesson.reader.thread)
            self.assertIsNone(lesson.live.pipeline)

    def test_signal_graph_does_not_draw_across_a_missing_interval(self):
        canvas = Mock()
        canvas.winfo_width.return_value = 500
        canvas.winfo_height.return_value = 200
        draw_traces(canvas, [[0, 1, math.nan, 2, 3]])
        segments = [
            call.args
            for call in canvas.create_line.call_args_list
            if call.kwargs.get("fill") == AXIS_COLORS[0]
        ]
        self.assertEqual(len(segments), 2)
        self.assertLess(max(segments[0][::2]), min(segments[1][::2]))

    def lesson(self, **kwargs):
        lesson = SensorLesson(**kwargs)
        self.addCleanup(lesson.close)
        return lesson

    def test_still_has_gravity_without_angular_velocity_for_different_mountings(self):
        for mount in ("Along segment", "Quarter turn", "Tilted"):
            with self.subTest(mount=mount):
                lesson = self.lesson()
                lesson.configure(dict(lesson="STILL", upper_mount=mount, forearm_mount=mount))
                row = advance(lesson, 1)
                for values in row["raw"].values():
                    self.assertAlmostEqual(
                        math.sqrt(sum(v * v for v in values[:3])), 1, delta=0.002
                    )
                    self.assertEqual(values[3:], [0, 0, 0])
                self.assertEqual(lesson.reader.decoder.decode_errors, {"imu": 0, "emg": 0})

    def test_bending_keeps_upper_arm_still_and_human_angle_exceeds_robot_travel(self):
        lesson = self.lesson()
        lesson.configure(dict(lesson="STILL"))
        advance(lesson, 1)
        lesson.configure(dict(lesson="MANUAL", elbow=130))
        moving = advance(lesson, 0.3)
        self.assertEqual(moving["raw"]["shoulder"][3:], [0, 0, 0])
        self.assertGreater(moving["raw"]["wrist"][3], 50)
        held = advance(lesson, 3)
        self.assertEqual(held["raw"]["wrist"][3:], [0, 0, 0])
        self.assertAlmostEqual(held["true_angles"]["elbow_flexion"], 130)
        self.assertAlmostEqual(held["angles"]["elbow_flexion"], 130, delta=2)
        self.assertEqual(held["targets"]["elbow_x"], 60)

    def test_real_causal_emg_calibration_responds_to_fluctuation_amplitude(self):
        lesson = self.lesson()
        lesson.configure(dict(lesson="RELAX"))
        resting = advance(lesson, 1)
        self.assertLess(resting["emg_rms"], 5)
        lesson.configure(dict(lesson="GRIP"))
        self.assertTrue(lesson.live.calibrate_emg_strength())
        gripping = advance(lesson, 3.5)
        self.assertEqual(gripping["emg_state"]["phase"], "ready")
        self.assertGreater(gripping["emg_rms"], resting["emg_rms"] * 10)
        self.assertGreater(gripping["activation"], 0.7)
        raw = [value for row in list(lesson.history)[-100:] for value in row["emg"]]
        self.assertAlmostEqual(float(np.mean(raw)), 2048, delta=10)
        lesson.configure(dict(lesson="RELEASE"))
        released = advance(lesson, 1)
        self.assertLess(released["emg_rms"], gripping["emg_rms"] * 0.2)
        self.assertLess(released["activation"], 0.1)

    def test_reference_rehearsal_fits_decoded_measurements_and_never_calibrates_hardware(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                lesson = self.lesson(side=side)
                lesson.configure(dict(upper_mount="Quarter turn", forearm_mount="Tilted"))
                lesson.start_preparation()
                while lesson.guide.active:
                    if lesson.guide.phase == "reference_prepare":
                        lesson.start_step()
                    lesson.advance()
                    self.assertLess(lesson.t, 100)
                profile = lesson.reader.alignment_profile
                self.assertIsNotNone(profile, lesson.fit_message)
                self.assertFalse(profile["accuracy_validated"])
                self.assertEqual(profile["source"], "synthetic_teaching_model")
                self.assertEqual(profile["subject_id"], "synthetic")
                for role, name in (("shoulder", "Quarter turn"), ("wrist", "Tilted")):
                    np.testing.assert_allclose(
                        profile["sensor_to_segment"][role], mounting(name), atol=0.005
                    )
                self.assertEqual(lesson.latest["emg_state"]["phase"], "ready")
                held = advance(lesson, 0.5)
                self.assertAlmostEqual(held["angles"]["elbow_flexion"], 90, delta=2)
                self.assertTrue(held["calibration_valid"])
                for angle in (30, -30):
                    lesson.configure(dict(lesson="MANUAL", turn=angle))
                    held = advance(lesson, 2)
                    self.assertAlmostEqual(held["true_angles"]["wrist_rotation"], angle)
                    self.assertAlmostEqual(held["angles"]["wrist_rotation"], angle, delta=0.5)
                generation = lesson.reader.mounting_session_id
                lesson.configure(dict(forearm_mount="Half turn"))
                self.assertNotEqual(lesson.reader.mounting_session_id, generation)
                self.assertFalse(lesson.live._calibration_valid)
                self.assertIsNone(lesson.reader.alignment_profile)

    def test_missing_sources_and_clipping_are_visible(self):
        lesson = self.lesson()
        advance(lesson, 1)
        lesson.configure(dict(fault="Forearm missing"))
        missing = advance(lesson, 0.1)
        self.assertEqual(missing["received_roles"], ["shoulder"])
        lesson.configure(dict(fault="Camera missing"))
        missing_camera = advance(lesson, 0.6)
        self.assertIn("Camera --", missing_camera["status"])
        self.assertIn("Body reference unavailable", missing_camera["status"])
        lesson.configure(dict(fault="ADC clipping", lesson="GRIP"))
        clipped = advance(lesson, 1)
        self.assertEqual(clipped["emg_state"]["phase"], "clipping")
        self.assertTrue(
            any(v in (0, 4095) for row in list(lesson.history)[-20:] for v in row["emg"])
        )

    def test_cup_rehearsal_uses_the_existing_countdown_capture_and_rest_times(self):
        lesson = self.lesson()
        lesson.start_trial()
        self.assertEqual(advance(lesson, 2.99)["guide"]["phase"], "countdown")
        self.assertEqual(advance(lesson, 0.02)["guide"]["phase"], "practice")
        self.assertEqual(advance(lesson, 8)["guide"]["phase"], "rest")
        self.assertEqual(lesson.latest["guide"]["movement"], "CUP")

    def test_bounded_export_preserves_adc_samples_and_labels_synthetic_provenance(self):
        lesson = self.lesson()
        advance(lesson, 21)
        self.assertEqual(len(lesson.history), 2000)
        with tempfile.TemporaryDirectory(prefix="learning Å ") as temporary:
            path = lesson.export(Path(temporary))
            manifest = json.loads((path / "lesson.json").read_text())
            self.assertFalse(manifest["participant_data"])
            self.assertFalse(manifest["accuracy_validated"])
            self.assertEqual(manifest["source"], "synthetic_teaching_model")
            self.assertAlmostEqual(manifest["window_seconds"], 20)
            with (path / "emg.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 20 * EMG_RATE)
            stamps = [int(row["device_time"]) for row in rows]
            self.assertEqual(set(np.diff(stamps)), {250})
            self.assertEqual([int(row["value"]) for row in rows[:40]], lesson.history[0]["emg"])
            self.assertFalse((path / "trial.json").exists())

    def test_invalid_settings_do_not_partially_change_the_model(self):
        lesson = self.lesson()
        for values in (dict(elbow=float("nan")), dict(effort=2), dict(fault="unknown")):
            original = dict(lesson.settings)
            with self.assertRaises(ValueError):
                lesson.configure(values)
            self.assertEqual(lesson.settings, original)


class NativeLearningTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Native Tk unavailable: {error}")
        self.root.withdraw()
        self.temporary = tempfile.TemporaryDirectory(prefix="sensor GUI Å ")
        self.addCleanup(self.temporary.cleanup)
        self.errors = []
        self.root.report_callback_exception = lambda *error: self.errors.append(error)
        self.app = SensorLearning(self.root, side="right", output=Path(self.temporary.name))
        self.addCleanup(self.close_app)
        self.pump(lambda: bool(self.app.rows))

    def pump(self, condition, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.root.update()
            if condition():
                self.assertEqual(self.errors, [])
                return
            time.sleep(0.01)
        self.fail(f"Learning action timed out: {self.app.status.get()}; {self.errors}")

    def close_app(self):
        self.app.close()
        deadline = time.monotonic() + 4
        while self.app.worker.thread.is_alive() and time.monotonic() < deadline:
            self.root.update()
            time.sleep(0.01)
        self.assertFalse(self.app.worker.thread.is_alive())
        try:
            self.root.update()
            self.root.destroy()
        except tk.TclError:
            pass
        self.app = None
        self.root = None
        gc.collect()

    def test_controls_reach_processing_pause_scrub_step_and_export(self):
        self.app.variables["lesson"].set(MOVEMENT_NAMES["MANUAL"])
        self.app.variables["elbow"].set(130)
        self.app.configure_source()
        self.app.command("explore")
        self.pump(lambda: self.app.rows[-1]["true_angles"]["elbow_flexion"] == 130)
        self.app.pause_button.invoke()
        self.pump(lambda: self.app.paused)
        stamp = self.app.rows[-1]["time_s"]
        self.app.age.set(0.2)
        self.app.render()
        self.assertNotIn(f"Simulation {stamp:.2f} s", self.app.status.get())
        self.app.command("step")
        self.pump(lambda: self.app.rows[-1]["time_s"] > stamp)
        self.assertAlmostEqual(self.app.rows[-1]["time_s"], stamp + STEP_S)
        self.app.command("export")
        self.pump(lambda: self.app.last_output is not None)
        manifest = json.loads((self.app.last_output / "lesson.json").read_text())
        self.assertEqual(manifest["side"], "right")
        self.assertEqual(manifest["settings"]["elbow"], 130)
        self.assertFalse(manifest["participant_data"])

    def test_guide_uses_backend_cues_and_closes_during_preparation(self):
        self.app.command("prepare")
        self.pump(lambda: self.app.rows[-1]["guide"].get("phase") == "reference_prepare")
        self.app.open_guide()
        self.app.next_button.invoke()
        self.pump(lambda: self.app.rows[-1]["guide"].get("phase") == "reference_countdown")
        self.assertEqual(self.app.guide_pane.snapshot["movement"], "STILL")
        self.assertEqual(self.app.guide_pane.snapshot["phase"], "reference_countdown")
        self.assertEqual(self.app.guide_pane.side, "right")


if __name__ == "__main__":
    unittest.main()
