"""Exercise desktop forms through real offline workers and their saved artifacts."""

from __future__ import annotations

import csv
import gc
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import time
import tkinter as tk
import unittest
import zipfile
from pathlib import Path
from tkinter import ttk
from unittest.mock import patch

import numpy as np

from aurora.gui.app import ProjectGui
from aurora.gui.tool_catalog import arguments_for
from aurora.gui.workflow_tools import WorkflowTools


def button(parent, text):
    for child in parent.winfo_children():
        if isinstance(child, ttk.Button) and child.cget("text") == text:
            return child
        found = button(child, text)
        if found is not None:
            return found
    return None


class ToolValidationTests(unittest.TestCase):
    def test_cooperative_worker_can_spawn_a_noninteractive_helper_and_stop_cleanly(self):
        from aurora.recording.common import STOP_COMMAND

        code = """
import faulthandler, pathlib, subprocess, sys, time
from aurora.recording.common import enable_ctrl_break_as_interrupt
faulthandler.dump_traceback_later(2)
enable_ctrl_break_as_interrupt(stdin_stop=True)
time.sleep(0.1)
root = pathlib.Path(sys.argv[1])
value = subprocess.check_output([sys.executable, '-c', 'print(7)'], text=True, stdin=subprocess.DEVNULL)
(root / 'child_ready').write_text(value)
faulthandler.cancel_dump_traceback_later()
try:
    while True:
        time.sleep(0.01)
except KeyboardInterrupt:
    (root / 'closed').write_text('clean shutdown')
    raise SystemExit(130)
"""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            process = subprocess.Popen(
                [sys.executable, "-u", "-c", code, str(root)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env={**os.environ, "AURORA_GUI_CONTROL": "1"},
            )
            try:
                deadline = time.monotonic() + 5
                while not (root / "child_ready").is_file() and time.monotonic() < deadline:
                    time.sleep(0.01)
                if not (root / "child_ready").is_file():
                    process.kill()
                    output, _ = process.communicate(timeout=5)
                    self.fail(
                        "Worker could not spawn a child while its command pipe was idle: " + output
                    )
                output, _ = process.communicate(STOP_COMMAND, timeout=5)
                self.assertEqual(process.returncode, 130, output)
                self.assertEqual((root / "child_ready").read_text().strip(), "7")
                self.assertEqual((root / "closed").read_text(), "clean shutdown")
            finally:
                if process.poll() is None:
                    process.kill()
                process.communicate(timeout=5)

    def test_usb_diagnostic_form_uses_the_existing_controller_contract(self):
        from aurora.simulation.interactive.settings import parse_args

        args = parse_args(
            arguments_for(
                "serial_control",
                {
                    "port": "COM7",
                    "baud": 230400,
                    "side": "right",
                    "subject": "S03",
                    "mode": "Motor physics",
                    "upper_axis": "-x",
                    "forearm_axis": "z",
                    "weight": 0.4,
                },
            )
        )
        self.assertEqual(
            (args.control, args.transport, args.port), ("camera-imu", "serial", "COM7")
        )
        self.assertEqual((args.baud, args.side, args.subject), (230400, "right", "S03"))
        self.assertEqual((args.shoulder_twist_axis, args.wrist_twist_axis), ("-x", "z"))
        self.assertEqual(args.camera_weight, 0.4)
        self.assertEqual(args.mode, "dynamic")
        self.assertTrue(args.depth)

    def test_unsafe_form_combinations_are_rejected_before_a_child_starts(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "emg.csv"
            source.write_text("device_time,value\n0,2048\n")
            settings = {"source": str(source), "output": str(root / "derived")}
            for update in ({"window": "nan"}, {"overlap": 200}, {"noise": -1}):
                with self.subTest(update=update), self.assertRaises(ValueError):
                    arguments_for("process_emg", settings | update)
            with self.assertRaisesRegex(ValueError, "already exists"):
                arguments_for("bundle", {"source": str(source), "output": str(source)})
            with self.assertRaisesRegex(ValueError, "headless"):
                arguments_for("dmp_play", {"path": str(root), "loop": True, "headless": True})

    def test_summary_distinguishes_capture_completion_labels_and_camera_duplicates(self):
        summary = WorkflowTools.trial_summary(
            {
                "complete": True,
                "pace_condition": "fast",
                "task_completion_verified": False,
                "movement_labels": "not_reviewed",
                "camera_source_frame_count": 8,
                "camera_video_frame_count": 12,
            }
        )
        self.assertIn("Requested pace: fast", summary)
        self.assertIn("Capture complete: yes", summary)
        self.assertIn("Task completion independently verified: no", summary)
        self.assertIn("Movement labels: not_reviewed", summary)
        self.assertIn("Acquired RGB frames: 8", summary)
        self.assertIn("Encoded video frames (may include duplicates): 12", summary)


class NativeWorkflowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            root = tk.Tk()
        except tk.TclError as error:
            raise unittest.SkipTest(f"Native Tk unavailable: {error}") from error
        root.destroy()

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="aurora GUI Å ")
        self.addCleanup(self.temporary.cleanup)
        self.root = tk.Tk()
        self.root.withdraw()
        self.callback_errors, self.errors = [], []
        self.root.report_callback_exception = lambda *error: self.callback_errors.append(error)
        self.error_patch = patch(
            "tkinter.messagebox.showerror",
            side_effect=lambda title, message, **kw: self.errors.append((title, message)),
        )
        self.error_patch.start()
        self.addCleanup(self.error_patch.stop)
        self.gui = ProjectGui(self.root)
        self.addCleanup(self.close_gui)
        self.directory = Path(self.temporary.name)
        self.trial = self.directory / "original trial"
        (self.trial / "ble").mkdir(parents=True)
        self.source = self.trial / "ble/emg.csv"
        times = np.arange(4096) / 4000
        values = 2048 + 100 * np.sin(2 * np.pi * 125 * times)
        with self.source.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("device_time", "channel", "value"))
            writer.writerows(zip(times * 1e6, np.zeros(len(times), dtype=int), values, strict=True))
        (self.trial / "trial.json").write_text(
            json.dumps(
                {
                    "subject_id": "S01",
                    "trial_id": "T001",
                    "complete": False,
                    "pace_condition": "slow",
                }
            )
        )
        self.original = self.source.read_bytes()
        self.gui.recordings_root.set(str(self.trial))
        self.gui._refresh_recordings()
        self.pump(
            lambda: any(
                p == self.trial / "trial.json" for p in self.gui.recording_tree_paths.values()
            )
        )
        for item, path in self.gui.recording_tree_paths.items():
            if path == self.trial / "trial.json":
                self.gui.recordings_tree.selection_set(item)

    def pump(self, condition, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.root.update()
            if condition():
                self.assertEqual(self.callback_errors, [])
                return
            time.sleep(0.01)
        self.fail(
            f"GUI action timed out. Errors: {self.errors}; Activity: {self.gui.log.get('1.0', 'end')[-3500:]}"
        )

    def close_gui(self):
        for process in self.gui._active_processes():
            self.gui._kill_process_tree_if_running(process.process)
            process.process.wait(timeout=10)
        self.gui.close()
        # Tk objects must be finalized on their owning thread between desktop cases.
        gc.collect()

    def form(self, key, values, via=None):
        if via:
            action = button(self.gui, via)
            self.assertIsNotNone(action, via)
            action.invoke()
            dialog = self.gui.workflow.dialogs[key]
        else:
            dialog = self.gui.workflow.show_tool(key)
        for name, value in values.items():
            variable = dialog.tool_variables[name]
            if isinstance(variable, dict):
                for option, selected in variable.items():
                    selected.set(option in value)
            else:
                variable.set(value)
        self.pump(lambda: str(dialog.run_button.cget("state")) == "normal")
        return dialog

    def run_form(self, key, values, via=None, expected_code=0):
        dialog = self.form(key, values, via)
        dialog.run_button.invoke()
        managed = self.gui.processes.get("tool:" + key)
        self.assertIsNotNone(managed, self.errors)
        self.pump(lambda: "tool:" + key not in self.gui.processes, timeout=90)
        self.assertEqual(managed.process.returncode, expected_code, "\n".join(managed.output_tail))
        self.assertEqual(self.source.read_bytes(), self.original)
        return managed

    def test_review_processing_and_spectrum_export_use_selected_settings(self):
        output = self.directory / "processed EMG"
        self.run_form(
            "process_emg",
            {
                "output": str(output),
                "lowcut": 30,
                "highcut": 400,
                "notch": 50,
                "window": 100,
                "overlap": 25,
                "noise": 7,
            },
            via="Process EMG",
        )
        metadata = json.loads((output / "emg.processing.json").read_text())
        self.assertEqual(
            {
                key: metadata[key]
                for key in (
                    "lowcut_hz",
                    "highcut_hz",
                    "window_ms",
                    "overlap_ms",
                    "noise_threshold_adc",
                    "samples",
                )
            },
            {
                "lowcut_hz": 30,
                "highcut_hz": 400,
                "window_ms": 100,
                "overlap_ms": 25,
                "noise_threshold_adc": 7,
                "samples": 4096,
            },
        )
        self.assertEqual(
            json.loads((output / "processing.run.json").read_text())["status"], "complete"
        )
        image = self.directory / "spectrum Å.png"
        self.run_form(
            "plot",
            {"sensor": "emg", "spectrum": True, "save": str(image)},
            via="Plot / export image",
        )
        self.assertGreater(image.stat().st_size, 1000)
        capture_root = self.gui.recording_output.get()
        self.gui.workflow.review_output()
        self.assertEqual(self.gui.recordings_root.get(), str(image.parent))
        self.assertEqual(self.gui.recording_output.get(), capture_root)
        self.assertEqual(self.errors, [])

    def test_bundle_action_exports_whole_selected_trial_and_reports_progress(self):
        output = self.directory / "trial bundle.zip"
        process = self.run_form("bundle", {"output": str(output)}, via="Export recording bundle")
        with zipfile.ZipFile(output) as archive:
            self.assertEqual(archive.read("recording/ble/emg.csv"), self.original)
            self.assertFalse(json.loads(archive.read("recording/trial.json"))["complete"])
        self.assertIn("Export: 2/2 files", "".join(process.output_tail))
        self.assertEqual(self.gui.workflow.last_output.get(), str(output))

    def test_dmp_fit_export_can_be_played_by_the_existing_headless_backend(self):
        source = self.directory / "angles"
        source.mkdir()
        elbow = 2 * np.sin(np.pi * np.linspace(0, 1, 201))
        np.savez(
            source / "angles.npz",
            elbow_rad=elbow,
            shoulder_rad=np.column_stack((elbow / 2, elbow / 3, elbow / 4)),
        )
        output = self.directory / "DMP export"
        self.run_form(
            "dmp_export", {"source": str(source), "output": str(output)}, via="Fit and export DMP"
        )
        trajectory = np.loadtxt(output / "trajectory.csv", delimiter=",", skiprows=1)
        self.assertGreater(trajectory[:, 1].max(), 1.4)
        playback = self.run_form(
            "dmp_play", {"path": str(output), "headless": True}, via="Play DMP trajectory"
        )
        self.assertIn("values clipped", "".join(playback.output_tail))
        self.assertEqual(self.errors, [])

    def test_backend_error_is_visible_and_stop_cancels_a_real_worker(self):
        output = self.directory / "invalid processing"
        self.run_form(
            "process_emg",
            {"output": str(output), "highcut": 3000},
            via="Process EMG",
            expected_code=1,
        )
        run = json.loads((output / "processing.run.json").read_text())
        self.assertEqual(run["status"], "failed")
        self.assertTrue(self.errors)
        self.assertIn(run["error"], self.errors[-1][1])
        dialog = self.form(
            "benchmark",
            {
                "seconds": 60,
                "steps": 10000,
                "repeats": 3,
                "output": str(self.directory / "cancelled.json"),
            },
        )
        dialog.run_button.invoke()
        managed = self.gui.processes["tool:benchmark"]
        self.pump(lambda: any("Measured sensor_core" in line for line in managed.output_tail))
        self.gui.stop_button.invoke()
        self.pump(lambda: "tool:benchmark" not in self.gui.processes, timeout=12)
        self.assertIsNotNone(managed.process.poll())
        self.assertIn("stopped", self.gui.status.get())
        self.assertEqual(str(self.gui.stop_button.cget("state")), "disabled")

    def test_recorded_replay_check_routes_to_sensor_processing_without_hardware(self):
        from test_recorded_control import make_trial

        replay = self.directory / "replay"
        replay.mkdir()
        make_trial(replay)
        output = self.directory / "replayed"
        self.run_form("replay_check", {"trial": str(replay), "output": str(output)})
        report = json.loads((output / "check.json").read_text())
        self.assertEqual(report["events_decoded"], 204)
        self.assertGreater(report["imu_ready_snapshots"], 0)
        self.assertEqual(self.errors, [])

    def test_keyboard_physics_settings_produce_telemetry_and_stop_cleanly(self):
        self.pump(lambda: self.gui.simulation_python is not None)
        self.gui.dynamic_simulation.set(True)
        self.gui.save_simulation_torque.set(True)
        self.gui.save_simulation_telemetry.set(True)
        self.gui.recording_camera_side.set("right")
        button(self.gui, "Start with keyboard").invoke()
        managed = self.gui.processes["simulation:Interactive task simulator"]
        self.pump(lambda: any("Torque CSV:" in line for line in managed.output_tail))
        paths = [
            Path(line.split(": ", 1)[1])
            for line in managed.output_tail
            if line.startswith(("Torque CSV:", "Physics telemetry:"))
        ]
        self.assertEqual(len(paths), 2)
        self.addCleanup(lambda: [path.unlink(missing_ok=True) for path in paths])
        self.pump(lambda: all(path.stat().st_size > 100 for path in paths))
        self.assertTrue(
            all(
                str(widget.cget("state")) == "disabled"
                for widget, _ in self.gui.workflow.context_widgets
            )
        )
        self.gui.stop_button.invoke()
        self.pump(lambda: managed.key not in self.gui.processes, timeout=12)
        self.assertIn("Simulation finished.", managed.output_tail)
        self.assertEqual(managed.process.returncode, 130)
        telemetry = next(path for path in paths if path.suffix == ".jsonl")
        rows = [json.loads(line) for line in telemetry.read_text().splitlines()]
        self.assertGreater(len(rows), 0)
        self.assertEqual(self.errors, [])

    def test_movement_model_selection_validates_training_manifest_and_weights(self):
        from aurora.gui.project_support import MOVEMENT_MODEL
        from aurora.ml.features import PREPROCESSING

        model = self.directory / MOVEMENT_MODEL.name
        model.write_bytes(MOVEMENT_MODEL.read_bytes())
        manifest = {
            "format_version": 1,
            "status": "complete",
            "preprocessing": PREPROCESSING,
            "artifacts": {model.name: hashlib.sha256(model.read_bytes()).hexdigest()},
        }
        (self.directory / "training.json").write_text(json.dumps(manifest))
        with patch("tkinter.filedialog.askopenfilename", return_value=str(model)):
            button(self.gui, "Review / select movement model").invoke()
        self.pump(lambda: button(self.root, "Use for movement-profile recognition") is not None)
        self.assertEqual(self.gui.movement_model_path, MOVEMENT_MODEL)
        button(self.root, "Use for movement-profile recognition").invoke()
        self.assertEqual(self.gui.movement_model_path, model)
        from test_movement_features import measured_frame

        references = self.directory / "reference profiles"
        references.mkdir()
        profile = {"user_id": "S01", "data": [measured_frame()] * 3}
        target = self.directory / "movement.json"
        target.write_text(json.dumps(profile))
        (references / "reference.json").write_text(json.dumps(profile))
        self.gui.motion_recording.set(str(target))
        self.gui.motion_references.set(str(references))
        self.gui.movement_threshold.set("0.123")
        self.pump(lambda: str(self.gui.analyze_motion_button.cget("state")) == "normal")
        self.gui.analyze_motion_button.invoke()
        managed = self.gui.processes["motion_ai:Movement recognition"]
        self.pump(lambda: managed.key not in self.gui.processes)
        self.assertEqual(managed.process.returncode, 0, managed.output_tail)
        self.assertIn("Prediction: S01", managed.output_tail)
        button(self.gui, "Use bundled model").invoke()
        model.write_bytes(b"invalid replacement weights")
        with patch("tkinter.filedialog.askopenfilename", return_value=str(model)):
            button(self.gui, "Review / select movement model").invoke()
        self.pump(lambda: bool(self.errors))
        self.assertEqual(self.gui.movement_model_path, MOVEMENT_MODEL)
        self.assertIn("matching completed training manifest", self.errors[-1][1])

    def test_manual_joints_and_recorded_pose_route_to_existing_previews(self):
        from test_movement_features import measured_frame

        dialog = self.form("joints", {"step": 0.02}, via="Manual robot joint sliders")
        dialog.run_button.invoke()
        managed = self.gui.processes["tool:joints"]
        self.pump(lambda: any("Manual joint control ready" in line for line in managed.output_tail))
        self.gui.stop_button.invoke()
        self.pump(lambda: managed.key not in self.gui.processes)
        self.assertEqual(managed.process.returncode, 0)
        source = self.directory / "pose.json"
        source.write_text(json.dumps({"data": [measured_frame()] * 3}))
        playback = self.run_form(
            "pose_play",
            {"recording": str(source), "headless": True, "fps": 30},
            via="Play recorded camera pose",
        )
        self.assertIn("Playback finished; skipped 0 invalid frame(s)", playback.output_tail)
        self.assertEqual(self.errors, [])

    def test_lda_training_and_explicit_model_selection_keep_participant_evaluation(self):
        from test_emg_learning import recording

        data = self.directory / "reviewed"
        for index in range(3):
            recording(data, subject=f"S{index + 1:02}", seed=index)
        output = self.directory / "training run"
        self.run_form(
            "emg_train",
            {
                "data": str(data),
                "models": ["lda"],
                "features": ["basic"],
                "rate": 1000,
                "sequence": 1,
                "epochs": 1,
                "seed": 19,
                "output": str(output),
            },
            via="Compare EMG models",
        )
        report = json.loads((output / "training.json").read_text())
        self.assertEqual(report["status"], "complete")
        self.assertEqual({fold["subject_id"] for fold in report["folds"]}, {"S01", "S02", "S03"})
        model_file = output / "bundles/lda-basic/model.json"
        self.assertFalse(json.loads(model_file.read_text())["reviewed_for_live"])
        with patch("tkinter.filedialog.askopenfilename", return_value=str(model_file)):
            button(self.gui, "Review and select a live EMG model").invoke()
        self.pump(lambda: button(self.root, "Use this model for live EMG") is not None)
        self.assertIsNone(getattr(self.gui, "emg_model_path", None))
        button(self.root, "Use this model for live EMG").invoke()
        self.assertEqual(self.gui.emg_model_path, model_file.parent)
        self.assertTrue(json.loads(model_file.read_text())["reviewed_for_live"])
        button(self.gui, "Use calibrated EMG amplitude").invoke()
        self.assertIsNone(self.gui.emg_model_path)
        self.assertEqual(self.errors, [])


if __name__ == "__main__":
    unittest.main()
