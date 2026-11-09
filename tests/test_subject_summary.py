"""Check end-of-subject reports against saved capture health."""

from __future__ import annotations

import io
import json
import queue
import tempfile
import tkinter as tk
import unittest
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch

from aurora.gui.cup_recording import CupRecordingMixin
from aurora.gui.process_manager import ManagedProcess, ProcessManagerMixin
from aurora.gui.subject_summary import SubjectSummaryWindow
from aurora.recording.record_cup_trials import CupRecorder, main
from aurora.recording.subject_summary import COUNTERS, summarize_subject


def capture(directory, name, *, lost=0, reference=False):
    path = directory / ("references" if reference else "trials") / name
    path.mkdir(parents=True)
    data = {
        "trial_id": name,
        "test_type": "subject_reference" if reference else "cup_transfer",
        "pace_condition": None if reference else "slow",
        "complete": lost == 0,
        "stop_reason": "Recording window finished",
        "sensor_to_arm_alignment": "fitted",
        "reference_checks": {"emg_rest_ready": True, "emg_grip_ready": True},
        "packet_health": {
            "imu": {"received_packets": 98, "missing_packets": lost, "decode_errors": 0},
            "emg": {
                "received_packets": 98,
                "missing_packets": lost,
                "decode_errors": 0,
                "sample_count": 4000,
                "adc_rail_samples": lost * 2,
            },
        },
        "capture_health": {
            "camera_sequence_gaps": lost,
            "raw_frame_queue_overruns": 0,
            "depth_sequence_gaps": 0,
            "depth_frame_queue_overruns": 0,
        },
    }
    (path / "trial.json").write_text(json.dumps(data), encoding="utf-8")
    return path


class SubjectSummaryTests(unittest.TestCase):
    def test_only_current_session_is_counted_and_reference_losses_are_separate(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            old = capture(directory, "T000", lost=30)
            reference = capture(directory, "R001", lost=10, reference=True)
            trial = capture(directory, "T001", lost=2)
            clean = capture(directory, "T002")
            paths = [reference, trial, clean]
            original = {path: (path / "trial.json").read_bytes() for path in [old, *paths]}
            report = summarize_subject(directory, "S01", [*paths, trial])
            self.assertEqual(report["trial_count"], 2)
            self.assertEqual(report["complete_trials"], 1)
            self.assertEqual(report["trials_with_warnings"], 1)
            self.assertEqual(report["reference_count"], 1)
            self.assertEqual(report["trial_totals"]["imu_lost"], 2)
            self.assertEqual(report["reference_totals"]["imu_lost"], 10)
            self.assertEqual(report["trial_totals"]["emg_rail_samples"], 4)
            self.assertIn("IMU: 2 missing packets (2.00%)", report["captures"][1]["issues"])
            self.assertEqual(report["captures"][2]["issues"], [])
            self.assertEqual(
                original, {path: (path / "trial.json").read_bytes() for path in original}
            )

    def test_missing_or_unreadable_health_is_not_reported_as_zero_loss(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            path = capture(directory, "T001")
            data = json.loads((path / "trial.json").read_text())
            del data["packet_health"]
            (path / "trial.json").write_text(json.dumps(data))
            corrupt = capture(directory, "T002")
            (corrupt / "trial.json").write_text("{broken")
            report = summarize_subject(directory, "S01", [path, corrupt])
            self.assertIsNone(report["trial_totals"]["imu_lost"])
            self.assertIsNone(report["trial_totals"]["emg_lost"])
            self.assertEqual(report["trials_with_warnings"], 2)
            self.assertIn("not reported", " ".join(report["captures"][0]["issues"]))
            self.assertFalse(report["captures"][1]["complete"])

    def test_tracking_overflows_and_empty_camera_files_are_visible(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            path = capture(directory, "T001")
            data = json.loads((path / "trial.json").read_text())
            data.update(
                camera_video_frame_count=0, camera_depth_frame_count=0, stereo_depth_enabled=True
            )
            data["capture_health"]["tracking_queue_overruns"] = 3
            (path / "trial.json").write_text(json.dumps(data))
            report = summarize_subject(directory, "S01", [path])
            issues = report["captures"][0]["issues"]
            self.assertIn("Tracking results lost from the host queue: 3", issues)
            self.assertIn("No video frames were saved", issues)
            self.assertIn("No depth frames were saved", issues)

    def test_publishing_persists_the_report_once_and_keeps_cleanup_errors(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            path = capture(directory, "T001")
            recorder = object.__new__(CupRecorder)
            recorder.args = SimpleNamespace(subject="S01")
            recorder.trials = SimpleNamespace(directory=directory, capture_paths=[path])
            with patch("aurora.recording.record_cup_trials.emit") as emitted:
                recorder.publish_subject_summary(
                    session_errors=["Sensor reader: worker did not stop"]
                )
                recorder.publish_subject_summary()
            emitted.assert_called_once()
            self.assertEqual(emitted.call_args.args[0], "subject_summary")
            saved = json.loads(Path(emitted.call_args.kwargs["path"]).read_text())
            self.assertEqual(saved["session_errors"], ["Sensor reader: worker did not stop"])
            self.assertEqual(saved, emitted.call_args.kwargs["summary"])

    def test_main_publishes_after_cleanup_even_when_disconnect_fails(self):
        live, recorder = Mock(), Mock()
        order = []

        def fail_close():
            order.append("disconnect")
            raise RuntimeError("Sensor reader: worker did not stop")

        live.close.side_effect = fail_close
        recorder.publish_subject_summary.side_effect = lambda **_kwargs: order.append("report")
        with (
            patch(
                "sys.argv",
                ["record_cup_trials.py", "--subject-id", "S01", "--raw-only", "--no-depth"],
            ),
            patch("aurora.recording.record_cup_trials.CupSensorInput", return_value=live),
            patch("aurora.recording.record_cup_trials.CupRecorder", return_value=recorder),
            patch("aurora.recording.record_cup_trials.threading.Thread"),
            patch("aurora.recording.record_cup_trials.enable_ctrl_break_as_interrupt"),
            patch("aurora.recording.record_cup_trials.emit"),
            self.assertRaisesRegex(RuntimeError, "worker did not stop"),
        ):
            main()
        self.assertEqual(order, ["disconnect", "report"])
        recorder.publish_subject_summary.assert_called_once_with(
            session_errors=["Camera/Bluetooth: Sensor reader: worker did not stop"]
        )

    def test_summary_delivery_is_protected_from_log_flood_and_reaches_the_gui(self):
        manager = SimpleNamespace()
        ProcessManagerMixin._initialize_process_manager(manager)
        manager._queue_process_event = MethodType(ProcessManagerMixin._queue_process_event, manager)
        for index in range(600):
            manager._queue_process_event(("line", ("job", "job", str(index))))
        child = Mock(
            stdout=io.StringIO('[CUP] {"event":"subject_summary","summary":{}}\n'), stdin=None
        )
        child.wait.return_value = 0
        ProcessManagerMixin._watch_process(
            manager, ManagedProcess("recording:cup_trials", "Cup", "recording", child)
        )
        event, payload = manager.process_exits.get_nowait()
        self.assertEqual(event, "cup_result")
        self.assertEqual(manager.process_exits.get_nowait()[0], "exit")
        gui = SimpleNamespace(
            window=Mock(),
            _append_log=Mock(),
            _update_cup_controls=Mock(),
            _refresh_cup_guidance=Mock(),
        )
        report = {
            "subject_id": "S01",
            "trial_count": 4,
            "complete_trials": 3,
            "trials_with_warnings": 2,
        }
        with patch("aurora.gui.cup_recording.SubjectSummaryWindow") as window:
            CupRecordingMixin._handle_cup_event(
                gui,
                "[CUP] "
                + json.dumps(
                    {"event": "subject_summary", "summary": report, "path": "report.json"}
                ),
            )
        window.assert_called_once_with(gui.window, report, "report.json")
        self.assertIn("subject_summary", payload[2])
        self.assertFalse(manager.log_queue.empty())
        self.assertIsInstance(manager.process_exits, queue.Queue)


class SubjectSummaryWindowTests(unittest.TestCase):
    def test_report_shows_trial_losses_reference_rows_and_selected_error_details(self):
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(str(error))
        root.withdraw()
        self.addCleanup(root.destroy)
        row = dict.fromkeys(COUNTERS, 0)
        row.update(
            id="T001",
            path="T001/trial.json",
            reference=False,
            pace="slow",
            complete=False,
            imu_received=98,
            imu_lost=2,
            issues=["IMU: 2 missing packets (2.00%)"],
        )
        report = {
            "subject_id": "S01",
            "trial_count": 1,
            "complete_trials": 0,
            "trials_with_warnings": 1,
            "reference_count": 0,
            "trial_totals": row,
            "reference_totals": dict.fromkeys(COUNTERS, 0),
            "captures": [row],
            "session_errors": ["Bluetooth cleanup timed out"],
        }
        view = SubjectSummaryWindow(root, report, "report.json")
        view.window.withdraw()
        self.addCleanup(view.window.destroy)
        self.assertIn("2 (2.00%)", view.tree.item("0", "values"))
        self.assertIn("2 missing packets", view.details.get("1.0", "end"))
        self.assertIn("Bluetooth cleanup timed out", view.session_status.cget("text"))


if __name__ == "__main__":
    unittest.main()
