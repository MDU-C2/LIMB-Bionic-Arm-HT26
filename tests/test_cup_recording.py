# Majd
"""Check trial files, calibration reuse and interrupted recordings."""

from __future__ import annotations

import io
import json
import queue
import struct
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
for directory in ("recording", "simulation/ai", "gui"):
    sys.path.insert(0, str(ROOT / "src" / directory))

from ble_packets import HIGH_RATE_EMG_HEADER, HIGH_RATE_IMU_PACKET
from ble_sensor_input import BleSensorReader
from cup_trials import CupTrialSession, next_trial_number
from oak_camera import build_pipeline
from process_manager import ManagedProcess, ProcessManagerMixin
from record_cup_trials import CupRecorder


def feed_samples(reader, sequence=1):
    reader.notification(
        "emg",
        HIGH_RATE_EMG_HEADER.pack(0x101, sequence, sequence * 10_000, 250, 40, 0)
        + struct.pack("<40H", *([2048] * 40)),
    )
    reader.notification(
        "imu",
        HIGH_RATE_IMU_PACKET.pack(
            0x103,
            sequence,
            sequence * 10_000,
            sequence * 10_000,
            0,
            0,
            1000,
            0,
            0,
            0,
            0,
            0,
            1000,
            0,
            0,
            0,
        ),
    )


def prepared_reader():
    reader = BleSensorReader()
    feed_samples(reader)
    reader.emg_estimator.baseline = 2048.0
    reader.emg_estimator.rest_envelope = 2.0
    reader.emg_estimator.reference = 30.0
    reader.emg_estimator.strong_envelope = 33.0
    reader.angles = {"elbow_flexion": 0.0}
    reader.imu_estimator.twist.bias = {role: (0, 0, 0) for role in ("shoulder", "wrist")}
    return reader


class CupRecordingTests(unittest.TestCase):
    def test_marker_keeps_button_time_instead_of_camera_processing_time(self):
        reader = prepared_reader()
        with tempfile.TemporaryDirectory() as temporary:
            trials = CupTrialSession(Path(temporary), "S01", "left", reader, Mock())
            trials.start()
            pressed = time.monotonic_ns()
            trials.mark(2, host_monotonic_ns=pressed)
            self.assertEqual(trials.metadata["operator_events"][-1]["host_monotonic_ns"], pressed)
            with self.assertRaisesRegex(ValueError, "within the active trial"):
                trials.mark(3, host_monotonic_ns=trials.started_ns - 1)
            trials.stop()

    def test_pace_and_operator_markers_are_saved_with_the_trial(self):
        reader = prepared_reader()
        with tempfile.TemporaryDirectory() as temporary:
            trials = CupTrialSession(Path(temporary), "S01", "left", reader, Mock())
            with self.assertRaises(ValueError):
                trials.start("unspecified")
            trials.start("slow")
            trials.mark(2)
            trials.mark(3)
            during = json.loads((trials.path / "trial.json").read_text())
            self.assertFalse(during["complete"])
            self.assertEqual(during["pace_condition"], "slow")
            self.assertEqual([event["marker"] for event in during["operator_events"]], [1, 2, 3])
            trials.mark(4)
            path = trials.stop()
            saved = json.loads((path / "trial.json").read_text())
            self.assertEqual(saved["task_markers"]["4"], "move the empty hand here")
            self.assertEqual([event["marker"] for event in saved["operator_events"]], [1, 2, 3, 4])
            times = [event["host_monotonic_ns"] for event in saved["operator_events"]]
            self.assertEqual(times, sorted(times))

    def test_two_trials_reuse_calibration_and_keep_separate_raw_files(self):
        reader = prepared_reader()
        estimator = reader.emg_estimator
        cv2 = Mock()
        cv2.VideoWriter.return_value.isOpened.return_value = True
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            trials = CupTrialSession(output, "S01", "left", reader, cv2)
            self.assertFalse(trials.directory.exists())
            saved = []
            for number in (1, 2):
                trials.start()
                camera_start = trials.camera.started_host_monotonic_ns
                initial = json.loads((trials.path / "trial.json").read_text())
                self.assertFalse(initial["complete"])
                self.assertEqual(initial["calibration"]["emg_baseline_adc"], 2048)
                feed_samples(reader, number + 1)
                frame = SimpleNamespace(
                    latest_frame=object(),
                    camera_sample=None,
                    camera_observation={
                        "arm_visible": True,
                        "cup_visible": True,
                        "cup_detections": [
                            {"label": "cup", "confidence": 0.8, "bbox_xyxy": [10, 20, 30, 50]}
                        ],
                        "cup_detector": "test_cups",
                        "cup_model_sha256": "abc",
                        "cup_confidence_threshold": 0.25,
                    },
                    frame_time_ns=time.monotonic_ns(),
                )
                trials.add_frame(frame)
                saved.append(trials.stop())
                manifest = json.loads((saved[-1] / "trial.json").read_text())
                self.assertTrue(manifest["complete"])
                self.assertEqual(manifest["trial_id"], f"T{number:03d}")
                self.assertEqual(manifest["packet_counts"], {"emg": 1, "imu": 1})
                self.assertEqual(manifest["calibration"]["emg_reference"], 30)
                self.assertEqual(manifest["camera_observation_frames"], 1)
                self.assertEqual(manifest["cup_model_sha256"], "abc")
                camera_meta = json.loads((saved[-1] / "camera/meta.json").read_text())
                self.assertEqual(camera_meta["started_host_monotonic_ns"], camera_start)
                self.assertEqual(camera_meta["trial_started_host_monotonic_ns"], trials.started_ns)
                self.assertEqual(len((saved[-1] / "ble/emg.csv").read_text().splitlines()), 41)
                self.assertEqual(len((saved[-1] / "ble/packets.jsonl").read_text().splitlines()), 2)
                pose = json.loads((saved[-1] / "camera/pose.json").read_text())
                self.assertEqual(pose["observations"][0]["host_monotonic_ns"], frame.frame_time_ns)
                self.assertEqual(
                    pose["observations"][0]["cup_detections"],
                    frame.camera_observation["cup_detections"],
                )
                self.assertIs(reader.emg_estimator, estimator)
            self.assertNotEqual(*saved)
            self.assertEqual(next_trial_number(output / "S01"), 3)

    def test_uncalibrated_trial_does_not_create_files(self):
        reader = prepared_reader()
        reader.emg_estimator.reference = None
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            trials = CupTrialSession(output, "S01", "left", reader, Mock())
            with self.assertRaisesRegex(RuntimeError, "Calibrate"):
                trials.start()
            self.assertFalse(list(output.iterdir()))

    def test_interrupted_trial_keeps_raw_data_and_marks_the_manifest(self):
        reader = prepared_reader()
        with tempfile.TemporaryDirectory() as temporary:
            trials = CupTrialSession(Path(temporary), "S01", "left", reader, Mock())
            trials.start()
            feed_samples(reader, 2)
            path = trials.stop("Bluetooth interrupted", complete=False)
            data = json.loads((path / "trial.json").read_text())
            self.assertFalse(data["complete"])
            self.assertEqual(data["stop_reason"], "Bluetooth interrupted")
            self.assertGreater((path / "ble/emg.csv").stat().st_size, 0)
            self.assertIsNone(trials.stop())

    def test_video_start_failure_does_not_leave_ble_recording(self):
        reader = prepared_reader()
        cv2 = Mock()
        cv2.VideoWriter.return_value.isOpened.return_value = False
        with tempfile.TemporaryDirectory() as temporary:
            trials = CupTrialSession(Path(temporary), "S01", "left", reader, cv2)
            with self.assertRaisesRegex(RuntimeError, "video.mp4"):
                trials.start()
            self.assertFalse(trials.active)
            self.assertIsNone(reader._recording)
            manifest = json.loads((trials.path / "trial.json").read_text())
            self.assertFalse(manifest["complete"])

    def test_video_start_failure_keeps_the_error_when_an_empty_file_was_created(self):
        reader = prepared_reader()
        cv2 = Mock()
        writer = Mock()
        writer.isOpened.return_value = False

        def failed_writer(filename, *_args):
            Path(filename).touch()
            return writer

        cv2.VideoWriter.side_effect = failed_writer
        with tempfile.TemporaryDirectory() as temporary:
            trials = CupTrialSession(Path(temporary), "S01", "left", reader, cv2)
            with self.assertRaisesRegex(RuntimeError, "Could not open video.mp4"):
                trials.start()
            self.assertIsNone(reader._recording)
            self.assertFalse(trials.active)

    def test_duplicate_start_does_not_replace_a_trial(self):
        reader = prepared_reader()
        with tempfile.TemporaryDirectory() as temporary:
            trials = CupTrialSession(Path(temporary), "S01", "left", reader, Mock())
            trials.start()
            first_path = trials.path
            with self.assertRaises(RuntimeError):
                trials.start()
            self.assertEqual(trials.path, first_path)
            trials.stop(complete=False)

    def test_camera_requires_a_new_frame_after_trial_start(self):
        reader = prepared_reader()
        with tempfile.TemporaryDirectory() as temporary:
            trials = CupTrialSession(Path(temporary), "S01", "left", reader, Mock())
            trials.start()
            trials.add_frame(SimpleNamespace(frame_time_ns=trials.started_ns - 1))
            path = trials.stop()
            manifest = json.loads((path / "trial.json").read_text())
            self.assertFalse(manifest["complete"])
            self.assertEqual(manifest["camera_observation_frames"], 0)

    def test_lost_camera_or_calibration_prevents_start(self):
        live = SimpleNamespace(
            reader=prepared_reader(),
            cv2=Mock(),
            camera_observation={"arm_visible": True},
            frame_time_ns=time.monotonic_ns(),
        )
        with tempfile.TemporaryDirectory() as temporary:
            args = SimpleNamespace(output=Path(temporary), subject="S01", side="left")
            recorder = CupRecorder(args, live)
            self.assertFalse(recorder.state()["ready"])
            recorder.rest_requested = recorder.imu_requested = True
            self.assertTrue(recorder.state()["ready"])
            live.reader.imu_estimator.twist.bias.clear()
            self.assertFalse(recorder.state()["ready"])
            live.reader.imu_estimator.twist.bias = {
                role: (0, 0, 0) for role in ("shoulder", "wrist")
            }
            live.frame_time_ns = 0
            self.assertFalse(recorder.state()["ready"])

    def test_calibration_cannot_change_during_a_trial(self):
        recorder = object.__new__(CupRecorder)
        recorder.trials = SimpleNamespace(active=True)
        recorder.live = Mock()
        with patch("record_cup_trials.emit") as emitted:
            recorder.command("rest")
        recorder.live.reader.calibrate_emg.assert_not_called()
        self.assertEqual(emitted.call_args.args[0], "error")

    def test_closed_app_marks_active_trial_incomplete(self):
        recorder = object.__new__(CupRecorder)
        recorder.stop_trial = Mock()
        recorder.running = True
        recorder.command("connection_closed")
        recorder.stop_trial.assert_called_once_with("App connection closed", complete=False)
        self.assertFalse(recorder.running)

    def test_missing_camera_explains_the_failure(self):
        dai = SimpleNamespace(Pipeline=Mock(side_effect=RuntimeError("No available devices")))
        with self.assertRaisesRegex(RuntimeError, "USB data cable"):
            build_pipeline(dai)

    def test_other_camera_errors_keep_their_original_message(self):
        dai = SimpleNamespace(Pipeline=Mock(side_effect=RuntimeError("USB transfer failed")))
        with self.assertRaisesRegex(RuntimeError, "USB transfer failed"):
            build_pipeline(dai)

    def test_process_output_is_kept_after_the_child_exits(self):
        process = Mock(stdout=io.StringIO("Starting\nRuntimeError: camera unavailable\n"))
        process.wait.return_value = 1
        manager = SimpleNamespace(log_queue=queue.Queue())
        with tempfile.TemporaryDirectory() as temporary:
            managed = ManagedProcess(
                "sim",
                "Live control",
                "simulation",
                process,
                log_path=Path(temporary) / "logs/live.log",
            )
            ProcessManagerMixin._watch_process(manager, managed)
            self.assertIn("camera unavailable", managed.log_path.read_text())
            self.assertEqual(managed.output_tail[-1], "RuntimeError: camera unavailable")
            self.assertEqual(manager.log_queue.get()[0], "line")
            self.assertEqual(manager.log_queue.get()[0], "line")
            self.assertEqual(manager.log_queue.get()[0], "exit")


if __name__ == "__main__":
    unittest.main()
