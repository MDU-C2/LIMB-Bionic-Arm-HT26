# Majd
"""Check trial files, calibration reuse and interrupted recordings."""

from __future__ import annotations

import csv
import io
import json
import os
import queue
import struct
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from aurora.gui.cup_recording import CUP_PROCESS_KEY, CupRecordingMixin
from aurora.gui.process_manager import ManagedProcess, ProcessManagerMixin
from aurora.recording.cup_input import CupCameraFrame, CupDepthFrame, CupSensorInput
from aurora.recording.cup_protocol import REFERENCE_STEPS, CupReferenceGuide, CupTrialGuide
from aurora.recording.cup_trials import CupTrialSession, next_trial_number
from aurora.recording.record_cup_trials import CupRecorder
from aurora.sensors.ble_input import BleSensorReader
from aurora.sensors.ble_protocol import HIGH_RATE_EMG_HEADER, HIGH_RATE_IMU_PACKET
from aurora.sensors.camera.oak_camera import build_pipeline


def feed_samples(reader, sequence=1, emg=None):
    reader.notification(
        "emg",
        HIGH_RATE_EMG_HEADER.pack(0x101, sequence, sequence * 10_000, 250, 40, 0)
        + struct.pack("<40H", *(emg if emg is not None else [2048] * 40)),
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


def recorder_args(**values):
    """Raw-only fixtures isolate file/timer checks; normal collection opts in below."""
    return SimpleNamespace(**{"require_alignment": False, **values})


class CupRecordingTests(unittest.TestCase):
    @unittest.skipUnless(os.name == "nt", "Case-insensitive Windows paths")
    def test_a_subject_case_collision_cannot_mix_participant_identities(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            reader = prepared_reader()
            first = CupTrialSession(root, "S01", "left", reader, Mock())
            first.start()
            first.stop(complete=False)
            before = {path: path.read_bytes() for path in root.rglob("*") if path.is_file()}
            with self.assertRaisesRegex(ValueError, "exact subject ID"):
                CupTrialSession(root, "s01", "left", reader, Mock())
            self.assertEqual(
                {path: path.read_bytes() for path in root.rglob("*") if path.is_file()}, before
            )

    def test_a_subject_id_cannot_be_silently_changed_to_another_folder_label(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with self.assertRaisesRegex(ValueError, "subject ID"):
                CupTrialSession(root, "A B", "left", prepared_reader(), Mock())
            self.assertFalse(list(root.iterdir()))

    def test_failed_ble_stop_closes_camera_and_keeps_trial_for_cleanup_retry(self):
        reader = prepared_reader()
        with tempfile.TemporaryDirectory() as temporary:
            trial = CupTrialSession(Path(temporary), "S01", "left", reader, Mock())
            trial.start()
            camera = trial.camera
            real_stop = reader.stop_recording
            try:
                with (
                    patch.object(
                        reader, "stop_recording", side_effect=RuntimeError("writer timeout")
                    ),
                    self.assertRaisesRegex(RuntimeError, "writer timeout"),
                ):
                    trial.stop()
                self.assertTrue(trial.active)
                self.assertFalse(trial.recording)
                self.assertTrue(trial.stopping)
                self.assertFalse(camera.recording)
                manifest = json.loads((trial.path / "trial.json").read_text())
                self.assertFalse(manifest["complete"])
                self.assertEqual(manifest["status"], "incomplete")
                self.assertIn("writer timeout", manifest["shutdown_error"])
                with self.assertRaisesRegex(RuntimeError, "current trial"):
                    trial.start()
                original_end = manifest["recording_ended_host_monotonic_ns"]
                trial.stop()
                self.assertFalse(trial.active)
                final = json.loads((trial.path / "trial.json").read_text())
                self.assertEqual(final["recording_ended_host_monotonic_ns"], original_end)
                self.assertFalse(final["complete"])
            finally:
                real_stop()

    def test_calibration_can_start_during_a_tracking_miss_but_trials_still_need_alignment(self):
        live = SimpleNamespace(
            reader=prepared_reader(),
            cv2=Mock(),
            frame_time_ns=time.monotonic_ns(),
            camera_metadata=lambda: {"stereo_depth_enabled": True},
            stereo_status=lambda: {
                "ready": False,
                "capture_ready": True,
                "message": "No 3D point: left wrist",
            },
            depth_receiving=lambda: True,
        )
        with tempfile.TemporaryDirectory() as temporary:
            recorder = CupRecorder(
                recorder_args(
                    output=Path(temporary), subject="S01", side="left", require_alignment=True
                ),
                live,
            )
            state = recorder.state()
            self.assertTrue(state["can_prepare"])
            self.assertFalse(state["can_record"])
            self.assertIn("left wrist", state["message"])
            with patch("aurora.recording.record_cup_trials.emit"):
                recorder.command("record")
                self.assertFalse(recorder.guide.active)
                recorder.command("reference")
                self.assertTrue(recorder.trials.active)
                recorder.stop_trial("Test cancelled", complete=False)

    def test_slow_tracking_never_replaces_a_newer_preview_frame(self):
        live = SimpleNamespace(
            camera_frame=CupCameraFrame(20, 2_000_000_000, object()),
            tracking_frame=CupCameraFrame(10, 1_700_000_000, object()),
        )
        recorder = object.__new__(CupRecorder)
        recorder.live = live
        self.assertIs(recorder.preview_frame(), live.camera_frame)
        live.tracking_frame = CupCameraFrame(15, 1_900_000_000, object())
        self.assertIs(recorder.preview_frame(), live.camera_frame)

    def test_reconnect_during_a_trial_cannot_export_data_with_the_previous_alignment(self):
        from test_arm_alignment import generated_reference

        from aurora.sensors.arm_alignment import fit_alignment

        stages, camera, _, _ = generated_reference()
        profile = fit_alignment(stages, camera, side="left")
        with tempfile.TemporaryDirectory() as temporary:
            reader = prepared_reader()
            trials = CupTrialSession(Path(temporary), "S01", "left", reader, Mock())
            trials.start(alignment_profile=profile)
            feed_samples(reader, 2)
            trials.add_frame(
                SimpleNamespace(
                    latest_frame=object(),
                    camera_sample=None,
                    camera_observation={},
                    frame_time_ns=time.monotonic_ns(),
                )
            )
            trials.metadata["subject_reference_invalidated"] = True
            path = trials.stop()
            document = json.loads((path / "trial.json").read_text())
            self.assertFalse(document["complete"])
            self.assertIn("calibration invalidated", document["alignment_export_error"])
            self.assertTrue((path / "ble/imu.csv").is_file())
            self.assertFalse((path / "ble/imu_aligned.csv").exists())

    def test_stereo_loss_or_reconnect_during_countdown_cannot_start_a_normal_trial(self):
        from test_arm_alignment import generated_reference

        from aurora.sensors.arm_alignment import fit_alignment

        stages, camera, _, _ = generated_reference()
        profile = fit_alignment(stages, camera, side="left")
        profile.update(connection_generation=0, subject_id="S01")
        for interruption in ("depth", "reconnect"):
            with self.subTest(interruption=interruption):
                now = [10_000_000_000]
                depth_ready = [True]
                with (
                    patch("time.monotonic_ns", side_effect=lambda now=now: now[0]),
                    patch("time.monotonic", side_effect=lambda now=now: now[0] / 1e9),
                    tempfile.TemporaryDirectory() as temporary,
                    patch("aurora.recording.record_cup_trials.emit"),
                ):
                    live = SimpleNamespace(
                        reader=prepared_reader(),
                        cv2=Mock(),
                        camera_observation={},
                        frame_time_ns=now[0],
                        stereo_status=lambda depth_ready=depth_ready: {"ready": depth_ready[0]},
                    )
                    recorder = CupRecorder(
                        recorder_args(
                            output=Path(temporary),
                            subject="S01",
                            side="left",
                            require_alignment=True,
                        ),
                        live,
                    )
                    recorder.alignment_profile = profile
                    recorder.reference_checks = dict.fromkeys(
                        (
                            "sensor_to_arm_ready",
                            "imu_zero_ready",
                            "emg_rest_ready",
                            "emg_grip_ready",
                        ),
                        True,
                    )
                    recorder.command("record")
                    self.assertEqual(recorder.guide.phase, "countdown")
                    now[0] += 3_000_000_000
                    live.frame_time_ns = now[0]
                    feed_samples(live.reader, 2)
                    if interruption == "depth":
                        depth_ready[0] = False
                    else:
                        live.reader.connection_generation += 1
                        recorder.monitor_streams()
                    recorder.advance_protocol()
                    self.assertFalse(recorder.trials.active)
                    self.assertEqual(recorder.guide.phase, "ready")
                    self.assertFalse(list(Path(temporary).iterdir()))

    def test_guided_subject_fit_enables_a_trial_exports_aligned_data_and_invalidates_on_reconnect(
        self,
    ):
        import math

        import numpy as np
        from test_arm_alignment import generated_reference

        stages, camera, expected_mounting, _ = generated_reference()
        now = [10_000_000_000]
        sequence = [1]
        with (
            patch("time.monotonic_ns", side_effect=lambda: now[0]),
            patch("time.monotonic", side_effect=lambda: now[0] / 1e9),
            tempfile.TemporaryDirectory() as temporary,
            patch("aurora.recording.record_cup_trials.emit"),
        ):
            live = CupSensorInput(
                side="left", device="generated", shoulder_twist_axis="y", wrist_twist_axis="y"
            )
            live.reader, live.cv2 = prepared_reader(), Mock()
            live.cv2.imencode.return_value = (True, np.array([0], dtype=np.uint8))
            live.intrinsics = [[500, 0, 320], [0, 500, 240], [0, 0, 1]]
            live.reader.clock_sync = Mock()
            live.reader.clock_sync.to_host_ns.side_effect = lambda stamp: (
                stamp * 1000 + 10_000_000_000
            )
            live.reader.clock_sync.summary.return_value = {"available": True}
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            depth = np.full((4, 4), 1600, dtype=np.uint16)

            def publish_camera(stage="STILL", index=0):
                points = camera[stage][min(index // 20, len(camera[stage]) - 1)]
                frame = CupCameraFrame(
                    sequence[0],
                    now[0],
                    image,
                    camera_sample={
                        "keypoints_camera_m": {key: value.tolist() for key, value in points.items()}
                    },
                    camera_observation={"depth_time_difference_ns": 0, "arm_visible": True},
                    device_time_ns=now[0],
                )
                live._publish_frame(frame)
                live._publish_depth(CupDepthFrame(sequence[0] // 4, now[0], now[0], depth))
                live.observations.append(frame)
                live.tracking_frame = frame

            publish_camera()
            recorder = CupRecorder(
                recorder_args(
                    output=Path(temporary),
                    subject="S01",
                    side="left",
                    require_alignment=True,
                    rest=0,
                ),
                live,
            )
            self.assertTrue(recorder.state()["can_prepare"])
            self.assertFalse(recorder.state()["can_record"])
            recorder.command("record")
            self.assertFalse(recorder.guide.active)

            def tick():
                if (
                    isinstance(recorder.guide, CupReferenceGuide)
                    and recorder.guide.phase == "reference_prepare"
                ):
                    recorder.command("reference_step")
                now[0] += 10_000_000
                sequence[0] += 1
                elapsed = (now[0] - recorder.guide.phase_started_ns) / 1e9
                cue = (
                    REFERENCE_STEPS[recorder.guide.step][0]
                    if isinstance(recorder.guide, CupReferenceGuide)
                    and recorder.guide.phase == "reference"
                    else "STILL"
                )
                stage = cue if cue in stages else "STILL"
                index = min(round(elapsed * 100), len(stages[stage]["shoulder"].time_s) - 1)
                values = []
                for role in ("shoulder", "wrist"):
                    series = stages[stage][role]
                    values.extend(round(value * 1000) for value in series.accel_g[index])
                    values.extend(round(value * 100) for value in series.gyro_dps[index])
                device_us = (now[0] - 10_000_000_000) // 1000
                amplitude = 180 if cue == "GRIP" else 3
                emg = [
                    round(2048 + amplitude * math.sin(2 * math.pi * 100 * i / 4000))
                    for i in range(40)
                ]
                live.reader.notification(
                    "emg",
                    HIGH_RATE_EMG_HEADER.pack(0x101, sequence[0], device_us - 9750, 250, 40, 0)
                    + struct.pack("<40H", *emg),
                )
                live.reader.notification(
                    "imu",
                    HIGH_RATE_IMU_PACKET.pack(0x103, sequence[0], device_us, device_us, *values),
                )
                if sequence[0] % 100 == 0:
                    live.reader.flush_recording()
                if sequence[0] % 4 == 0:
                    publish_camera(stage, index)
                recorder.collect_frames()
                recorder.monitor_streams()
                recorder.advance_protocol()

            recorder.command("reference")
            try:
                for _ in range(8200):
                    tick()
                    if not isinstance(recorder.guide, CupReferenceGuide):
                        break
                self.assertFalse(recorder.trials.active)
                self.assertTrue(recorder.state()["can_record"], recorder.state()["message"])
                for role in expected_mounting:
                    np.testing.assert_allclose(
                        recorder.alignment_profile["sensor_to_segment"][role],
                        expected_mounting[role],
                        atol=0.002,
                    )
                reference = recorder.trials.path
                original = (reference / "trial.json").read_bytes()
                recorder.command({"action": "record", "condition": "natural"})
                for _ in range(1150):
                    tick()
                    if not recorder.guide.active:
                        break
                manifest = json.loads((recorder.trials.path / "trial.json").read_text())
                self.assertTrue(manifest["complete"], manifest.get("alignment_export_error"))
                self.assertEqual(manifest["sensor_to_arm_alignment"], "fitted")
                self.assertGreater(manifest["aligned_imu"]["rows"], 1500)
                self.assertTrue((recorder.trials.path / "ble/imu_aligned.csv").is_file())
                self.assertEqual(original, (reference / "trial.json").read_bytes())
                live.reader.connection_generation += 1
                recorder.monitor_streams()
                self.assertIsNone(recorder.alignment_profile)
                self.assertFalse(recorder.state()["can_record"])
                self.assertEqual(recorder.state()["next_trial"], "T002")
            finally:
                recorder.stop_trial(complete=False)

    def test_slow_trial_can_extend_the_window_without_reconnecting(self):
        now = [10_000_000_000]
        with (
            patch("time.monotonic_ns", side_effect=lambda: now[0]),
            patch("time.monotonic", side_effect=lambda: now[0] / 1e9),
            tempfile.TemporaryDirectory() as temporary,
            patch("aurora.recording.record_cup_trials.emit"),
        ):
            reader = prepared_reader()
            live = SimpleNamespace(
                reader=reader, cv2=Mock(), camera_observation={}, frame_time_ns=now[0]
            )
            recorder = CupRecorder(
                recorder_args(output=Path(temporary), subject="S01", side="left", countdown=0),
                live,
            )
            recorder.command({"action": "record", "condition": "slow", "duration": 12})
            recorder.advance_protocol()
            feed_samples(reader, 2)
            recorder.trials.add_frame(
                SimpleNamespace(
                    latest_frame=object(),
                    camera_sample=None,
                    camera_observation={},
                    frame_time_ns=now[0],
                )
            )
            now[0] += 8_000_000_000
            recorder.advance_protocol()
            self.assertTrue(recorder.trials.active)
            now[0] += 4_000_000_000
            recorder.advance_protocol()
            manifest = json.loads((recorder.trials.path / "trial.json").read_text())
            self.assertTrue(manifest["complete"])
            self.assertEqual(manifest["protocol"]["recording_seconds"], 12)
            self.assertEqual(manifest["pace_condition"], "slow")
            self.assertEqual(reader.connection_generation, 0)

    def test_preparation_runs_each_calibration_once_and_saves_the_result(self):
        import math

        now = [10_000_000_000]
        with (
            patch("time.monotonic_ns", side_effect=lambda: now[0]),
            patch("time.monotonic", side_effect=lambda: now[0] / 1e9),
            tempfile.TemporaryDirectory() as temporary,
            patch("aurora.recording.record_cup_trials.emit"),
        ):
            reader = prepared_reader()
            live = SimpleNamespace(
                reader=reader, cv2=Mock(), camera_observation={}, frame_time_ns=now[0]
            )
            recorder = CupRecorder(
                recorder_args(output=Path(temporary), subject="S01", side="left"), live
            )
            with (
                patch.object(reader, "calibrate", wraps=reader.calibrate) as imu,
                patch.object(reader, "calibrate_emg", wraps=reader.calibrate_emg) as rest,
                patch.object(
                    reader, "calibrate_emg_strength", wraps=reader.calibrate_emg_strength
                ) as grip,
            ):
                recorder.command("reference")
                for sequence in range(2, 8300):
                    if not isinstance(recorder.guide, CupReferenceGuide):
                        break
                    if recorder.guide.phase == "reference_prepare":
                        recorder.command("reference_step")
                    now[0] += 10_000_000
                    cue = (
                        REFERENCE_STEPS[recorder.guide.step][0]
                        if recorder.guide.phase == "reference"
                        else ""
                    )
                    amplitude = 180 if cue == "GRIP" else 3
                    samples = [
                        round(2048 + amplitude * math.sin(2 * math.pi * 100 * index / 4000))
                        for index in range(40)
                    ]
                    feed_samples(reader, sequence, samples)
                    if sequence % 100 == 0:
                        reader.flush_recording()
                    live.frame_time_ns = now[0]
                    if sequence % 4 == 0:
                        recorder.trials.add_frame(
                            SimpleNamespace(
                                latest_frame=object(),
                                camera_sample=None,
                                camera_observation={},
                                frame_time_ns=now[0],
                            )
                        )
                    recorder.advance_protocol()
                imu.assert_called_once_with(reset_emg=False)
                rest.assert_called_once_with()
                grip.assert_called_once_with()
            self.assertFalse(recorder.trials.active)
            self.assertTrue(recorder.reference_path)
            reference = json.loads((recorder.trials.path / "trial.json").read_text())
            self.assertTrue(reference["complete"])
            checks = reference["reference_checks"]
            self.assertTrue(checks["imu_zero_ready"])
            self.assertTrue(checks["emg_rest_ready"])
            self.assertTrue(checks["emg_grip_ready"], checks["emg_message"])
            self.assertGreater(reference["calibration_after"]["emg_reference"], 0)
            self.assertEqual(reference["sensor_to_arm_alignment"], "failed")
            self.assertFalse(checks["sensor_to_arm_ready"])

    def test_practice_ends_after_the_window_without_saving_or_consuming_an_id(self):
        now = [10_000_000_000]
        with (
            patch("time.monotonic_ns", side_effect=lambda: now[0]),
            patch("time.monotonic", side_effect=lambda: now[0] / 1e9),
            tempfile.TemporaryDirectory() as temporary,
            patch("aurora.recording.record_cup_trials.emit"),
        ):
            live = SimpleNamespace(
                reader=prepared_reader(), cv2=Mock(), camera_observation={}, frame_time_ns=now[0]
            )
            recorder = CupRecorder(
                recorder_args(output=Path(temporary), subject="S01", side="left"), live
            )
            recorder.command("practice")
            now[0] += 3_000_000_000
            recorder.advance_protocol()
            self.assertEqual(recorder.guide.phase, "practice")
            now[0] += 8_000_000_000
            recorder.advance_protocol()
            self.assertEqual(recorder.guide.phase, "rest")
            self.assertIsNone(live.reader._recording)
            self.assertFalse(list(Path(temporary).iterdir()))
            self.assertEqual(recorder.state()["next_trial"], "T001")

    def test_stream_loss_during_countdown_cancels_at_go_without_creating_files(self):
        now = [10_000_000_000]
        with (
            patch("time.monotonic_ns", side_effect=lambda: now[0]),
            patch("time.monotonic", side_effect=lambda: now[0] / 1e9),
            tempfile.TemporaryDirectory() as temporary,
            patch("aurora.recording.record_cup_trials.emit"),
        ):
            live = SimpleNamespace(
                reader=prepared_reader(), cv2=Mock(), camera_observation={}, frame_time_ns=now[0]
            )
            recorder = CupRecorder(
                recorder_args(output=Path(temporary), subject="S01", side="left"), live
            )
            recorder.command("record")
            now[0] += 3_000_000_000
            recorder.advance_protocol()
            self.assertEqual(recorder.guide.phase, "ready")
            self.assertFalse(recorder.trials.active)
            self.assertFalse(list(Path(temporary).iterdir()))

    def test_default_countdown_then_eight_seconds_autosave_without_movement_labels(self):
        now = [10_000_000_000]
        with (
            patch("time.monotonic_ns", side_effect=lambda: now[0]),
            patch("time.monotonic", side_effect=lambda: now[0] / 1e9),
        ):
            reader = prepared_reader()
            live = SimpleNamespace(
                reader=reader, cv2=Mock(), camera_observation={}, frame_time_ns=now[0]
            )
            with (
                tempfile.TemporaryDirectory() as temporary,
                patch("aurora.recording.record_cup_trials.emit"),
            ):
                recorder = CupRecorder(
                    recorder_args(output=Path(temporary), subject="S01", side="left"), live
                )
                recorder.command("record")
                self.assertEqual(recorder.guide.view()["cue"], "3")
                self.assertIsNone(reader._recording)
                now[0] += 2_999_000_000
                recorder.advance_protocol()
                self.assertIsNone(reader._recording)
                now[0] += 1_000_000
                live.frame_time_ns = now[0]
                feed_samples(reader, 2)
                recorder.advance_protocol()
                self.assertTrue(recorder.trials.active)
                self.assertEqual(recorder.guide.view()["cue"], "GO")
                go = recorder.guide.go_ns
                feed_samples(reader, 3)
                recorder.trials.add_frame(
                    SimpleNamespace(
                        latest_frame=object(),
                        camera_sample=None,
                        camera_observation={},
                        frame_time_ns=now[0],
                    )
                )
                now[0] = go + 7_999_000_000
                recorder.advance_protocol()
                self.assertTrue(recorder.trials.active)
                now[0] += 1_000_000
                recorder.advance_protocol()
                self.assertFalse(recorder.trials.active)
                manifest = json.loads((recorder.trials.path / "trial.json").read_text())
                self.assertTrue(manifest["complete"])
                self.assertEqual(manifest["protocol"]["recording_seconds"], 8)
                self.assertEqual(manifest["movement_duration_seconds"], 8)
                self.assertEqual(manifest["operator_events"], [])
                self.assertFalse(manifest["task_completion_verified"])
                self.assertEqual(manifest["stop_reason"], "Recording window finished")
                self.assertTrue(
                    any(
                        event["time_s"] < 0 and event["phase"] == "countdown"
                        for event in manifest["cue_events"]
                    )
                )
                self.assertEqual(recorder.saved_counts["natural"], 1)

    def test_cancel_before_go_creates_no_trial_and_does_not_consume_an_id(self):
        live = SimpleNamespace(
            reader=prepared_reader(),
            cv2=Mock(),
            camera_observation={},
            frame_time_ns=time.monotonic_ns(),
        )
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch("aurora.recording.record_cup_trials.emit"),
        ):
            recorder = CupRecorder(
                recorder_args(output=Path(temporary), subject="S01", side="left"), live
            )
            recorder.command("record")
            recorder.command("stop")
            self.assertEqual(recorder.guide.phase, "ready")
            self.assertEqual(recorder.state()["next_trial"], "T001")
            self.assertFalse(list(Path(temporary).iterdir()))

    def test_early_stop_keeps_the_partial_window_incomplete(self):
        live = SimpleNamespace(
            reader=prepared_reader(),
            cv2=Mock(),
            camera_observation={},
            frame_time_ns=time.monotonic_ns(),
        )
        with (
            tempfile.TemporaryDirectory() as temporary,
            patch("aurora.recording.record_cup_trials.emit"),
        ):
            recorder = CupRecorder(
                recorder_args(output=Path(temporary), subject="S01", side="left", countdown=0),
                live,
            )
            recorder.command("record")
            recorder.advance_protocol()
            feed_samples(live.reader, 2)
            recorder.trials.add_frame(
                SimpleNamespace(
                    latest_frame=object(),
                    camera_sample=None,
                    camera_observation={},
                    frame_time_ns=time.monotonic_ns(),
                )
            )
            recorder.command("stop")
            manifest = json.loads((recorder.trials.path / "trial.json").read_text())
            self.assertFalse(manifest["complete"])
            self.assertEqual(recorder.saved_counts["natural"], 0)
            self.assertGreater(manifest["packet_counts"]["emg"], 0)

    def test_reconnection_does_not_reuse_the_old_clock_fit_in_saved_packets(self):
        reader = prepared_reader()
        reader.clock_sync = Mock()
        reader.clock_sync.to_host_ns.side_effect = lambda stamp: stamp * 1000 + 9_000_000_000
        reader.clock_sync.summary.return_value = {"available": True}
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary)
            reader.start_recording(session)
            feed_samples(reader, 2)
            reader._start_connection()
            feed_samples(reader, 1)
            health = reader.stop_recording()
            packets = [
                json.loads(line) for line in (session / "packets.jsonl").read_text().splitlines()
            ]
            self.assertTrue(all("synced_host_monotonic_ns" in packet for packet in packets[:2]))
            self.assertTrue(all("synced_host_monotonic_ns" not in packet for packet in packets[2:]))
            self.assertEqual([packet["connection_generation"] for packet in packets], [0, 0, 1, 1])
            self.assertEqual(
                [epoch["connection_generation"] for epoch in health["time_sync_epochs"]], [0, 1]
            )
            self.assertEqual(health["packet_health"]["emg"]["missing_packets"], 0)
            with (session / "emg.csv").open(newline="") as source:
                samples = list(csv.DictReader(source))
            self.assertTrue(samples[0]["synced_host_monotonic_ns"])
            self.assertEqual(samples[-1]["synced_host_monotonic_ns"], "")

    def test_recorder_loop_keeps_raw_video_and_ble_through_tracking_failure_and_stream_gap(self):
        import numpy as np

        live = CupSensorInput(
            side="left", device="test", shoulder_twist_axis="y", wrist_twist_axis="y"
        )
        live.reader = prepared_reader()
        live.cv2 = Mock()
        live.cv2.WND_PROP_VISIBLE = 1
        live.cv2.waitKey.return_value = -1
        live.cv2.getWindowProperty.return_value = 1
        live.tracking_error = "Tracker unavailable"
        commands = queue.Queue()
        iteration = [0]

        def poll():
            iteration[0] += 1
            sequence = iteration[0]
            feed_samples(live.reader, sequence + 1)
            if sequence == 4:
                live.reader.emg_time = 0
            if sequence == 5:
                live.reader.connection_generation += 1
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            live._publish_frame(CupCameraFrame(sequence, time.monotonic_ns(), image))
            for tick, action in ((1, "record"), (6, "stop"), (7, "close")):
                if sequence == tick:
                    commands.put(action)
            return live.capture_health()

        live.poll = poll
        with tempfile.TemporaryDirectory() as temporary:
            args = recorder_args(
                output=Path(temporary),
                subject="S01",
                side="left",
                duration=0,
                preview_position=None,
                baseline=0,
                countdown=0,
                rest=0,
            )
            recorder = CupRecorder(args, live)
            recorder.reference_path = "references/R001_previous"
            with patch("aurora.recording.record_cup_trials.emit"):
                recorder.run(commands)
            manifest = json.loads((recorder.trials.path / "trial.json").read_text())
            self.assertFalse(manifest["complete"])
            self.assertTrue(manifest["subject_reference_invalidated"])
            self.assertIsNone(recorder.reference_path)
            self.assertGreaterEqual(manifest["camera_source_frame_count"], 5)
            self.assertEqual(manifest["camera_observation_frame_count"], 0)
            self.assertGreaterEqual(manifest["packet_counts"]["emg"], 5)
            self.assertEqual(manifest["capture_health"]["tracking_error"], "Tracker unavailable")
            cues = [event["cue"] for event in manifest["cue_events"]]
            self.assertIn("stream_gap", cues)
            self.assertIn("streams_recovered", cues)
            self.assertIn("ble_reconnected", cues)
            times = (recorder.trials.path / "camera/frame_times.jsonl").read_text().splitlines()
            self.assertEqual(len(times), manifest["camera_source_frame_count"])

    def test_reference_capture_links_later_trials_without_claiming_alignment(self):
        reader = prepared_reader()
        live = SimpleNamespace(
            reader=reader, cv2=Mock(), camera_observation={}, frame_time_ns=time.monotonic_ns()
        )
        with tempfile.TemporaryDirectory() as temporary:
            recorder = CupRecorder(
                recorder_args(output=Path(temporary), subject="S01", side="left"), live
            )
            with patch("aurora.recording.record_cup_trials.emit"):
                recorder.command("reference")
                self.assertIsInstance(recorder.guide, CupReferenceGuide)
                first = recorder.trials.path
                self.assertEqual(first.parent.name, "references")
                self.assertEqual(
                    recorder.trials.metadata["sensor_to_arm_alignment"], "not_estimated"
                )
                self.assertEqual(recorder.trials.metadata["operator_events"], [])
                feed_samples(reader, 2)
                recorder.trials.add_frame(
                    SimpleNamespace(
                        latest_frame=object(),
                        camera_sample=None,
                        camera_observation={},
                        frame_time_ns=time.monotonic_ns(),
                    )
                )
                recorder.guide.completed = True
                recorder.stop_trial("Reference sequence finished")
                reference = json.loads((first / "trial.json").read_text())
                self.assertTrue(reference["complete"])
                self.assertIsNone(reference["pace_condition"])
                self.assertEqual(reference["test_type"], "subject_reference")
                self.assertEqual(recorder.saved_counts, {"natural": 0, "slow": 0, "fast": 0})
                saved_reference = (first / "trial.json").read_bytes()
                recorder.guide = CupTrialGuide(countdown_s=0)
                recorder.command({"action": "record", "condition": "fast"})
                recorder.advance_protocol()
                self.assertEqual(recorder.trials.metadata["trial_id"], "T001")
                self.assertEqual(
                    recorder.trials.metadata["subject_reference"], recorder.reference_path
                )
                self.assertEqual(recorder.trials.directory / recorder.reference_path, first)
                recorder.stop_trial(complete=False)
                self.assertEqual((first / "trial.json").read_bytes(), saved_reference)

    def test_cancelled_reference_does_not_enable_an_alignment_reference(self):
        live = SimpleNamespace(
            reader=prepared_reader(),
            cv2=Mock(),
            camera_observation={},
            frame_time_ns=time.monotonic_ns(),
        )
        with tempfile.TemporaryDirectory() as temporary:
            recorder = CupRecorder(
                recorder_args(output=Path(temporary), subject="S01", side="left"), live
            )
            with patch("aurora.recording.record_cup_trials.emit"):
                recorder.command("reference")
                path = recorder.trials.path
                recorder.command("stop")
                self.assertIsNone(recorder.reference_path)
                self.assertEqual(recorder.guide.phase, "ready")
                self.assertFalse(json.loads((path / "trial.json").read_text())["complete"])

    def test_stream_interruption_keeps_other_packets_and_marks_attempt_incomplete(self):
        reader = prepared_reader()
        with tempfile.TemporaryDirectory() as temporary:
            trials = CupTrialSession(Path(temporary), "S01", "left", reader, Mock())
            trials.start()
            trials.metadata["stream_interrupted"] = True
            trials.cue("stream_gap")
            feed_samples(reader, 2)
            trials.add_frame(
                SimpleNamespace(
                    latest_frame=object(),
                    camera_sample=None,
                    camera_observation={},
                    frame_time_ns=time.monotonic_ns(),
                )
            )
            path = trials.stop()
            manifest = json.loads((path / "trial.json").read_text())
            self.assertFalse(manifest["complete"])
            self.assertEqual(manifest["packet_counts"], {"emg": 1, "imu": 1})
            self.assertEqual(manifest["cue_events"][0]["cue"], "stream_gap")
            self.assertGreater((path / "ble/emg.csv").stat().st_size, 0)

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
            self.assertEqual([event["marker"] for event in during["operator_events"]], [2, 3])
            trials.mark(4)
            path = trials.stop()
            saved = json.loads((path / "trial.json").read_text())
            self.assertEqual(saved["task_markers"]["4"], "move the empty hand here")
            self.assertEqual([event["marker"] for event in saved["operator_events"]], [2, 3, 4])
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
                self.assertEqual(manifest["subject_id"], "S01")
                self.assertEqual(manifest["format_version"], 3)
                self.assertEqual(manifest["trial_id"], f"T{number:03d}")
                self.assertEqual(manifest["packet_counts"], {"emg": 1, "imu": 1})
                self.assertEqual(manifest["calibration"]["emg_reference"], 30)
                self.assertEqual(manifest["camera_observation_frame_count"], 1)
                self.assertEqual(manifest["cup_model_sha256"], "abc")
                camera_meta = json.loads((saved[-1] / "camera/meta.json").read_text())
                self.assertEqual(camera_meta["started_host_monotonic_ns"], camera_start)
                self.assertEqual(camera_meta["trial_started_host_monotonic_ns"], trials.started_ns)
                self.assertEqual(len((saved[-1] / "ble/emg.csv").read_text().splitlines()), 41)
                self.assertEqual(len((saved[-1] / "ble/packets.jsonl").read_text().splitlines()), 2)
                pose = json.loads((saved[-1] / "camera/pose.json").read_text())
                self.assertEqual(pose["subject_id"], manifest["subject_id"])
                self.assertEqual(pose["trial_id"], manifest["trial_id"])
                self.assertEqual(pose["pose_frame_count"], len(pose["data"]))
                self.assertEqual(pose["observation_frame_count"], len(pose["observations"]))
                self.assertGreater(pose["video_frame_count"], 0)
                self.assertEqual(pose["coordinate_system"], "rgb_pixels_stereo_depth")
                self.assertEqual(pose["observations"][0]["host_monotonic_ns"], frame.frame_time_ns)
                self.assertEqual(
                    pose["observations"][0]["cup_detections"],
                    frame.camera_observation["cup_detections"],
                )
                self.assertIs(reader.emg_estimator, estimator)
            self.assertNotEqual(*saved)
            self.assertEqual(next_trial_number(output / "S01"), 3)

    def test_uncalibrated_trial_saves_raw_data_and_calibration_status(self):
        reader = prepared_reader()
        reader.emg_estimator.reset()
        reader.imu_estimator.reset_calibration()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            trials = CupTrialSession(output, "S01", "left", reader, Mock())
            trials.start()
            feed_samples(reader, 2)
            trials.add_frame(
                SimpleNamespace(
                    latest_frame=object(),
                    camera_sample=None,
                    camera_observation={"arm_visible": False, "cup_visible": False},
                    frame_time_ns=time.monotonic_ns(),
                )
            )
            path = trials.stop()
            data = json.loads((path / "trial.json").read_text())
            self.assertTrue(data["complete"])
            self.assertEqual(data["recording_mode"], "raw_signals")
            for field in ("imu_bias_ready", "emg_rest_ready", "emg_grip_ready"):
                self.assertFalse(data["calibration_status"][field])
            self.assertIsNone(data["calibration"]["emg_reference"])
            self.assertEqual(len((path / "ble/emg.csv").read_text().splitlines()), 41)
            self.assertEqual(len((path / "ble/packets.jsonl").read_text().splitlines()), 2)

    def test_missing_raw_stream_prevents_trial_files(self):
        reader = prepared_reader()
        reader._emg_history.clear()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            trials = CupTrialSession(output, "S01", "left", reader, Mock())
            with self.assertRaisesRegex(RuntimeError, "EMG and both IMUs"):
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
            self.assertEqual(manifest["camera_observation_frame_count"], 0)

    def test_raw_recording_requires_frames_and_streams_but_not_calibration_or_tracking(self):
        live = SimpleNamespace(
            reader=prepared_reader(),
            cv2=Mock(),
            camera_observation={"arm_visible": True},
            frame_time_ns=time.monotonic_ns(),
        )
        with tempfile.TemporaryDirectory() as temporary:
            args = recorder_args(output=Path(temporary), subject="S01", side="left")
            recorder = CupRecorder(args, live)
            self.assertTrue(recorder.state()["ready"])
            live.reader.imu_estimator.twist.bias.clear()
            live.reader.emg_estimator.reference = None
            live.camera_observation = {"arm_visible": False, "cup_visible": False}
            self.assertTrue(recorder.state()["ready"])
            live.frame_time_ns = 0
            self.assertFalse(recorder.state()["ready"])

    def test_gui_record_button_uses_raw_streams_and_protocol_state(self):
        child = Mock()
        child.poll.return_value = None
        managed = SimpleNamespace(key=CUP_PROCESS_KEY, process=child)
        from aurora.gui.session import CupViewState

        state = CupViewState()
        state.update(
            dict(
                ready=True,
                streams_ready=True,
                recording=False,
                can_start=True,
                can_record=True,
                can_prepare=True,
            )
        )
        gui = SimpleNamespace(
            cup_state=state,
            processes={CUP_PROCESS_KEY: managed},
            _active_processes=lambda: [managed],
            _ble_handoff_pending=False,
            _cup_camera_handoff_pending=False,
            cup_fields=[],
            cup_condition_field=Mock(),
            cup_next_trial=Mock(),
            cup_duration=Mock(get=lambda: "8"),
            cup_connect_button=Mock(),
            cup_record_button=Mock(),
            cup_stop_button=Mock(),
            cup_finish_button=Mock(),
            cup_graphs_button=Mock(),
            recording_advanced_button=Mock(),
            cup_retry_button=Mock(),
            cup_practice_button=Mock(),
        )
        CupRecordingMixin._update_cup_controls(gui)
        self.assertEqual(gui.cup_record_button.configure.call_args.kwargs["state"], "normal")
        CupRecordingMixin._update_cup_controls(gui)
        gui.cup_state.status["recording"] = True
        gui.cup_state.phase = "movement"
        CupRecordingMixin._update_cup_controls(gui)
        self.assertEqual(gui.cup_record_button.configure.call_args.kwargs["state"], "disabled")
        self.assertEqual(gui.cup_stop_button.configure.call_args.kwargs["state"], "normal")
        gui.processes.clear()
        CupRecordingMixin._update_cup_controls(gui)
        self.assertEqual(gui.cup_stop_button.configure.call_args.kwargs["state"], "disabled")

    def test_signals_button_creates_one_graph_window_and_reuses_it(self):
        recorder = object.__new__(CupRecorder)
        recorder.graphs = recorder.graph_root = None
        with (
            patch("tkinter.Tk") as root,
            patch("aurora.gui.sensor_graphs.LiveSignalGraphs") as graphs,
        ):
            recorder.show_graphs()
            recorder.show_graphs()
        root.assert_called_once()
        root.return_value.withdraw.assert_called_once()
        graphs.assert_called_once_with(root.return_value, show_activation=False)
        self.assertEqual(graphs.return_value.show.call_count, 2)

    def test_guided_start_opens_sources_at_go_and_does_not_label_marker_one(self):
        reader = prepared_reader()
        live = SimpleNamespace(
            reader=reader, cv2=Mock(), camera_observation={}, frame_time_ns=time.monotonic_ns()
        )
        with tempfile.TemporaryDirectory() as temporary:
            recorder = CupRecorder(
                recorder_args(output=Path(temporary), subject="S01", side="left"), live
            )
            recorder.guide = CupTrialGuide(baseline_s=0, countdown_s=0, rest_s=0)
            with (
                patch("aurora.recording.record_cup_trials.emit"),
            ):
                recorder.command({"action": "record", "condition": "slow"})
                self.assertFalse(recorder.trials.active)
                self.assertFalse(list(Path(temporary).iterdir()))
                recorder.advance_protocol()
                metadata = recorder.trials.metadata
                self.assertGreaterEqual(
                    metadata["movement_go_host_monotonic_ns"], recorder.trials.started_ns
                )
                self.assertEqual(metadata["operator_events"], [])
                self.assertFalse(metadata["task_completion_verified"])
                self.assertEqual(metadata["movement_labels"], "not_reviewed")
                self.assertEqual(metadata["protocol_version"], 3)
                self.assertEqual(metadata["pace_condition"], "slow")
                self.assertIn("GO", [event["cue"] for event in metadata["cue_events"]])
                recorder.stop_trial(complete=False)

    def test_cancel_and_retry_keep_the_first_attempt(self):
        reader = prepared_reader()
        live = SimpleNamespace(
            reader=reader, cv2=Mock(), camera_observation={}, frame_time_ns=time.monotonic_ns()
        )
        with tempfile.TemporaryDirectory() as temporary:
            recorder = CupRecorder(
                recorder_args(output=Path(temporary), subject="S01", side="left"), live
            )
            recorder.guide = CupTrialGuide(countdown_s=0, rest_s=0)
            with (
                patch("aurora.recording.record_cup_trials.emit"),
            ):
                recorder.command("record")
                recorder.advance_protocol()
                first = recorder.trials.path
                feed_samples(reader, 2)
                recorder.command("retry")
                original = (first / "trial.json").read_bytes()
                saved = json.loads(original)
                self.assertFalse(saved["complete"])
                self.assertTrue(saved["retry_requested"])
                self.assertIn("movement_go_host_monotonic_ns", saved)
                recorder.command("record")
                recorder.advance_protocol()
                self.assertNotEqual(recorder.trials.path, first)
                self.assertEqual(recorder.trials.metadata["protocol"]["repeat_of"], "T001")
                self.assertEqual((first / "trial.json").read_bytes(), original)
                recorder.stop_trial(complete=False)

    def test_practice_uses_the_cues_without_creating_recordings(self):
        live = SimpleNamespace(
            reader=prepared_reader(),
            cv2=Mock(),
            camera_observation={},
            frame_time_ns=time.monotonic_ns(),
        )
        with tempfile.TemporaryDirectory() as temporary:
            recorder = CupRecorder(
                recorder_args(output=Path(temporary), subject="S01", side="left"), live
            )
            with (
                patch("aurora.recording.record_cup_trials.emit"),
            ):
                recorder.command({"action": "practice", "condition": "fast"})
                self.assertTrue(recorder.guide.practice)
                self.assertTrue(recorder.guide.active)
                self.assertFalse(recorder.trials.active)
                self.assertIsNone(live.reader._recording)
                recorder.command("stop")
                self.assertFalse(recorder.guide.practice)
            self.assertFalse(list(Path(temporary).iterdir()))


class CupProtocolTests(unittest.TestCase):
    def test_reference_waits_for_the_operator_before_every_countdown(self):
        now = [1_000_000_000]
        guide = CupReferenceGuide(clock=lambda: now[0])
        for index, (cue, seconds, _instruction) in enumerate(REFERENCE_STEPS):
            self.assertTrue(guide.active)
            self.assertEqual(guide.step, index)
            self.assertIn(cue, guide.view()["cue"])
            self.assertEqual(guide.phase, "reference_prepare")
            now[0] += 120_000_000_000
            guide.tick()
            self.assertEqual(guide.phase, "reference_prepare")
            self.assertEqual(guide.step, index)
            guide.start_step()
            self.assertEqual(guide.phase, "reference_countdown")
            now[0] += 3_000_000_000
            guide.tick()
            self.assertEqual(guide.phase, "reference")
            self.assertIn("GO", guide.view()["cue"])
            now[0] += (seconds - 1) * 1_000_000_000
            guide.tick()
            self.assertEqual(guide.step, index)
            now[0] += 1_000_000_000
            guide.tick()
        self.assertFalse(guide.active)
        self.assertTrue(guide.completed)
        self.assertIsNone(guide.go_ns)
        guide.tick()
        self.assertFalse(guide.active)

    def test_countdown_go_and_rest_have_separate_timing(self):
        now = [10_000_000_000]
        guide = CupTrialGuide(clock=lambda: now[0], baseline_s=2)
        guide.begin("natural")
        self.assertEqual(guide.view()["cue"], "KEEP STILL")
        now[0] += 2_000_000_000
        guide.tick()
        self.assertEqual(guide.view()["cue"], "3")
        for cue in ("2", "1"):
            now[0] += 1_000_000_000
            guide.tick()
            self.assertEqual(guide.view()["cue"], cue)
        now[0] += 1_000_000_000
        guide.tick()
        self.assertEqual(guide.view()["cue"], "GO")
        self.assertEqual(guide.go_ns, now[0])
        guide.finish()
        self.assertEqual(guide.view()["cue"], "REST 20 s")
        with self.assertRaises(ValueError):
            guide.begin("slow")
        now[0] += 20_000_000_000
        guide.tick()
        self.assertEqual(guide.phase, "ready")
        guide.begin("slow")
        self.assertIsNone(guide.go_ns)

    def test_delayed_processing_cannot_skip_the_countdown(self):
        now = [1_000_000_000]
        guide = CupTrialGuide(clock=lambda: now[0], baseline_s=2)
        guide.begin("natural")
        now[0] += 20_000_000_000
        guide.tick()
        self.assertEqual(guide.view()["cue"], "3")
        self.assertIsNone(guide.go_ns)

    def test_cancel_before_go_does_not_require_movement_rest(self):
        guide = CupTrialGuide()
        guide.begin("fast")
        guide.finish()
        self.assertEqual(guide.phase, "ready")
        guide.begin("slow", practice=True)
        self.assertTrue(guide.practice)

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
        ProcessManagerMixin._initialize_process_manager(manager)
        manager._queue_process_event = lambda event: ProcessManagerMixin._queue_process_event(
            manager, event
        )
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
            self.assertEqual(manager.process_exits.get()[0], "exit")


if __name__ == "__main__":
    unittest.main()
