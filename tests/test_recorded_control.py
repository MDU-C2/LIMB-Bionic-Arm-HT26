# Majd
"""Replay synthetic recorded packets through the live controller."""

from __future__ import annotations

import base64
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from aurora.sensors.ble_protocol import HIGH_RATE_EMG_HEADER, HIGH_RATE_IMU_PACKET
from aurora.sensors.replay_input import RecordedSensorInput


def make_trial(root):
    (root / "ble").mkdir()
    (root / "camera").mkdir()
    origin = 10_000_000_000
    profile = {
        "emg_baseline_adc": 2048,
        "emg_rest": 1,
        "emg_reference": 10,
        "emg_strong": 12,
        "emg_period": 0.00025,
        "imu_tilt_offsets": {"shoulder": 0, "wrist": 0},
        "imu_gyro_bias_z": {"shoulder": 0, "wrist": 0},
        "imu_twist_bias_dps": {name: [0, 0, 0] for name in ("shoulder", "wrist")},
    }
    (root / "trial.json").write_text(
        json.dumps(
            {
                "side": "left",
                "duration_seconds": 1.0,
                "started_host_monotonic_ns": origin,
                "calibration": profile,
            }
        )
    )
    events = []
    for index in range(101):
        packets = {
            "imu": HIGH_RATE_IMU_PACKET.pack(
                0x103,
                index,
                1_000_000 + index * 10_000,
                1_000_000 + index * 10_000,
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
            "emg": HIGH_RATE_EMG_HEADER.pack(0x101, index, 1_000_000 + index * 10_000, 250, 40, 0)
            + struct.pack("<40H", *([2048] * 40)),
        }
        for sensor, data in packets.items():
            events.append(
                json.dumps(
                    {
                        "host_monotonic_ns": origin + index * 10_000_000,
                        "sensor": sensor,
                        "data_base64": base64.b64encode(data).decode(),
                    }
                )
            )
    (root / "ble/packets.jsonl").write_text("\n".join(events))
    (root / "camera/pose.json").write_text(
        json.dumps(
            {
                "observations": [
                    {
                        "host_monotonic_ns": origin,
                        "angles_deg": {"elbow_flexion": 95, "shoulder_flexion": 30},
                    },
                    {
                        "host_monotonic_ns": origin + 500_000_000,
                        "angles_deg": {"elbow_flexion": 85, "shoulder_flexion": 40},
                    },
                ]
            }
        )
    )


class RecordedControlTests(unittest.TestCase):
    def test_replay_rejects_nonfinite_duration_and_seek_times(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            make_trial(root)
            path = root / "trial.json"
            manifest = json.loads(path.read_text())
            for invalid in (float("nan"), float("inf"), -1):
                with self.subTest(duration=invalid):
                    path.write_text(json.dumps({**manifest, "duration_seconds": invalid}))
                    with self.assertRaisesRegex(ValueError, "duration"):
                        RecordedSensorInput(root, show_monitor=False)
            path.write_text(json.dumps(manifest))
            replay = RecordedSensorInput(root, show_monitor=False)
            try:
                replay.start()
                for invalid in (float("nan"), float("inf"), -1):
                    with self.subTest(seek=invalid):
                        with self.assertRaises(ValueError):
                            replay.advance_to(invalid)
                        self.assertEqual(replay.elapsed, 0)
                self.assertTrue(replay.advance_to(0.5).imu_ready)
            finally:
                replay.close()

    def test_video_release_failure_still_releases_sensor_input_and_can_retry(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            make_trial(root)
            replay = RecordedSensorInput(root, show_monitor=False)
            replay.start()
            video = Mock()
            video.release.side_effect = [RuntimeError("video busy"), None]
            replay.capture = video
            with self.assertRaisesRegex(RuntimeError, "video busy"):
                replay.close()
            self.assertIsNone(replay.reader)
            self.assertIs(replay.capture, video)
            replay.close()
            self.assertIsNone(replay.capture)
            self.assertEqual(video.release.call_count, 2)

    def test_headless_replay_does_not_require_camera_or_ble_packages(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            make_trial(root)
            replay = RecordedSensorInput(root, show_monitor=False)
            with patch.dict(
                sys.modules, {name: None for name in ("cv2", "depthai", "mediapipe", "bleak")}
            ):
                try:
                    replay.start()
                    result = replay.advance_to(0.5)
                    self.assertTrue(result.imu_ready)
                    self.assertTrue(result.camera_ready)
                finally:
                    replay.close()

    def test_fitted_mounting_produces_equivalent_live_and_recorded_anatomical_angles(self):
        import numpy as np
        from test_arm_alignment import generated_reference

        from aurora.sensors.arm_alignment import fit_alignment
        from aurora.sensors.ble_input import BleSensorReader

        stages, camera, _, _ = generated_reference("right")
        alignment = fit_alignment(stages, camera, side="right")
        clock = [1000.0]
        live = BleSensorReader(clock=lambda: clock[0], side="right")
        alignment.update(
            subject_id="S01",
            connection_generation=live.connection_generation,
            mounting_session_id=live.mounting_session_id,
        )
        live.apply_alignment(alignment, subject_id="S01", side="right")
        origin = 10_000_000_000
        packets = []
        for index in range(101):
            values = []
            for role in ("shoulder", "wrist"):
                series = stages["BEND"][role]
                values.extend(np.round(series.accel_g[index] * 1000).astype(int))
                values.extend(np.round(series.gyro_dps[index] * 100).astype(int))
            stamp = 1_000_000 + index * 10_000
            packets.append(HIGH_RATE_IMU_PACKET.pack(0x103, index, stamp, stamp, *values))
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "ble").mkdir()
            (root / "camera").mkdir()
            (root / "trial.json").write_text(
                json.dumps(
                    {
                        "side": "right",
                        "duration_seconds": 1,
                        "started_host_monotonic_ns": origin,
                        "calibration": live.calibration_profile(),
                    }
                ),
                encoding="utf-8",
            )
            (root / "camera/pose.json").write_text('{"observations": []}', encoding="utf-8")
            (root / "ble/packets.jsonl").write_text(
                "\n".join(
                    json.dumps(
                        {
                            "sensor": "imu",
                            "connection_generation": 0,
                            "host_monotonic_ns": origin + index * 10_000_000,
                            "data_base64": base64.b64encode(packet).decode(),
                        }
                    )
                    for index, packet in enumerate(packets)
                ),
                encoding="utf-8",
            )
            replay = RecordedSensorInput(root, show_monitor=False)
            try:
                replay.start()
                for index, packet in enumerate(packets):
                    clock[0] = 1000 + index / 100
                    live.notification("imu", packet)
                    result = replay.advance_to(index / 100)
                    expected = live.latest()["imu_angles"]
                    self.assertEqual(set(expected), set(result.angles))
                    for name, value in expected.items():
                        self.assertAlmostEqual(value, result.angles[name], delta=1e-8)
                self.assertGreater(result.angles["elbow_flexion"], 90)
                self.assertLessEqual(result.robot_targets["elbow_x"], 60)
            finally:
                replay.close()

    def test_saved_ble_replay_ignores_legacy_serial_selection(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            make_trial(root)
            replay = RecordedSensorInput(root, show_monitor=False, transport="serial", port="COM5")
            try:
                replay.start()
                result = replay.advance_to(0.5)
                self.assertTrue(result.imu_ready)
                self.assertEqual(set(result.sensors), {"shoulder", "wrist"})
                self.assertEqual(replay.transport, "ble")
            finally:
                replay.close()

    def test_replay_uses_saved_packets_and_calibration_without_hardware(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            make_trial(root)
            replay = RecordedSensorInput(root, show_monitor=False, grip_source="emg")
            with patch(
                "aurora.sensors.ble_input.BleSensorReader.start",
                side_effect=AssertionError("hardware"),
            ):
                try:
                    replay.start()
                    replay.calibrate({"elbow_flexion": 45, "shoulder_flexion": 45})
                    result = replay.advance_to(0.5)
                    self.assertTrue(result.imu_ready)
                    self.assertTrue(result.camera_ready)
                    self.assertAlmostEqual(replay.camera_angles["elbow_flexion"], 85)
                    self.assertAlmostEqual(replay.camera_angles["shoulder_flexion"], 40)
                    self.assertLessEqual(result.robot_targets["elbow_x"], 60)
                    self.assertEqual(result.emg_activation, 0)
                    self.assertEqual(replay.reader.emg_estimator.reference, 10)
                    self.assertIsNone(replay.pipeline)
                    self.assertNotIn("pace_condition", replay.manifest)
                finally:
                    replay.close()

    def test_pause_keeps_the_playback_position(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            make_trial(root)
            replay = RecordedSensorInput(root / "trial.json", show_monitor=False)
            try:
                replay.start()
                initial = replay.elapsed
                with patch(
                    "aurora.sensors.replay_input.time.monotonic",
                    return_value=replay.wall_started + 10,
                ):
                    replay.poll()
                self.assertEqual(replay.elapsed, initial)
                replay.advance_to(1)
                self.assertTrue(replay.ended)
                self.assertEqual(replay.index, len(replay.events))
            finally:
                replay.close()


if __name__ == "__main__":
    unittest.main()
