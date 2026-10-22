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
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/simulation"))

from ai.recorded_sensor_input import RecordedSensorInput
from ble_packets import HIGH_RATE_EMG_HEADER, HIGH_RATE_IMU_PACKET


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
                "ble_sensor_input.BleSensorReader.start", side_effect=AssertionError("hardware")
            ):
                try:
                    replay.start()
                    replay.calibrate({"elbow_flexion": 45, "shoulder_flexion": 45})
                    result = replay.advance_to(0.5)
                    self.assertTrue(result.imu_ready)
                    self.assertTrue(result.camera_ready)
                    self.assertAlmostEqual(replay.camera_angles["elbow_flexion"], 35)
                    self.assertAlmostEqual(replay.camera_angles["shoulder_flexion"], 55)
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
                    "ai.recorded_sensor_input.time.monotonic", return_value=replay.wall_started + 10
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
