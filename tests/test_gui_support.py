"""Test GUI planning and repository discovery without creating Tk windows."""

from __future__ import annotations

import os
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "gui"))
sys.path.insert(0, str(ROOT / "src" / "recording"))
sys.path.insert(0, str(ROOT / "src" / "simulation" / "ai"))

from common import create_session_directory, experiment_metadata
from imu_protocol import extract_imus
from project_support import discover_recording_programs
from process_manager import ManagedProcess, ProcessManagerMixin
from recording_tab import DEFAULT_BATCH_SOURCES, batch_arguments
from sensor_fusion import (
    DualImuArmEstimator,
    fuse_control_angles,
    interactive_control_targets,
)
from serial_sensor import decode_sensor_packet


class RecordingBatchTests(unittest.TestCase):
    def test_esp32_dual_imu_packet_has_physical_roles(self) -> None:
        packet = decode_sensor_packet(
            '{"device":"ESP32-C3","uptime_ms":1200,"imus":{'
            '"shoulder":{"connected":true,"address":"0x6A",'
            '"accel_g":{"x":0.1,"y":-0.2,"z":1.0},'
            '"gyro_dps":{"x":1,"y":2,"z":3}},'
            '"wrist":{"connected":true,"address":"0x6B",'
            '"accel_g":{"x":0,"y":0,"z":1},'
            '"gyro_dps":{"x":0,"y":0,"z":0}}}}'
        )
        self.assertEqual(packet["device"], "ESP32-C3")
        sensors = extract_imus(packet)
        self.assertEqual(sensors["shoulder"]["address"], "0x6A")
        self.assertEqual(sensors["wrist"]["accel_g"]["z"], 1.0)
        self.assertIsNone(decode_sensor_packet("ESP-ROM: boot message"))

    def test_default_capture_uses_camera_and_dual_imu_serial(self) -> None:
        self.assertEqual(
            DEFAULT_BATCH_SOURCES,
            {"record_oak_pose.py", "record_serial_sensors.py"},
        )

    def test_limb25_dual_packet_is_normalized_from_si_units(self) -> None:
        packet = {
            "imu1": {
                "accel": {"x": 0.0, "y": 0.0, "z": 9.80665},
                "gyro": {"x": 0.0, "y": 0.0, "z": 3.141592653589793},
            },
            "imu2": {
                "accel": {"x": 0.0, "y": 0.0, "z": 9.80665},
                "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
        }
        sensors = extract_imus(packet)
        self.assertAlmostEqual(sensors["shoulder"]["accel_g"]["z"], 1.0)
        self.assertAlmostEqual(sensors["shoulder"]["gyro_dps"]["z"], 180.0)

    def test_dual_imu_estimator_calibrates_and_camera_corrects(self) -> None:
        estimator = DualImuArmEstimator(alpha=0.0)
        level = {
            role: {
                "connected": True,
                "accel_g": {"x": 0.0, "y": 0.0, "z": 1.0},
                "gyro_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
            for role in ("shoulder", "wrist")
        }
        self.assertEqual(estimator.update(level, 0.02)["elbow_flexion"], 0.0)
        moved = {role: dict(sensor) for role, sensor in level.items()}
        moved["wrist"] = {
            **level["wrist"],
            "accel_g": {"x": 1.0, "y": 0.0, "z": 0.0},
        }
        imu = estimator.update(moved, 0.02)
        self.assertAlmostEqual(imu["elbow_flexion"], 90.0)
        fused = fuse_control_angles(imu, {"elbow_flexion": 70.0}, 0.25)
        self.assertAlmostEqual(fused["elbow_flexion"], 85.0)

    def test_fused_angles_map_to_interactive_right_arm_signs(self) -> None:
        targets = interactive_control_targets(
            {
                "elbow_flexion": 35.0,
                "shoulder_flexion": 40.0,
                "shoulder_abduction": 20.0,
                "shoulder_rotation_proxy": -15.0,
            }
        )
        self.assertEqual(
            targets,
            {
                "elbow_x": -35.0,
                "shoulder_y": 40.0,
                "shoulder_z": -20.0,
                "shoulder_x": -15.0,
            },
        )

    def test_running_preview_can_receive_another_window_request(self) -> None:
        child_input = io.StringIO()
        child = types.SimpleNamespace(poll=lambda: None, stdin=child_input)
        managed = ManagedProcess("preview:ble", "BLE preview", "preview", child)
        manager = types.SimpleNamespace(processes={"preview:ble": managed})
        sent = ProcessManagerMixin._send_process_input(manager, "preview:ble", "show imu\n")
        self.assertTrue(sent)
        self.assertEqual(child_input.getvalue(), "show imu\n")

    def test_source_specific_arguments_do_not_leak_between_recorders(self) -> None:
        camera_options = ["--depth", "--pose-model", "2"]
        self.assertEqual(
            batch_arguments("record_oak_pose.py", camera_options, "1"),
            ["--auto-start", *camera_options],
        )
        self.assertEqual(
            batch_arguments("record_ble_sensors.py", camera_options, "1"),
            ["--dataset-label", "1"],
        )
        self.assertEqual(batch_arguments("record_serial_sensors.py", camera_options), [])

    def test_batch_sources_share_a_session_prefix_and_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            environment = {
                "AURORA_OUTPUT_DIR": temporary,
                "AURORA_SESSION_ID": "20260923_120000_batch",
                "AURORA_BATCH_SOURCES": "record_ble_sensors.py,record_oak_pose.py",
                "AURORA_TRIAL_ID": "T07",
                "AURORA_TEST_TYPE": "combined",
            }
            with patch.dict(os.environ, environment, clear=False):
                ble = create_session_directory("ble", "S01")
                camera = create_session_directory("oak_pose", "S01")
                metadata = experiment_metadata()
            self.assertEqual(ble.name, "20260923_120000_batch_S01_ble")
            self.assertEqual(camera.name, "20260923_120000_batch_S01_oak_pose")
            self.assertEqual(metadata["session_id"], "20260923_120000_batch")
            self.assertEqual(metadata["trial_id"], "T07")

    def test_discovery_finds_maintained_and_training_capture_tools(self) -> None:
        programs = discover_recording_programs()
        names = {path.name for path in programs.values()}
        self.assertTrue({
            "record_ble_sensors.py",
            "record_oak_pose.py",
            "record_serial_sensors.py",
            "capture_movement.py",
        }.issubset(names))


if __name__ == "__main__":
    unittest.main()
