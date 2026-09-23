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

from common import create_session_directory, experiment_metadata
from project_support import discover_recording_programs
from process_manager import ManagedProcess, ProcessManagerMixin
from recording_tab import SENSOR_PREVIEWS, batch_arguments


class RecordingBatchTests(unittest.TestCase):
    def test_each_ble_sensor_has_its_own_preview_entry(self) -> None:
        previews = {preview_id: filename for preview_id, filename, *_rest in SENSOR_PREVIEWS}
        self.assertEqual(
            {name: previews[name] for name in ("emg", "imu", "piezo")},
            {
                "emg": "record_ble_sensors.py",
                "imu": "record_ble_sensors.py",
                "piezo": "record_ble_sensors.py",
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
