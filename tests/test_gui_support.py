# Majd, Christian
"""Test GUI planning and repository discovery without creating Tk windows."""

from __future__ import annotations

import io
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "gui"))
sys.path.insert(0, str(ROOT / "src" / "recording"))
sys.path.insert(0, str(ROOT / "src" / "simulation" / "ai"))

import app as gui_app
from common import create_session_directory, experiment_metadata
from ble_dashboard import BleDashboardMixin
from imu_protocol import (
    i2c_wiring_hint,
)
from live_sensor_input import installed_pose_model_complexity
from process_manager import ManagedProcess, ProcessManagerMixin
from project_support import (
    FIRMWARE_WORKSPACE_SCRIPT,
    FirmwareProject,
    discover_firmware_projects,
    discover_recording_programs,
    firmware_command,
    resolve_firmware_tool,
)
from recording_tab import DEFAULT_BATCH_SOURCES, DEFAULT_BLE_SENSORS, batch_arguments


class RecordingBatchTests(unittest.TestCase):
    def test_grip_options_only_reach_the_ble_recorder(self):
        self.assertEqual(
            batch_arguments("record_ble_sensors.py", [], "", ("emg",), True, 3),
            ["--sensors", "emg", "--grip-trial", "--grip-repeats", "3"],
        )
        self.assertEqual(batch_arguments("record_oak_pose.py", ["--depth"], "", (), True, 3),
                         ["--auto-start", "--depth"])
        with self.assertRaises(ValueError):
            batch_arguments("record_ble_sensors.py", [], "1", ("emg",), True, 3)

    def test_recorder_releases_ble_preview_before_opening_the_connection(self):
        order = []
        manager = types.SimpleNamespace(
            processes={},
            _active_processes=lambda: [],
            _stop_ble_dashboard=Mock(side_effect=lambda: order.append("disconnect") or True),
            _append_log=Mock(),
            _set_status=Mock(),
            _display_command=Mock(return_value="python recorder.py"),
            _update_controls=Mock(),
            _refresh_info=Mock(),
            _watch_process=Mock(),
        )
        with (
            patch("process_manager.threading.Thread"),
            patch(
                "process_manager.subprocess.Popen",
                side_effect=lambda *_args, **_kwargs: order.append("launch") or Mock(),
            ),
        ):
            self.assertTrue(
                ProcessManagerMixin._start_program(
                    manager, "BLE recording", "recording", ["python", "recorder.py"]
                )
            )
        self.assertEqual(order, ["disconnect", "launch"])

    def test_recorder_waits_when_ble_disconnect_has_not_finished(self):
        manager = types.SimpleNamespace(
            processes={}, _active_processes=lambda: [],
            _stop_ble_dashboard=Mock(return_value=False), _append_log=Mock(),
        )
        with patch("process_manager.subprocess.Popen") as launch:
            self.assertFalse(
                ProcessManagerMixin._start_program(
                    manager, "BLE recording", "recording", ["python", "recorder.py"]
                )
            )
        launch.assert_not_called()

    def test_ble_monitor_keeps_the_reader_until_its_thread_has_stopped(self):
        reader = Mock()
        reader.thread.is_alive.return_value = True
        dashboard = types.SimpleNamespace(
            ble_reader=reader, ble_graphs=Mock(), ble_sensor_connection=Mock(),
        )
        self.assertFalse(BleDashboardMixin._stop_ble_dashboard(dashboard))
        self.assertIs(dashboard.ble_reader, reader)
        dashboard.ble_graphs.window.withdraw.assert_not_called()
        reader.thread.is_alive.return_value = False
        self.assertTrue(BleDashboardMixin._stop_ble_dashboard(dashboard))
        self.assertIsNone(dashboard.ble_reader)
        dashboard.ble_graphs.window.withdraw.assert_called_once()

    def test_ble_handoff_runs_the_action_only_after_disconnect(self):
        action = Mock()
        dashboard = types.SimpleNamespace(
            _ble_handoff_pending=False,
            _stop_ble_dashboard=Mock(side_effect=[False, False, True]),
            _update_controls=Mock(), after=Mock(), ble_sensor_connection=Mock(),
        )
        self.assertTrue(BleDashboardMixin._release_ble_for(dashboard, action))
        action.assert_not_called()
        finish = dashboard.after.call_args.args[1]
        finish()
        action.assert_not_called()
        finish()
        action.assert_called_once()
        self.assertFalse(dashboard._ble_handoff_pending)

    def test_live_pose_model_never_requires_a_startup_download(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            package = Path(temporary) / "mediapipe"
            package.mkdir()
            fake_mp = types.SimpleNamespace(__file__=str(package / "__init__.py"))
            self.assertEqual(installed_pose_model_complexity(fake_mp), 1)
            heavy = package / "modules" / "pose_landmark" / "pose_landmark_heavy.tflite"
            heavy.parent.mkdir(parents=True)
            heavy.touch()
            self.assertEqual(installed_pose_model_complexity(fake_mp), 2)

    def test_gui_relaunch_keeps_spaced_script_path_as_one_argument(self) -> None:
        simulation_python = ROOT / "environment with spaces" / "python.exe"
        with (
            patch("app.importlib.util.find_spec", return_value=None),
            patch("app.find_simulation_python", return_value=simulation_python),
            patch.object(gui_app.sys, "executable", r"C:\Python313\python.exe"),
            patch.object(
                gui_app.sys,
                "argv",
                [str((ROOT / "src" / "gui" / "app.py").resolve())],
            ),
            patch("app.subprocess.call", return_value=0) as relaunch,
            patch.dict(os.environ, {"AURORA_GUI_RELAUNCHED": ""}),
            self.assertRaises(SystemExit) as exit_context,
        ):
            gui_app.main()

        self.assertEqual(exit_context.exception.code, 0)
        command = relaunch.call_args.args[0]
        self.assertEqual(command[0], str(simulation_python))
        self.assertEqual(command[1], str((ROOT / "src" / "gui" / "app.py").resolve()))
        self.assertEqual(len(command), 2)

    def test_default_capture_uses_camera_and_ble_sensors(self) -> None:
        self.assertEqual(
            DEFAULT_BATCH_SOURCES,
            {"record_oak_pose.py", "record_ble_sensors.py"},
        )
        self.assertEqual(DEFAULT_BLE_SENSORS, ("imu", "emg"))

    def test_firmware_project_uses_native_esp_idf_commands(self) -> None:
        discovered = discover_firmware_projects()[str(Path("firmware") / "high_rate_emg_imu")]
        project = FirmwareProject(
            Path(ROOT.anchor) / "firmware" / "high_rate_emg_imu",
            discovered.system,
            discovered.executable,
        )
        self.assertEqual(project.system, "ESP-IDF")
        self.assertEqual(project.executable, "idf.py")
        with patch(
            "project_support.resolve_firmware_tool",
            return_value=("idf.py",),
        ):
            self.assertEqual(
                firmware_command(project, "build"),
                ["idf.py", "build"],
            )
            self.assertEqual(
                firmware_command(project, "flash", "COM4"),
                ["idf.py", "-p", "COM4", "flash"],
            )
            self.assertEqual(
                firmware_command(project, "monitor", "COM4"),
                ["idf.py", "-p", "COM4", "monitor"],
            )

    def test_firmware_can_fall_back_to_platformio_with_esp_idf_only(self) -> None:
        discovered = discover_firmware_projects()[str(Path("firmware") / "high_rate_emg_imu")]
        project = FirmwareProject(
            Path(ROOT.anchor) / "firmware" / "high_rate_emg_imu",
            discovered.system,
            discovered.executable,
        )
        platformio = str(Path(ROOT.anchor) / "tools" / "platformio")
        with patch(
            "project_support.resolve_firmware_tool",
            return_value=(platformio,),
        ):
            self.assertEqual(
                firmware_command(project, "build"),
                [platformio, "run"],
            )
            self.assertEqual(
                firmware_command(project, "flash", "COM5"),
                [
                    platformio,
                    "run",
                    "-t",
                    "upload",
                    "--upload-port",
                    "COM5",
                ],
            )
            self.assertEqual(
                firmware_command(project, "monitor", "COM5"),
                [platformio, "device", "monitor", "--port", "COM5"],
            )

    def test_firmware_finds_platformio_installed_in_the_python_environment(self) -> None:
        project = discover_firmware_projects()[str(Path("firmware") / "high_rate_emg_imu")]
        python = Path(r"C:\env with spaces\python.exe")
        with (
            patch("project_support.shutil.which", return_value=None),
            patch("project_support.importlib.util.find_spec", return_value=object()),
            patch.object(sys, "executable", str(python)),
        ):
            self.assertEqual(
                resolve_firmware_tool(project),
                (str(python), "-m", "platformio"),
            )

    def test_firmware_build_is_staged_when_repository_path_contains_spaces(self) -> None:
        project = FirmwareProject(ROOT / "firmware with spaces", "ESP-IDF", "idf.py")
        platformio = (sys.executable, "-m", "platformio")
        with patch("project_support.resolve_firmware_tool", return_value=platformio):
            self.assertEqual(
                firmware_command(project, "flash", "COM5"),
                [
                    sys.executable,
                    str(FIRMWARE_WORKSPACE_SCRIPT),
                    "--project",
                    str(project.directory),
                    "--",
                    *platformio,
                    "run",
                    "-t",
                    "upload",
                    "--upload-port",
                    "COM5",
                ],
            )

    def test_firmware_wiring_and_role_addresses_match_the_current_arm(self) -> None:
        project = ROOT / "firmware" / "high_rate_emg_imu"
        source = (project / "main" / "main.c").read_text()
        config = (project / "include" / "config.h").read_text()
        self.assertIn('{IMU_SDA_PIN, IMU_SCL_PIN, "current harness"}', source)
        self.assertIn('{4, 5, "LIMB-HT25"}', source)
        self.assertIn('{8, 5, "AURORA prototype"}', source)
        for name, value in {
            "IMU_SDA_PIN": "GPIO_NUM_2",
            "IMU_SCL_PIN": "GPIO_NUM_1",
            "IMU_PRIMARY_ADDR": "0x6B",
            "IMU_SECONDARY_ADDR": "0x6A",
            "EMG_ADC_CHANNEL": "ADC_CHANNEL_0",
            "EMG_SAMPLE_RATE_HZ": "4000",
            "TRANSPORT_MODE": "TRANSPORT_BLE",
        }.items():
            with self.subTest(setting=name):
                self.assertRegex(config, rf"(?m)^#define\s+{name}\s+{value}\b")
        emg = (project / "main" / "emg.c").read_text()
        self.assertIn("adc_continuous", emg)

    def test_dual_imu_check_identifies_an_open_scl_conductor(self) -> None:
        self.assertEqual(
            i2c_wiring_hint(
                {
                    "sda_pin": 2,
                    "scl_pin": 1,
                    "external_sda_pullup": True,
                    "external_scl_pullup": False,
                }
            ),
            "SDA reaches GPIO2, but SCL does not reach GPIO1. "
            "Check the SCL conductor and sensor power/ground.",
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
            batch_arguments("record_ble_sensors.py", camera_options, "1", ("imu", "emg")),
            ["--sensors", "imu", "emg", "--dataset-label", "1"],
        )
        self.assertEqual(batch_arguments("record_serial_sensors.py", camera_options), [])
        self.assertEqual(batch_arguments("record_high_rate_serial.py", camera_options), [])

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
        self.assertTrue(
            {
                "record_ble_sensors.py",
                "record_oak_pose.py",
                "record_serial_sensors.py",
                "capture_movement.py",
            }.issubset(names)
        )


if __name__ == "__main__":
    unittest.main()
