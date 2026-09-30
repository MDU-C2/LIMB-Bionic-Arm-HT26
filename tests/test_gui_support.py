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
from imu_protocol import (
    extract_imus,
    i2c_wiring_hint,
    ImuStreamDecoder,
    imu_configuration,
)
from project_support import (
    FIRMWARE_WORKSPACE_SCRIPT,
    FirmwareProject,
    discover_firmware_projects,
    discover_recording_programs,
    firmware_command,
    resolve_firmware_tool,
)
from process_manager import ManagedProcess, ProcessManagerMixin
from emg_protocol import EmgActivationEstimator, extract_emg, select_grip_source
from recording_tab import DEFAULT_BATCH_SOURCES, DEFAULT_BLE_SENSORS, batch_arguments
from sensor_fusion import (
    ControlAngleSmoother,
    DualImuArmEstimator,
    camera_control_angles,
    camera_hand_curl,
    fuse_control_angles,
    interactive_control_targets,
)
from live_sensor_input import installed_pose_model_complexity
from serial_sensor import decode_sensor_packet
import app as gui_app


class RecordingBatchTests(unittest.TestCase):
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

    def test_esp32_dual_imu_packet_has_physical_roles(self) -> None:
        packet = decode_sensor_packet(
            '{"device":"ESP32-C3","uptime_ms":1200,"imus":{'
            '"shoulder":{"connected":true,"address":"0x6B",'
            '"accel_g":{"x":0.1,"y":-0.2,"z":1.0},'
            '"gyro_dps":{"x":1,"y":2,"z":3}},'
            '"wrist":{"connected":true,"address":"0x6A",'
            '"accel_g":{"x":0,"y":0,"z":1},'
            '"gyro_dps":{"x":0,"y":0,"z":0}}}}'
        )
        self.assertEqual(packet["device"], "ESP32-C3")
        sensors = extract_imus(packet)
        self.assertEqual(sensors["shoulder"]["address"], "0x6B")
        self.assertEqual(sensors["wrist"]["accel_g"]["z"], 1.0)
        self.assertIsNone(decode_sensor_packet("ESP-ROM: boot message"))

    def test_default_capture_uses_camera_and_synchronized_dual_imu_ble(self) -> None:
        self.assertEqual(
            DEFAULT_BATCH_SOURCES,
            {"record_oak_pose.py", "record_ble_sensors.py"},
        )
        self.assertEqual(DEFAULT_BLE_SENSORS, ("imu", "emg"))

    def test_serial_packet_exposes_emg_and_data_driven_activation(self) -> None:
        packet = {
            "emg": {
                "connected": True,
                "gpio": 0,
                "adc_channel": 0,
                "adc_raw": 2025,
            }
        }
        self.assertEqual(extract_emg(packet)["adc_raw"], 2025)
        estimator = EmgActivationEstimator(calibration_samples=4)
        for value in (2000, 2002, 1998, 2000):
            estimator.update(value)
        self.assertTrue(estimator.calibrated)
        activation = None
        for value in (2600,) * 20:
            activation = estimator.update(value)
        self.assertIsNotNone(activation)
        self.assertGreater(activation, 0.25)
        self.assertEqual(select_grip_source(activation, 0.1)[1], "EMG")
        self.assertEqual(select_grip_source(None, 0.1), (0.1, "camera"))

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
            "accel_g": {"x": 0.0, "y": 1.0, "z": 0.0},
        }
        imu = estimator.update(moved, 0.02)
        self.assertAlmostEqual(imu["elbow_flexion"], 60.0)
        self.assertNotIn("shoulder_rotation_proxy", imu)
        fused = fuse_control_angles(imu, {"elbow_flexion": 50.0}, 0.25)
        self.assertAlmostEqual(fused["elbow_flexion"], 57.5)

    def test_single_imu_controls_shoulder_and_camera_can_supply_elbow(self) -> None:
        estimator = DualImuArmEstimator(alpha=0.0)
        level = {
            "shoulder": {
                "connected": True,
                "accel_g": {"x": 0.0, "y": 0.0, "z": 1.0},
                "gyro_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        }
        initial = estimator.update(level, 0.02)
        self.assertEqual(initial["shoulder_flexion"], 0.0)
        self.assertNotIn("elbow_flexion", initial)

        moved = {
            "shoulder": {
                **level["shoulder"],
                "accel_g": {"x": 0.0, "y": 1.0, "z": 0.0},
            }
        }
        imu = estimator.update(moved, 0.02)
        self.assertAlmostEqual(imu["shoulder_flexion"], 90.0)
        fused = fuse_control_angles(imu, {"elbow_flexion": 42.0}, 0.25)
        self.assertEqual(fused["elbow_flexion"], 42.0)

    def test_wrist_only_imu_does_not_impersonate_the_upper_arm(self) -> None:
        estimator = DualImuArmEstimator(alpha=0.0)
        wrist = {
            "wrist": {
                "connected": True,
                "accel_g": {"x": 0.0, "y": 1.0, "z": 0.0},
                "gyro_dps": {"x": 0.0, "y": 0.0, "z": 20.0},
            }
        }
        self.assertEqual(estimator.update(wrist, 0.02), {})

    def test_imu_gyro_z_drives_shoulder_left_right_without_axial_drift(self) -> None:
        estimator = DualImuArmEstimator(
            alpha=0.0,
            gyro_deadzone_dps=1.0,
            gyro_abduction_gain=1.0,
        )
        sensor = {
            "shoulder": {
                "connected": True,
                "accel_g": {"x": 0.0, "y": 0.0, "z": 1.0},
                "gyro_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        }
        estimator.update(sensor, 0.1)
        sensor["shoulder"]["gyro_dps"]["z"] = 31.0
        estimate = None
        for _ in range(10):
            estimate = estimator.update(sensor, 0.1)
        self.assertAlmostEqual(estimate["shoulder_abduction"], 30.0)
        self.assertNotIn("shoulder_rotation_proxy", estimate)

    def test_imu_gyro_direction_is_not_reflected_to_the_opposite_side(self) -> None:
        estimator = DualImuArmEstimator(
            alpha=0.0,
            gyro_deadzone_dps=1.0,
            gyro_abduction_gain=1.0,
        )
        sensor = {
            "shoulder": {
                "connected": True,
                "accel_g": {"x": 0.0, "y": 0.0, "z": 1.0},
                "gyro_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        }
        estimator.update(sensor, 0.1)
        sensor["shoulder"]["gyro_dps"]["z"] = -31.0
        for _ in range(10):
            estimate = estimator.update(sensor, 0.1)
        self.assertEqual(estimate["shoulder_abduction"], 0.0)

    def test_default_imu_sensitivity_is_time_based_and_reduced(self) -> None:
        def one_second(period: float) -> float:
            estimator = DualImuArmEstimator(alpha=0.0)
            sensor = {
                "shoulder": {
                    "connected": True,
                    "accel_g": {"x": 0.0, "y": 0.0, "z": 1.0},
                    "gyro_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
                }
            }
            estimator.update(sensor, period)
            sensor["shoulder"]["gyro_dps"]["z"] = 42.5
            estimate = None
            for _ in range(round(1.0 / period)):
                estimate = estimator.update(sensor, period)
            return estimate["shoulder_abduction"]

        slow_stream = one_second(0.2)
        fast_stream = one_second(0.02)
        self.assertAlmostEqual(slow_stream, 18.0, places=3)
        self.assertAlmostEqual(fast_stream, slow_stream, places=3)

    def test_camera_owns_absolute_horizontal_axes_when_enabled(self) -> None:
        imu = {
            "shoulder_abduction": 35.0,
            "shoulder_rotation_proxy": -40.0,
        }
        camera = {
            "shoulder_abduction": 10.0,
            "shoulder_rotation_proxy": 15.0,
        }
        self.assertEqual(
            fuse_control_angles(imu, camera, 0.25),
            camera,
        )
        self.assertEqual(
            fuse_control_angles(imu, camera, 0.0),
            imu,
        )

    def test_adding_wrist_imu_does_not_create_a_false_elbow_bend(self) -> None:
        estimator = DualImuArmEstimator(alpha=0.0)
        shoulder = {
            "shoulder": {
                "connected": True,
                "accel_g": {"x": 0.0, "y": 0.0, "z": 1.0},
                "gyro_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        }
        estimator.update(shoulder, 0.02)
        shoulder["shoulder"]["accel_g"] = {
            "x": 0.0, "y": 0.5, "z": 0.8660254,
        }
        estimator.update(shoulder, 0.02)
        both = {
            **shoulder,
            "wrist": {
                "connected": True,
                "accel_g": {"x": 0.0, "y": 0.5, "z": 0.8660254},
                "gyro_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
        }
        self.assertAlmostEqual(estimator.update(both, 0.02)["elbow_flexion"], 0.0)

    def test_packet_reports_none_single_or_dual_imu_configuration(self) -> None:
        sensor = {
            "connected": True,
            "accel_g": {"x": 0.0, "y": 0.0, "z": 1.0},
            "gyro_dps": {"x": 0.0, "y": 0.0, "z": 0.0},
        }
        self.assertEqual(imu_configuration({}), ("none", ()))
        self.assertEqual(
            imu_configuration({"imus": {"shoulder": sensor}}),
            ("single", ("shoulder",)),
        )
        self.assertEqual(
            imu_configuration({"imus": {"shoulder": sensor, "wrist": sensor}}),
            ("dual", ("shoulder", "wrist")),
        )

    def test_last_verified_single_imu_packet_remains_visible(self) -> None:
        packet = decode_sensor_packet(
            '{"device":"ESP32-C3","imu":{"connected":true,'
            '"address":"0x6B","accel_g":{"x":0,"y":0,"z":1},'
            '"gyro_dps":{"x":1,"y":2,"z":3}}}'
        )
        self.assertEqual(imu_configuration(packet), ("single", ("shoulder",)))
        self.assertEqual(extract_imus(packet)["shoulder"]["gyro_dps"]["z"], 3.0)

    def test_single_imu_accepts_legacy_xyz_arrays_and_names(self) -> None:
        packet = {
            "imu": {
                "connected": True,
                "address": "0x6A",
                "acceleration": [0.0, 0.0, 9.80665],
                "gyroscope": [0.0, 0.0, 3.141592653589793],
            }
        }
        shoulder = extract_imus(packet)["shoulder"]
        self.assertTrue(shoulder["connected"])
        self.assertAlmostEqual(shoulder["accel_g"]["z"], 1.0)
        self.assertAlmostEqual(shoulder["gyro_dps"]["z"], 180.0)

    def test_currently_flushed_legacy_text_stream_shows_single_imu_values(self) -> None:
        decoder = ImuStreamDecoder()
        self.assertIsNone(decoder.feed("IMU 1 WHO_AM_I: 0x6C"))
        self.assertIsNone(decoder.feed("IMU 2 WHO_AM_I: 0xFF"))
        self.assertIsNone(decoder.feed("IMU1 ACC: 875  -156  4052"))
        packet = decoder.feed("IMU1 GYRO: 46  -95  -33")
        self.assertIsNotNone(packet)
        sensors = extract_imus(packet)
        self.assertTrue(sensors["shoulder"]["connected"])
        self.assertFalse(sensors["wrist"]["connected"])
        self.assertAlmostEqual(
            sensors["shoulder"]["accel_g"]["z"], 4052 / 4096
        )
        self.assertAlmostEqual(
            sensors["shoulder"]["gyro_dps"]["y"], -95 / 65.5
        )

    def test_camera_angles_are_normalized_before_they_reach_the_robot(self) -> None:
        normalized = camera_control_angles({
            "elbow_flexion": 130.0,
            "shoulder_flexion": -12.0,
            "shoulder_abduction": -28.0,
            "shoulder_rotation_proxy": 95.0,
        })
        self.assertEqual(normalized, {
            "elbow_flexion": 60.0,
            "shoulder_flexion": 0.0,
            "shoulder_abduction": 0.0,
            "shoulder_rotation_proxy": 60.0,
        })

    def test_camera_smoother_filters_jitter_without_filling_missing_axes(self) -> None:
        smoother = ControlAngleSmoother(time_constant_s=0.1)
        self.assertEqual(smoother.update({"elbow_flexion": 20.0}, 0.04), {
            "elbow_flexion": 20.0,
        })
        filtered = smoother.update({"elbow_flexion": 40.0}, 0.04)
        self.assertGreater(filtered["elbow_flexion"], 20.0)
        self.assertLess(filtered["elbow_flexion"], 40.0)
        self.assertEqual(
            smoother.update({"shoulder_flexion": 30.0}, 0.04),
            {"shoulder_flexion": 30.0},
        )

    def test_camera_hand_tracking_controls_a_bounded_shared_grip(self) -> None:
        self.assertAlmostEqual(camera_hand_curl({
            "thumb": 0.0, "index": 45.0, "middle": 90.0,
            "ring": 135.0, "pinky": -20.0,
        }), 0.5)
        self.assertIsNone(camera_hand_curl({"index": 30.0, "middle": 30.0}))

    def test_firmware_project_uses_native_esp_idf_commands(self) -> None:
        discovered = discover_firmware_projects()["firmware\\dual_imu_serial"]
        project = FirmwareProject(
            Path(r"C:\firmware\dual_imu_serial"),
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
        discovered = discover_firmware_projects()["firmware\\dual_imu_serial"]
        project = FirmwareProject(
            Path(r"C:\firmware\dual_imu_serial"),
            discovered.system,
            discovered.executable,
        )
        platformio = r"C:\Users\tester\.platformio\platformio.exe"
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
        project = discover_firmware_projects()["firmware\\dual_imu_serial"]
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
        project = discover_firmware_projects()["firmware\\dual_imu_serial"]
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

    def test_fused_angles_map_to_interactive_left_arm_signs(self) -> None:
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
                "elbow_x": 35.0,
                "shoulder_y": 50.0,
                "shoulder_z": -20.0,
                "shoulder_x": 15.0,
            },
        )

    def test_firmware_wiring_and_role_addresses_match_the_current_arm(self) -> None:
        source = (ROOT / "firmware" / "dual_imu_serial" / "main" / "main.c").read_text()
        self.assertIn("#define I2C_SDA_PIN GPIO_NUM_2", source)
        self.assertIn("#define I2C_SCL_PIN GPIO_NUM_1", source)
        self.assertIn('{I2C_SDA_PIN, I2C_SCL_PIN, "current-harness"}', source)
        self.assertIn('{GPIO_NUM_4, GPIO_NUM_5, "limb-ht25"}', source)
        self.assertIn('{GPIO_NUM_8, GPIO_NUM_5, "aurora-prototype"}', source)
        self.assertNotIn("legacy-6-7", source)
        self.assertNotIn("legacy-8-9", source)
        self.assertIn("#define I2C_FREQUENCY_HZ 100000", source)
        self.assertIn("#define SHOULDER_ADDRESS 0x6B", source)
        self.assertIn("#define WRIST_ADDRESS 0x6A", source)
        self.assertIn("#define EMG_ADC_CHANNEL ADC1_CHANNEL_0", source)
        self.assertIn("#define EMG_GPIO_NUM GPIO_NUM_0", source)
        self.assertIn("adc1_get_raw(EMG_ADC_CHANNEL)", source)

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

    def test_anatomical_shoulder_zero_is_converted_to_cad_zero(self) -> None:
        arm_down = interactive_control_targets({"shoulder_flexion": 0.0})
        arm_forward = interactive_control_targets({"shoulder_flexion": 90.0})
        self.assertEqual(arm_down, {"shoulder_y": 90.0})
        self.assertEqual(arm_forward, {"shoulder_y": 0.0})

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
            batch_arguments(
                "record_ble_sensors.py", camera_options, "1", ("imu", "emg")
            ),
            ["--sensors", "imu", "emg", "--dataset-label", "1"],
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
