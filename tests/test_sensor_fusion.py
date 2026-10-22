# Majd, Christian
"""Check sensor normalization, calibration, fusion and grip control."""

from __future__ import annotations

import unittest

from aurora.gui.serial_sensor import decode_sensor_packet
from aurora.sensors.emg_protocol import EmgActivationEstimator, extract_emg, select_grip_source
from aurora.sensors.fusion import (
    ControlAngleSmoother,
    DualImuArmEstimator,
    camera_control_angles,
    camera_hand_curl,
    fuse_control_angles,
    interactive_control_targets,
)
from aurora.sensors.imu_protocol import ImuStreamDecoder, extract_imus, imu_configuration


class SensorFusionTests(unittest.TestCase):
    """Sensor normalization, calibration, fusion, and grip behavior."""

    def test_camera_grip_ignores_emg_packets_without_electrode_contact(self):
        self.assertEqual(select_grip_source(0.0, 0.9, "camera"), (0.9, "camera"))
        self.assertEqual(select_grip_source(1.0, 0.0, "camera"), (0.0, "camera"))
        self.assertEqual(select_grip_source(0.0, 0.9, "emg"), (0.0, "EMG"))
        self.assertEqual(select_grip_source(None, 0.9, "emg"), (None, "--"))

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
        self.assertAlmostEqual(imu["elbow_flexion"], 90.0)
        self.assertNotIn("shoulder_rotation_proxy", imu)
        fused = fuse_control_angles(imu, {"elbow_flexion": 50.0}, 0.25)
        self.assertAlmostEqual(fused["elbow_flexion"], 80.0)

    def test_tabletop_reference_preserves_pose_and_distinguishes_raise_from_lower(self):
        import math

        def sensors(tilt):
            return {
                role: {
                    "connected": True,
                    "accel_g": {
                        "x": 0,
                        "y": math.sin(math.radians(tilt)),
                        "z": math.cos(math.radians(tilt)),
                    },
                    "gyro_dps": dict.fromkeys("xyz", 0),
                }
                for role in ("shoulder", "wrist")
            }

        estimator = DualImuArmEstimator(alpha=0)
        reference = {
            "shoulder_flexion": 45,
            "shoulder_abduction": 5,
            "elbow_flexion": 45,
            "wrist_rotation": 20,
            "shoulder_rotation_proxy": 0,
        }
        estimator.reset_calibration(reference)
        for _ in range(51):
            initial = estimator.update(sensors(40), 0.01)
        self.assertEqual(initial, reference)
        raised = estimator.update(sensors(55), 0.01)
        lowered = estimator.update(sensors(25), 0.01)
        self.assertAlmostEqual(raised["shoulder_flexion"], 60)
        self.assertAlmostEqual(lowered["shoulder_flexion"], 30)
        self.assertEqual(raised["elbow_flexion"], 45)
        self.assertEqual(raised["wrist_rotation"], 20)
        fused = fuse_control_angles(initial, reference, 0.25)
        self.assertEqual(interactive_control_targets(fused)["shoulder_y"], 45)

    def test_tabletop_reference_allows_elbow_extension(self):
        estimator = DualImuArmEstimator(alpha=0)
        reference = {"shoulder_flexion": 45, "elbow_flexion": 45}
        estimator.reset_calibration(reference)
        sensors = {
            role: {
                "connected": True,
                "accel_g": {"x": 0, "y": 1, "z": 0},
                "gyro_dps": dict.fromkeys("xyz", 0),
            }
            for role in ("shoulder", "wrist")
        }
        estimator.update(sensors, 0.01)
        sensors["wrist"]["accel_g"] = {"x": 0, "y": 0.8660254, "z": 0.5}
        self.assertAlmostEqual(estimator.update(sensors, 0.01)["elbow_flexion"], 15, places=5)

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
        self.assertEqual(estimate["shoulder_abduction"], -30.0)
        self.assertEqual(interactive_control_targets(estimate)["shoulder_z"], -5.0)

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

    def test_camera_abduction_and_corrected_gyro_twist_are_preserved(self) -> None:
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
            {"shoulder_abduction": 10.0, "shoulder_rotation_proxy": -40.0},
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
            "x": 0.0,
            "y": 0.5,
            "z": 0.8660254,
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
        self.assertAlmostEqual(sensors["shoulder"]["accel_g"]["z"], 4052 / 4096)
        self.assertAlmostEqual(sensors["shoulder"]["gyro_dps"]["y"], -95 / 65.5)

    def test_measured_angles_are_preserved_until_robot_mapping(self) -> None:
        normalized = camera_control_angles(
            {
                "elbow_flexion": 130.0,
                "shoulder_flexion": -12.0,
                "shoulder_abduction": -28.0,
                "shoulder_rotation_proxy": 95.0,
            }
        )
        self.assertEqual(
            normalized,
            {
                "elbow_flexion": 130.0,
                "shoulder_flexion": -12.0,
                "shoulder_abduction": -28.0,
                "shoulder_rotation_proxy": 95.0,
            },
        )
        self.assertEqual(
            interactive_control_targets(normalized),
            {
                "elbow_x": 60.0,
                "shoulder_y": 75.0,
                "shoulder_z": -5.0,
                "shoulder_x": -60.0,
            },
        )

    def test_camera_smoother_filters_jitter_without_filling_missing_axes(self) -> None:
        smoother = ControlAngleSmoother(time_constant_s=0.1)
        self.assertEqual(
            smoother.update({"elbow_flexion": 20.0}, 0.04),
            {
                "elbow_flexion": 20.0,
            },
        )
        filtered = smoother.update({"elbow_flexion": 40.0}, 0.04)
        self.assertGreater(filtered["elbow_flexion"], 20.0)
        self.assertLess(filtered["elbow_flexion"], 40.0)
        self.assertEqual(
            smoother.update({"shoulder_flexion": 30.0}, 0.04),
            {"shoulder_flexion": 30.0},
        )

    def test_camera_hand_tracking_controls_a_bounded_shared_grip(self) -> None:
        self.assertAlmostEqual(
            camera_hand_curl(
                {
                    "thumb": 0.0,
                    "index": 45.0,
                    "middle": 90.0,
                    "ring": 135.0,
                    "pinky": -20.0,
                }
            ),
            0.5,
        )
        self.assertIsNone(camera_hand_curl({"index": 30.0, "middle": 30.0}))

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

    def test_anatomical_shoulder_zero_is_converted_to_cad_zero(self) -> None:
        arm_down = interactive_control_targets({"shoulder_flexion": 0.0})
        arm_forward = interactive_control_targets({"shoulder_flexion": 90.0})
        self.assertEqual(arm_down, {"shoulder_y": 75.0})
        self.assertEqual(arm_forward, {"shoulder_y": 0.0})
