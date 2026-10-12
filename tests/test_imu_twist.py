# Christian, Majd
"""Check axial gyro mapping, drift correction, calibration and joint limits."""

from __future__ import annotations

import copy
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/simulation/ai"))

from sensor_fusion import DualImuArmEstimator, interactive_control_targets


def resting_sensors():
    return {
        role: {
            "connected": True,
            "accel_g": {"x": 0.0, "y": 1.0, "z": 0.0},
            "gyro_dps": dict.fromkeys("xyz", 0.0),
        }
        for role in ("shoulder", "wrist")
    }


def calibrate(estimator, sensors, period=0.01):
    for _ in range(round(0.5 / period) + 1):
        angles = estimator.update(sensors, period)
    return angles


class ImuTwistTests(unittest.TestCase):
    def test_matched_upper_arm_motion_does_not_turn_the_wrist(self):
        estimator = DualImuArmEstimator(alpha=0)
        sensors = resting_sensors()
        calibrate(estimator, sensors)
        for sensor in sensors.values():
            sensor["gyro_dps"]["y"] = 30.0
        for _ in range(10):
            angles = estimator.update(sensors, 0.1)
        targets = interactive_control_targets(angles)
        self.assertAlmostEqual(targets["shoulder_x"], 30.0)
        self.assertAlmostEqual(targets["wrist_rotation"], 0.0)

    def test_forearm_rotates_in_both_directions_without_turning_upper_arm(self):
        estimator = DualImuArmEstimator(alpha=0)
        sensors = resting_sensors()
        calibrate(estimator, sensors)
        sensors["wrist"]["gyro_dps"]["y"] = 40.0
        for _ in range(10):
            angles = estimator.update(sensors, 0.1)
        self.assertAlmostEqual(angles["wrist_rotation"], 40.0)
        self.assertAlmostEqual(angles["shoulder_rotation_proxy"], 0.0)
        sensors["wrist"]["gyro_dps"]["y"] = -30.0
        for _ in range(10):
            angles = estimator.update(sensors, 0.1)
        self.assertAlmostEqual(angles["wrist_rotation"], 10.0)

    def test_shared_rotation_at_a_bent_elbow_does_not_create_wrist_twist(self):
        estimator = DualImuArmEstimator(alpha=0)
        sensors = resting_sensors()
        calibrate(estimator, sensors)
        bend = math.radians(60.0)
        sensors["wrist"]["accel_g"] = {
            "x": 0.0, "y": math.cos(bend), "z": -math.sin(bend),
        }
        sensors["shoulder"]["gyro_dps"] = {"x": 0.0, "y": 30.0, "z": 20.0}
        sensors["wrist"]["gyro_dps"] = {
            "x": 0.0,
            "y": 30.0 * math.cos(bend) + 20.0 * math.sin(bend),
            "z": -30.0 * math.sin(bend) + 20.0 * math.cos(bend),
        }
        for _ in range(10):
            angles = estimator.update(sensors, 0.1)
        self.assertAlmostEqual(angles["elbow_flexion"], 60.0)
        self.assertAlmostEqual(angles["shoulder_rotation_proxy"], -30.0)
        self.assertEqual(angles["wrist_rotation"], 0.0)

    def test_elbow_bend_is_not_mapped_to_wrist_twist(self):
        estimator = DualImuArmEstimator(alpha=0)
        sensors = resting_sensors()
        calibrate(estimator, sensors)
        sensors["wrist"]["accel_g"] = {"x": 0.0, "y": 0.8660254, "z": 0.5}
        sensors["wrist"]["gyro_dps"]["x"] = 30.0
        angles = estimator.update(sensors, 0.1)
        self.assertAlmostEqual(angles["elbow_flexion"], 30.0, places=5)
        self.assertEqual(angles["wrist_rotation"], 0.0)
        self.assertEqual(angles["shoulder_rotation_proxy"], 0.0)

    def test_signed_axes_support_different_sensor_mounts(self):
        estimator = DualImuArmEstimator(shoulder_twist_axis="-z", wrist_twist_axis="x")
        sensors = resting_sensors()
        calibrate(estimator, sensors)
        sensors["shoulder"]["gyro_dps"]["z"] = -30.0
        sensors["wrist"]["gyro_dps"]["x"] = 30.0
        for _ in range(10):
            angles = estimator.update(sensors, 0.1)
        self.assertAlmostEqual(angles["shoulder_rotation_proxy"], -30.0)
        self.assertEqual(angles["wrist_rotation"], 0.0)
        with self.assertRaises(ValueError):
            DualImuArmEstimator(shoulder_twist_axis="guess")

    def test_rest_bias_and_deadzone_keep_twist_stationary(self):
        estimator = DualImuArmEstimator()
        sensors = resting_sensors()
        sensors["shoulder"]["gyro_dps"]["y"] = 1.5
        sensors["wrist"]["gyro_dps"]["y"] = -1.0
        calibrate(estimator, sensors)
        sensors["shoulder"]["gyro_dps"]["y"] = 2.0
        sensors["wrist"]["gyro_dps"]["y"] = -1.5
        for _ in range(300):
            angles = estimator.update(sensors, 0.1)
        self.assertEqual(angles["shoulder_rotation_proxy"], 0.0)
        self.assertEqual(angles["wrist_rotation"], 0.0)

    def test_reversing_at_a_limit_moves_immediately(self):
        estimator = DualImuArmEstimator()
        sensors = resting_sensors()
        calibrate(estimator, sensors)
        sensors["shoulder"]["gyro_dps"]["y"] = 200.0
        sensors["wrist"]["gyro_dps"]["y"] = 400.0
        for _ in range(30):
            angles = estimator.update(sensors, 0.1)
        self.assertEqual(angles["shoulder_rotation_proxy"], -60.0)
        self.assertEqual(angles["wrist_rotation"], 140.0)
        sensors["shoulder"]["gyro_dps"]["y"] = -30.0
        sensors["wrist"]["gyro_dps"]["y"] = -60.0
        angles = estimator.update(sensors, 0.1)
        self.assertAlmostEqual(angles["shoulder_rotation_proxy"], -57.0)
        self.assertAlmostEqual(angles["wrist_rotation"], 137.0)

    def test_twist_is_independent_of_packet_rate(self):
        results = []
        for period in (0.01, 0.2):
            estimator = DualImuArmEstimator()
            sensors = resting_sensors()
            calibrate(estimator, sensors, period)
            sensors["shoulder"]["gyro_dps"]["y"] = 15.0
            sensors["wrist"]["gyro_dps"]["y"] = 45.0
            for _ in range(round(1.0 / period)):
                angles = estimator.update(sensors, period)
            results.append(angles)
        self.assertAlmostEqual(results[0]["shoulder_rotation_proxy"], -15.0)
        self.assertAlmostEqual(results[0]["wrist_rotation"], 30.0)
        self.assertAlmostEqual(results[1]["shoulder_rotation_proxy"], -15.0)
        self.assertAlmostEqual(results[1]["wrist_rotation"], 30.0)

    def test_camera_corrects_state_without_changing_relative_wrist(self):
        estimator = DualImuArmEstimator()
        sensors = resting_sensors()
        calibrate(estimator, sensors)
        sensors["shoulder"]["gyro_dps"]["y"] = 20.0
        sensors["wrist"]["gyro_dps"]["y"] = 60.0
        for _ in range(10):
            estimator.update(sensors, 0.1)
        self.assertEqual(estimator.correct_rotation(-30.0, 0.25), -30.0)
        angles = estimator.update(sensors, 0.1)
        self.assertAlmostEqual(angles["shoulder_rotation_proxy"], -32.0)
        self.assertAlmostEqual(angles["wrist_rotation"], 44.0)
        self.assertAlmostEqual(estimator.correct_rotation(-30.0, 0.25), -31.5)
        angles = estimator.update(sensors, 0.1)
        self.assertAlmostEqual(angles["shoulder_rotation_proxy"], -33.5)
        self.assertAlmostEqual(angles["wrist_rotation"], 48.0)

    def test_calibration_rejects_movement_and_requires_rest(self):
        estimator = DualImuArmEstimator()
        sensors = resting_sensors()
        estimator.update(sensors, 0.2)
        sensors["shoulder"]["gyro_dps"]["y"] = 20.0
        angles = estimator.update(sensors, 0.2)
        self.assertNotIn("shoulder_rotation_proxy", angles)
        sensors["shoulder"]["gyro_dps"]["y"] = 0.0
        self.assertNotIn("shoulder_rotation_proxy", estimator.update(sensors, 0.3))
        self.assertEqual(estimator.update(sensors, 0.2)["shoulder_rotation_proxy"], 0.0)

    def test_missing_sensor_and_long_gap_do_not_preserve_an_old_twist(self):
        estimator = DualImuArmEstimator()
        sensors = resting_sensors()
        calibrate(estimator, sensors)
        wrist_only = {"wrist": copy.deepcopy(sensors["wrist"])}
        self.assertEqual(estimator.update(wrist_only, 0.1), {})
        self.assertIsNone(estimator.correct_rotation(30.0, 0.25))
        calibrate(estimator, sensors)
        self.assertNotIn("wrist_rotation", estimator.update({"shoulder": sensors["shoulder"]}, 0.1))
        self.assertNotIn("shoulder_rotation_proxy", estimator.update(sensors, 1.0))


if __name__ == "__main__":
    unittest.main()
