"""Known rigid-arm trajectories check mounting and anatomical/robot separation."""

from __future__ import annotations

import math
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
from test_arm_alignment import generated_reference

from aurora.sensors.arm_alignment import fit_alignment
from aurora.sensors.ble_input import BleSensorReader
from aurora.sensors.fusion import DualImuArmEstimator, interactive_control_targets
from aurora.sensors.live_input import LiveSensorInput


class AnatomicalProcessingTests(unittest.TestCase):
    def test_an_explicitly_hidden_trunk_cannot_keep_a_camera_body_reference(self):
        live = LiveSensorInput(show_monitor=False, clock=lambda: 100.0)
        live.accept_camera_observation(
            {"angles_deg": {"elbow_flexion": 90, "shoulder_flexion": 20}}, 100.0
        )
        self.assertEqual(live.poll().body_reference, "camera_trunk")
        live.accept_camera_observation({"person_visible": False, "reference_visible": False}, 100.0)
        self.assertNotEqual(live.poll().body_reference, "camera_trunk")

    def test_sensor_only_zero_is_relative_and_signed_until_a_human_pose_is_observed(self):
        clock = [100.0]
        live = LiveSensorInput(transport="serial", show_monitor=False, clock=lambda: clock[0])
        live.estimator = DualImuArmEstimator(alpha=0)

        def sample(elbow):
            clock[0] += 0.01
            return {
                "connection_generation": 1,
                "host_monotonic_ns": round(clock[0] * 1e9),
                "imus": {
                    role: {
                        "connected": True,
                        "sample_time_us": round(clock[0] * 1e6),
                        "host_monotonic_ns": round(clock[0] * 1e9),
                        "accel_g": dict(
                            x=0, y=math.sin(math.radians(angle)), z=math.cos(math.radians(angle))
                        ),
                        "gyro_dps": dict.fromkeys("xyz", 0),
                    }
                    for role, angle in (("shoulder", 0), ("wrist", elbow))
                },
            }

        live.feed_serial(sample(0))
        live.calibrate({"elbow_flexion": 45})
        for change in (0, 10, -10):
            live.feed_serial(sample(change))
            state = live.poll()
            self.assertIsNone(state.angles)
            self.assertAlmostEqual(state.relative_angles["elbow_flexion"], change)
            self.assertAlmostEqual(state.robot_targets["elbow_x"], 45 + change)
        live.accept_camera_observation({"angles_deg": {"elbow_flexion": 95}}, clock[0])
        first_observed = live.poll()
        self.assertAlmostEqual(first_observed.angles["elbow_flexion"], 95)
        self.assertAlmostEqual(first_observed.robot_targets["elbow_x"], 35)
        live.calibrate({"elbow_flexion": 45})
        live.feed_serial(sample(-10))
        state = live.poll()
        self.assertAlmostEqual(state.angles["elbow_flexion"], 95)
        self.assertNotIn("elbow_flexion", state.relative_angles or {})
        self.assertEqual(state.robot_targets["elbow_x"], 45)

    def test_outward_motion_has_the_same_anatomical_sign_for_either_arm(self):
        for side in ("left", "right"):
            stages, camera, mounting, bias = generated_reference(side)
            profile = fit_alignment(stages, camera, side=side)
            estimator = DualImuArmEstimator(alpha=0, gyro_deadzone_dps=0)
            estimator.set_alignment(profile)
            references = {
                role: np.asarray(value)
                for role, value in profile["camera_from_segment_at_reference"].items()
            }
            sign = 1 if side == "left" else -1
            for stamp in np.arange(0.01, 1.001, 0.01):
                upper = (
                    references["shoulder"]
                    @ Rotation.from_rotvec([0, 0, np.radians(sign * 20 * stamp)]).as_matrix()
                )
                orientations = {
                    "shoulder": upper,
                    "wrist": upper @ references["shoulder"].T @ references["wrist"],
                }
                rate_camera = references["shoulder"][:, 2] * sign * 20
                sensors = {}
                for role, orientation in orientations.items():
                    accel = (orientation.T @ [0, 1, 0]) @ mounting[role]
                    gyro = (orientation.T @ rate_camera) @ mounting[role] + bias[role]
                    sensors[role] = {
                        "connected": True,
                        "accel_g": dict(zip("xyz", accel, strict=True)),
                        "gyro_dps": dict(zip("xyz", gyro, strict=True)),
                    }
                angles = estimator.update(sensors, 0.01)
            self.assertAlmostEqual(angles["shoulder_abduction"], 20, delta=0.01)
            self.assertAlmostEqual(angles["elbow_flexion"], 90, delta=0.01)
            self.assertAlmostEqual(angles["shoulder_flexion"], 0, delta=0.01)
            self.assertAlmostEqual(angles["wrist_rotation"], 0, delta=0.01)

    def test_arbitrary_mountings_keep_isolated_raise_and_bend_separate(self):
        for side in ("left", "right"):
            stages, camera, _, _ = generated_reference(side)
            profile = fit_alignment(stages, camera, side=side)
            for stage in ("RAISE", "BEND", "TURN", "RETURN"):
                with self.subTest(side=side, stage=stage):
                    estimator = DualImuArmEstimator(alpha=0)
                    estimator.set_alignment(profile)
                    angles = []
                    for index in range(len(stages[stage]["shoulder"].time_s)):
                        sensors = {
                            role: {
                                "connected": True,
                                "accel_g": dict(
                                    zip("xyz", stages[stage][role].accel_g[index], strict=True)
                                ),
                                "gyro_dps": dict(
                                    zip("xyz", stages[stage][role].gyro_dps[index], strict=True)
                                ),
                            }
                            for role in ("shoulder", "wrist")
                        }
                        angles.append(estimator.update(sensors, 0.01))
                    elbow = np.array([value["elbow_flexion"] for value in angles])
                    shoulder = np.array([value["shoulder_flexion"] for value in angles])
                    if stage == "BEND":
                        self.assertAlmostEqual(float(elbow.max()), 135, delta=0.01)
                        self.assertLess(float(np.abs(shoulder).max()), 0.01)
                        self.assertEqual(
                            interactive_control_targets(angles[len(angles) // 4])["elbow_x"], 60
                        )
                    elif stage == "RAISE":
                        self.assertAlmostEqual(float(shoulder.max()), 45, delta=0.01)
                        self.assertLess(float(np.abs(elbow - 90).max()), 0.01)
                    else:
                        self.assertLess(float(np.abs(elbow - 90).max()), 0.01)
                        self.assertLess(float(np.abs(shoulder).max()), 0.01)
                    if stage == "TURN":
                        wrist = np.array([value["wrist_rotation"] for value in angles])
                        self.assertGreater(float(np.ptp(wrist)), 150)
                        self.assertLess(
                            max(abs(value["shoulder_rotation_proxy"]) for value in angles), 0.01
                        )

    def test_alignment_cannot_cross_subject_side_or_connection(self):
        stages, camera, _, _ = generated_reference()
        profile = fit_alignment(stages, camera, side="left")
        reader = BleSensorReader()
        profile.update(
            subject_id="S01",
            connection_generation=reader.connection_generation,
            mounting_session_id=reader.mounting_session_id,
        )
        reader.apply_alignment(profile, subject_id="S01", side="left")
        self.assertIsNotNone(reader.imu_estimator.alignment_profile)
        for subject, side in (("S02", "left"), ("S01", "right")):
            with self.assertRaisesRegex(ValueError, "another subject"):
                reader.apply_alignment(profile, subject_id=subject, side=side)
        reader._start_connection()
        with self.assertRaisesRegex(ValueError, "another subject"):
            reader.apply_alignment(profile, subject_id="S01", side="left")
        reader._clear_state()
        self.assertEqual(reader.imu_estimator.sensor_rotations, {})


if __name__ == "__main__":
    unittest.main()
