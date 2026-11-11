# Christian och Majd
"""Shared contracts for phase-driven human mimic control."""

from __future__ import annotations

import unittest

import numpy as np

from aurora.simulation.dmp.experiment import LIVE_CHANNELS
from aurora.simulation.dmp.mimic import (
    MIMIC_JOINTS,
    HumanMotionSample,
    MimicState,
    MimicTarget,
    ProgressState,
    validate_mimic_joint_count,
    validate_mimic_trajectory,
)


class HumanMotionSampleTests(unittest.TestCase):
    def test_valid_sample_uses_the_declared_joint_order_and_protects_its_angles(self):
        source = np.array([0.1, 0.2, 0.3, 0.4])
        sample = HumanMotionSample(0.25, source, source="synthetic", status="ready")

        self.assertEqual(
            MIMIC_JOINTS,
            (
                "elbow_flexion",
                "shoulder_flexion",
                "shoulder_abduction",
                "shoulder_rotation_proxy",
            ),
        )
        self.assertEqual(LIVE_CHANNELS, MIMIC_JOINTS)
        np.testing.assert_array_equal(sample.angles_rad, source)
        self.assertFalse(sample.angles_rad.flags.writeable)
        source[0] = 9
        self.assertEqual(sample.angles_rad[0], 0.1)

    def test_invalid_sample_may_explicitly_have_no_angles(self):
        sample = HumanMotionSample(
            1.5,
            None,
            valid=False,
            source="camera-imu",
            status="tracking lost",
        )
        self.assertIsNone(sample.angles_rad)
        self.assertFalse(sample.valid)

    def test_valid_samples_reject_missing_malformed_or_nonfinite_angles(self):
        for angles in (None, [0, 1, 2], [0, 1, 2, float("nan")], [[0, 1, 2, 3]]):
            with self.subTest(angles=angles), self.assertRaises(ValueError):
                HumanMotionSample(0, angles)

    def test_sample_metadata_is_validated(self):
        for arguments in (
            {"time_s": -1, "angles_rad": np.zeros(4)},
            {"time_s": float("inf"), "angles_rad": np.zeros(4)},
            {"time_s": 0, "angles_rad": np.zeros(4), "valid": 1},
            {"time_s": 0, "angles_rad": np.zeros(4), "source": " "},
            {"time_s": 0, "angles_rad": np.zeros(4), "status": None},
        ):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                HumanMotionSample(**arguments)


class MimicStateTests(unittest.TestCase):
    def test_progress_state_normalizes_values_and_state(self):
        progress = ProgressState(0.25, 3, 0.8, "running", "matched")
        self.assertEqual(progress.state, MimicState.RUNNING)
        self.assertEqual(progress.active_segment, 3)
        self.assertEqual(progress.progress, 0.25)

    def test_progress_state_rejects_invalid_ranges_segments_and_completion(self):
        for arguments in (
            (-0.1, 0, 1, "running"),
            (1.1, 0, 1, "running"),
            (0.1, -1, 1, "running"),
            (0.1, True, 1, "running"),
            (0.1, 0, float("nan"), "running"),
            (0.1, 0, 1, "other"),
            (0.9, 10, 1, "complete"),
        ):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                ProgressState(*arguments)

    def test_mimic_target_distinguishes_a_target_from_waiting_or_lost_input(self):
        target = MimicTarget(0.5, np.arange(4) / 10, MimicState.RUNNING)
        self.assertFalse(target.angles_rad.flags.writeable)
        self.assertIsNone(MimicTarget(0, None, MimicState.WAITING).angles_rad)
        self.assertIsNone(MimicTarget(0.5, None, MimicState.LOST).angles_rad)
        with self.assertRaisesRegex(ValueError, "requires joint angles"):
            MimicTarget(0.5, None, MimicState.HOLDING)
        with self.assertRaisesRegex(ValueError, "must not contain joint angles"):
            MimicTarget(0.5, np.zeros(4), MimicState.LOST)
        with self.assertRaisesRegex(ValueError, "progress 1"):
            MimicTarget(0.9, np.zeros(4), MimicState.COMPLETE)


class MimicValidationTests(unittest.TestCase):
    def test_reference_trajectory_requires_two_finite_four_joint_samples(self):
        source = np.arange(12, dtype=float).reshape(3, 4)
        validated = validate_mimic_trajectory(source)
        np.testing.assert_array_equal(validated, source)
        self.assertFalse(validated.flags.writeable)
        source[0, 0] = 99
        self.assertEqual(validated[0, 0], 0)

        for values in (
            np.zeros(4),
            np.zeros((1, 4)),
            np.zeros((3, 3)),
            np.full((3, 4), np.inf),
        ):
            with self.subTest(shape=values.shape), self.assertRaises(ValueError):
                validate_mimic_trajectory(values)

    def test_dmp_model_must_have_exactly_the_mimic_joint_count(self):
        self.assertEqual(validate_mimic_joint_count(4), 4)
        for value in (3, 5, 4.0, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_mimic_joint_count(value)


if __name__ == "__main__":
    unittest.main()
