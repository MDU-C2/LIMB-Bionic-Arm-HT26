# Majd
"""Known planar trajectories check filter math, not hardware accuracy."""

from __future__ import annotations

import unittest

import numpy as np

from aurora.sensors.fusion_comparison import compare, correct, scores, synthetic_samples


class FusionComparisonTests(unittest.TestCase):
    def test_synthetic_outputs_are_finite_with_camera_dropout(self):
        values = synthetic_samples()
        outputs = compare(values)
        self.assertEqual(len(outputs), 4)
        for name, estimate in outputs.items():
            with self.subTest(name=name):
                self.assertEqual(estimate.shape, (len(values),))
                self.assertTrue(np.isfinite(estimate).all())
                self.assertLess(scores(values[:, 5], estimate, values[:, 0])["rmse_deg"], 12)

    def test_neutral_static_pose_stays_neutral_and_unclipped(self):
        time = np.arange(100) / 100
        angle = 165.0
        values = np.column_stack(
            (
                time,
                np.zeros(100),
                np.full(100, np.sin(np.radians(angle))),
                np.full(100, np.cos(np.radians(angle))),
                np.full(100, angle),
                np.full(100, angle),
            )
        )
        for estimated in compare(values).values():
            np.testing.assert_allclose(estimated, angle, atol=1e-8)

    def test_covariance_update_stays_symmetric_and_positive(self):
        state, covariance = np.array([0.0, 0.0]), np.eye(2)
        for _index in range(100):
            state, covariance = correct(
                state, covariance, 0.1, np.array([[1.0, 0.0]]), np.array([[0.01]])
            )
        np.testing.assert_allclose(covariance, covariance.T)
        self.assertTrue(np.all(np.linalg.eigvalsh(covariance) > 0))

    def test_invalid_clock_or_reference_is_rejected(self):
        values = synthetic_samples()[:20]
        values[5, 0] = values[4, 0]
        with self.assertRaises(ValueError):
            compare(values)
        values = synthetic_samples()[:20]
        values[5, 5] = np.nan
        with self.assertRaises(ValueError):
            compare(values)


if __name__ == "__main__":
    unittest.main()
