# Majd
"""Signal-level checks of calibrated grip, noise rejection and invalid inputs."""

from __future__ import annotations

import math
import unittest

from aurora.sensors.emg_protocol import EmgActivationEstimator


class LiveEmgCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.rate = 4000
        self.tick = 0
        self.estimator = EmgActivationEstimator(self.rate)

    def feed(self, seconds, amplitude, frequency=120, offset=2048, mains=10):
        for _ in range(round(seconds * self.rate)):
            time = self.tick / self.rate
            sample = (
                offset
                + amplitude * math.sin(2 * math.pi * frequency * time)
                + mains * math.sin(2 * math.pi * 50 * time)
            )
            self.estimator.update(sample, 1 / self.rate)
            self.tick += 1
        return self.estimator.activation

    def test_half_strength_is_repeatable_after_long_rest_and_stronger_effort(self):
        self.feed(1, 2)
        self.assertTrue(self.estimator.calibrate_strength())
        self.feed(3.1, 300)
        reference = self.estimator.reference
        self.assertIsNotNone(reference)
        first = self.feed(1, 150)
        self.assertAlmostEqual(first, 0.5, delta=0.05)
        self.feed(1, 600)
        self.feed(30, 2)
        self.assertEqual(self.estimator.activation, 0)
        self.assertAlmostEqual(self.feed(1, 150), first, delta=0.01)
        self.assertEqual(self.estimator.reference, reference)

    def test_mains_and_slow_electrode_drift_do_not_command_full_grip(self):
        self.feed(1, 2)
        self.estimator.calibrate_strength()
        self.feed(3.1, 300)
        self.feed(2, 0, mains=100)
        self.assertLess(self.estimator.activation, 0.02)
        self.feed(2, 100, frequency=1, offset=2100)
        self.assertLess(self.estimator.activation, 0.02)

    def test_weak_strength_trial_is_rejected(self):
        self.feed(1, 2)
        self.estimator.calibrate_strength()
        self.feed(3.1, 2)
        self.assertIsNone(self.estimator.reference)
        self.assertIn("No clear grip signal", self.estimator.status)

    def test_rest_ignores_notch_settling_after_reset(self):
        self.feed(1, 0, mains=100)
        self.assertLess(self.estimator.rest_envelope, 8)

    def test_small_but_distinct_signal_can_calibrate_without_fixed_gain_cutoff(self):
        self.feed(1, 0.1, mains=0)
        self.estimator.calibrate_strength()
        self.feed(3.1, 6, mains=0)
        self.assertIsNotNone(self.estimator.reference)
        self.assertGreater(self.feed(1, 6, mains=0), 0.9)

    def test_brief_spike_is_not_a_sustained_grip_calibration(self):
        self.feed(1, 2)
        self.estimator.calibrate_strength()
        self.feed(0.6, 300)
        self.feed(2.6, 2)
        self.assertIsNone(self.estimator.reference)

    def test_packet_gap_interrupts_strength_trial_and_suppresses_filter_transient(self):
        self.feed(1, 2)
        self.estimator.calibrate_strength()
        self.feed(1, 300)
        self.estimator.sample_gap()
        self.assertIn("packet gap", self.estimator.status)
        self.assertIsNone(self.feed(0.1, 0, offset=2500))
        self.feed(3, 2, offset=2500)
        self.assertIsNone(self.estimator.reference)
        self.assertIn("missing BLE samples", self.estimator.status)

    def test_clipping_and_nonfinite_samples_have_no_activation(self):
        self.feed(1, 2)
        for _ in range(400):
            self.estimator.update(4095, 1 / self.rate)
        self.assertIsNone(self.estimator.activation)
        self.assertIn("clipping", self.estimator.status)
        self.assertIsNone(self.estimator.update(float("nan")))
        self.assertIsNone(self.estimator.update(2048, float("nan")))


if __name__ == "__main__":
    unittest.main()
