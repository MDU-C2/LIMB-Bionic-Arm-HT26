"""DMP movement reproduction and inherited rollout loading without windows."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from aurora.simulation.dmp.dmp import fit, rollout_simple
from aurora.simulation.dmp.trajectory_io import load_dmp_trajectory


class DmpPlaybackTests(unittest.TestCase):
    def test_returning_movements_keep_their_amplitude_for_equal_and_nearby_endpoints(self):
        time = np.linspace(0, 1, 201)
        for displacement in (0, 0.005, -0.005, 0.3):
            with self.subTest(displacement=displacement):
                demo = (0.4 * np.sin(2 * np.pi * time) + displacement * time)[:, None]
                model = fit(
                    [demo],
                    tau=1,
                    dt=0.005,
                    n_basis_functions=30,
                    alpha_canonical=4,
                    alpha_transformation=25,
                    beta_transformation=6.25,
                )
                generated = rollout_simple(model, demo[0], demo[-1], 1, 0.005)
                self.assertEqual(generated.shape, demo.shape)
                np.testing.assert_array_equal(generated[0], demo[0])
                # This bounds numerical reproduction error on an analytic signal,
                # not anatomical or hardware tracking error.
                self.assertLess(float(np.sqrt(np.mean((generated - demo) ** 2))), 0.035)
                self.assertGreater(float(np.ptp(generated)), 0.6)

    def test_angles_fallback_reproduces_a_return_without_modifying_its_source(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            time = np.linspace(0, 1, 201)
            elbow = 0.4 * np.sin(2 * np.pi * time)
            shoulder = np.column_stack((elbow / 2, elbow / 3, elbow / 4))
            source = directory / "angles.npz"
            np.savez(source, elbow_rad=elbow, shoulder_rad=shoulder)
            original = source.read_bytes()
            generated, dt = load_dmp_trajectory(directory)
            self.assertEqual(dt, 0.005)
            self.assertEqual(generated.shape, (201, 4))
            self.assertLess(float(np.sqrt(np.mean((generated[:, 0] - elbow) ** 2))), 0.035)
            self.assertEqual(source.read_bytes(), original)

    def test_precomputed_rollout_keeps_its_original_samples_and_timestep(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            expected = np.arange(80, dtype=float).reshape(20, 4) / 100
            path = directory / "dmp_rollout_clean.npz"
            np.savez(path, q_gen_rad=expected, dt=0.02)
            original = path.read_bytes()
            actual, dt = load_dmp_trajectory(directory)
            np.testing.assert_array_equal(actual, expected)
            self.assertEqual(dt, 0.02)
            self.assertEqual(path.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
