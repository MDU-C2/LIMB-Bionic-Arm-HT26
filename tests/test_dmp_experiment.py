# Christian och Majd
"""Exercise the future recording workflow without hardware or participant data."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from aurora.gui.tool_catalog import arguments_for
from aurora.simulation.dmp.experiment import (
    actuator_angles,
    load_demonstration,
    make_synthetic_trial,
    run_experiment,
)
from aurora.simulation.dmp.trajectory_io import load_dmp_trajectory


class DmpExperimentTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="DMP Å demo ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = make_synthetic_trial(self.root / "synthetic")

    def manifest(self, **updates):
        path = self.source / "trial.json"
        data = json.loads(path.read_text())
        path.write_text(json.dumps(data | updates))

    def test_cropped_trial_keeps_time_units_unclipped_angles_and_playback_mapping(self):
        paths = [p for p in self.source.rglob("*") if p.is_file()]
        originals = {p: p.read_bytes() for p in paths}
        output = run_experiment(
            self.source / "trial.json",
            self.root / "experiment",
            start=0.4,
            end=3.6,
            duration_scale=2,
        )
        report = json.loads((output / "experiment.json").read_text())
        self.assertEqual(report["status"], "complete")
        self.assertTrue(report["synthetic"])
        self.assertAlmostEqual(report["original_duration_s"], 3.2)
        self.assertAlmostEqual(report["generated_duration_s"], 6.4)
        self.assertIn("proxy", " ".join(report["warnings"]))
        self.assertEqual(report["joint_order"][-1], "shoulder_rotation_proxy")
        with np.load(output / "comparison.npz") as data:
            self.assertAlmostEqual(data["source_time_s"][0], 0.4)
            self.assertAlmostEqual(data["source_time_s"][-1], 3.6)
            self.assertEqual(data["q_demo_rad"].shape, (161, 4))
            expected = actuator_angles(data["q_fit_rad"], report["coordinate_frame"])
            error = np.rad2deg(data["q_fit_rad"] - data["q_demo_rad"])
        actual, dt = load_dmp_trajectory(output)
        np.testing.assert_allclose(actual, expected)
        self.assertAlmostEqual(dt, 0.04)
        _, refit_dt = load_dmp_trajectory(output, prefer_saved_rollout=False)
        self.assertAlmostEqual(refit_dt, 0.02)
        self.assertAlmostEqual(
            report["metrics"]["elbow_flexion"]["rmse_deg"], np.sqrt(np.mean(error[:, 0] ** 2))
        )
        self.assertGreater((output / "comparison.png").stat().st_size, 10000)
        self.assertTrue((output / "model.npz").is_file())
        for path, original in originals.items():
            self.assertEqual(path.read_bytes(), original)
        with self.assertRaises(FileExistsError):
            run_experiment(self.source, output)

    def test_duration_multiplier_preserves_shape_at_the_same_phase(self):
        outputs = [
            run_experiment(self.source, self.root / f"tempo_{scale}", duration_scale=scale)
            for scale in (0.5, 1, 2)
        ]
        arrays = []
        for path in outputs:
            with np.load(path / "comparison.npz") as data:
                arrays.append(data["q_fit_rad"])
        for generated in arrays[1:]:
            np.testing.assert_allclose(generated, arrays[0], atol=1e-12)

    def test_invalid_intervals_incomplete_and_reference_captures_are_rejected(self):
        for options in (
            {"start": -1},
            {"start": 2, "end": 1},
            {"end": 5},
            {"sample_hz": float("nan")},
            {"end": 0.01},
        ):
            with self.subTest(options=options), self.assertRaises(ValueError):
                load_demonstration(self.source, **options)
        self.manifest(complete=False)
        with self.assertRaisesRegex(ValueError, "completed"):
            load_demonstration(self.source)
        self.manifest(complete=True, test_type="subject_reference")
        with self.assertRaisesRegex(ValueError, "reference"):
            load_demonstration(self.source)

    def test_unavailable_interval_is_not_filled_or_time_compressed(self):
        origin = 10_000_000_000
        path = self.source / "ble/packets.jsonl"
        packets = [json.loads(line) for line in path.read_text().splitlines()]
        path.write_text(
            "\n".join(
                json.dumps(p)
                for p in packets
                if not 1 < (p["host_monotonic_ns"] - origin) / 1e9 < 3
            )
        )
        path = self.source / "camera/pose.json"
        data = json.loads(path.read_text())
        data["observations"] = [
            p for p in data["observations"] if not 1 < (p["host_monotonic_ns"] - origin) / 1e9 < 3
        ]
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "continuous valid interval"):
            run_experiment(self.source, self.root / "invalid")
        self.assertFalse((self.root / "invalid").exists())
        # An earlier, continuous portion remains available for explicit selection.
        time, values, _ = load_demonstration(self.source, end=0.8)
        self.assertAlmostEqual(time[-1], 0.8)
        self.assertEqual(values.shape[1], 4)

    def test_reconnection_invalidates_the_selected_interval(self):
        path = self.source / "ble/packets.jsonl"
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        for row in rows[200:]:
            row["connection_generation"] = 1
        path.write_text("\n".join(json.dumps(row) for row in rows))
        with self.assertRaisesRegex(ValueError, "reconnected"):
            load_demonstration(self.source)

    def test_legacy_example_time_assumption_and_prepared_timing_are_explicit(self):
        source = self.root / "angles"
        source.mkdir()
        phase = np.linspace(0, 1, 101)
        q = np.column_stack([0.2 * np.sin(2 * np.pi * phase)] * 4)
        np.savez(source / "angles.npz", elbow_rad=q[:, 0], shoulder_rad=q[:, 1:])
        time, _, metadata = load_demonstration(source)
        self.assertEqual(time[-1], 1)
        self.assertIn("one second", metadata["timing_assumption"])
        (source / "timing.json").write_text('{"duration_seconds": 5, "synthetic": true}')
        time, _, metadata = load_demonstration(source, start=1, end=4)
        self.assertEqual((time[0], time[-1]), (1, 4))
        self.assertTrue(metadata["synthetic"])
        self.assertIsNone(metadata["timing_assumption"])

    def test_source_protection_and_desktop_crop_validation(self):
        with self.assertRaisesRegex(ValueError, "outside"):
            run_experiment(self.source, self.source / "derived")
        with self.assertRaisesRegex(ValueError, "End must"):
            arguments_for(
                "dmp_experiment",
                {
                    "source": str(self.source),
                    "output": str(self.root / "out"),
                    "start": 2,
                    "end": 1,
                },
            )
        for value in (float("nan"), 0, 10):
            with self.subTest(value=value), self.assertRaises(ValueError):
                run_experiment(self.source, self.root / "out", duration_scale=value)


if __name__ == "__main__":
    unittest.main()
