"""Export real files without changing sources or publishing partial bundles."""

from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
import zipfile
from pathlib import Path

import numpy as np

from aurora.recording.exports import export_dmp, export_recording


class RecordingExportTests(unittest.TestCase):
    def test_trial_bundle_preserves_raw_bytes_incomplete_status_and_hashes(self):
        with tempfile.TemporaryDirectory(prefix="aurora export Å ") as temporary:
            root = Path(temporary)
            trial = root / "original"
            (trial / "camera/depth").mkdir(parents=True)
            (trial / "trial.json").write_text('{"complete": false, "pace_condition": "slow"}')
            (trial / "camera/depth/0001.npz").write_bytes(bytes(range(256)))
            original = {
                p.relative_to(trial).as_posix(): p.read_bytes()
                for p in trial.rglob("*")
                if p.is_file()
            }
            progress = []
            bundle = export_recording(
                trial / "trial.json",
                root / "bundle.zip",
                progress=lambda done, total: progress.append((done, total)),
            )
            with zipfile.ZipFile(bundle) as archive:
                manifest = json.loads(archive.read("AURORA-export.json"))
                self.assertEqual(len(manifest["files"]), len(original))
                for item in manifest["files"]:
                    content = archive.read("recording/" + item["file"])
                    self.assertEqual(content, original[item["file"]])
                    self.assertEqual(item["sha256"], hashlib.sha256(content).hexdigest())
                self.assertFalse(json.loads(archive.read("recording/trial.json"))["complete"])
            self.assertEqual(progress, [(0, 2), (1, 2), (2, 2)])
            self.assertEqual(
                {
                    p.relative_to(trial).as_posix(): p.read_bytes()
                    for p in trial.rglob("*")
                    if p.is_file()
                },
                original,
            )

    def test_cancel_removes_partial_output_but_never_removes_existing_destination(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "input.csv"
            source.write_bytes(b"original" * 100)
            output = root / "export.zip"
            with self.assertRaises(InterruptedError):
                export_recording(source, output, cancelled=lambda: True)
            self.assertFalse(output.exists())
            output.write_bytes(b"existing export")
            with self.assertRaises(FileExistsError):
                export_recording(source, output)
            self.assertEqual(output.read_bytes(), b"existing export")
            self.assertEqual(source.read_bytes(), b"original" * 100)

    def test_changed_recording_is_rejected_and_new_partial_archive_removed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "input.csv"
            source.write_text("first version")

            def progress(done, total):
                if done:
                    source.write_text("longer changed version")

            with self.assertRaisesRegex(ValueError, "changed during export"):
                export_recording(source, root / "partial.zip", progress=progress)
            self.assertFalse((root / "partial.zip").exists())
            with self.assertRaisesRegex(ValueError, "outside"):
                export_recording(root, root / "inside.zip")

    def test_dmp_fit_exports_unclipped_radians_and_a_reusable_rollout(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "original"
            source.mkdir()
            times = np.linspace(0, 1, 201)
            elbow = 2 * np.sin(np.pi * times)
            np.savez(
                source / "angles.npz",
                elbow_rad=elbow,
                shoulder_rad=np.column_stack((elbow / 2, elbow / 3, elbow / 4)),
            )
            original = (source / "angles.npz").read_bytes()
            output = export_dmp(source, root / "fitted")
            manifest = json.loads((output / "trajectory.json").read_text())
            self.assertEqual(manifest["status"], "complete")
            self.assertEqual(manifest["units"], "rad")
            values = np.loadtxt(output / "trajectory.csv", delimiter=",", skiprows=1)
            self.assertEqual(values.shape, (201, 5))
            self.assertGreater(values[:, 1].max(), 1.4)
            with np.load(output / "dmp_rollout_clean.npz") as rollout:
                np.testing.assert_array_equal(rollout["q_gen_rad"], values[:, 1:])
                self.assertAlmostEqual(float(rollout["dt"]), 0.005)
            self.assertEqual((source / "angles.npz").read_bytes(), original)
            self.assertEqual(
                manifest["source_sha256"]["angles.npz"], hashlib.sha256(original).hexdigest()
            )


if __name__ == "__main__":
    unittest.main()
