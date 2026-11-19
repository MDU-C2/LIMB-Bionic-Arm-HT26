# Majd
"""Check that a training run keeps its inputs, split and exported model together."""

from __future__ import annotations

import importlib.util
import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

from aurora.ml.experiment import create_manifest, sha256, verify_model_manifest
from aurora.ml.features import PREPROCESSING
from aurora.paths import MODEL_DIRECTORY

TRAINING_AVAILABLE = all(importlib.util.find_spec(name) for name in ("torch", "onnx"))


def profile(subject, run):
    return {
        "user_id": subject,
        "data": [
            {"shoulder": [320, 160, 1.2], "elbow": [330 + run + frame, 260, 1.1], "hand": []}
            for frame in range(3)
        ],
    }


class TrainingManifestTests(unittest.TestCase):
    def test_explicit_selection_requires_provenance_without_removing_legacy_loading(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = Path(temporary) / "movement_gru.onnx"
            model.write_bytes(b"test model")
            self.assertIsNone(verify_model_manifest(model))
            with self.assertRaisesRegex(ValueError, "manifest"):
                verify_model_manifest(model, require_manifest=True)
            manifest = {
                "artifacts": {model.name: sha256(model)},
                "preprocessing": dict(PREPROCESSING),
            }
            path = model.parent / "training.json"
            path.write_text(json.dumps(manifest))
            verify_model_manifest(model)
            with self.assertRaisesRegex(ValueError, "completed"):
                verify_model_manifest(model, require_manifest=True)
            manifest["status"] = "inherited"
            path.write_text(json.dumps(manifest))
            self.assertEqual(verify_model_manifest(model, require_manifest=True), (manifest, path))
            model.with_suffix(".json").write_text(
                json.dumps({**manifest, "preprocessing": {**PREPROCESSING, "centering": "none"}})
            )
            with self.assertRaisesRegex(ValueError, "preprocessing"):
                verify_model_manifest(model, require_manifest=True)

    def test_incompatible_or_unfinished_model_manifests_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = Path(temporary) / "movement_gru.onnx"
            model.write_bytes(b"test model")
            manifest = {
                "format_version": 1,
                "status": "complete",
                "artifacts": {model.name: sha256(model)},
                "preprocessing": dict(PREPROCESSING),
            }
            sidecar = model.with_suffix(".json")
            sidecar.write_text(json.dumps(manifest))
            verify_model_manifest(model)
            for changes in ({"centering": "none"}, {"frames": 30}, {"resampling": "nearest"}):
                sidecar.write_text(
                    json.dumps({**manifest, "preprocessing": {**PREPROCESSING, **changes}})
                )
                with self.assertRaisesRegex(ValueError, "preprocessing"):
                    verify_model_manifest(model)
            sidecar.write_text(json.dumps({**manifest, "status": "running"}))
            with self.assertRaisesRegex(ValueError, "completed"):
                verify_model_manifest(model)

    def test_a_file_cannot_be_in_both_splits(self):
        with tempfile.TemporaryDirectory() as temporary:
            data = Path(temporary)
            (data / "trial.json").write_text(json.dumps(profile("S01", 1)))
            with self.assertRaisesRegex(ValueError, "separate"):
                create_manifest(
                    data, {"training": ["trial.json"], "validation": ["trial.json"]}, {}
                )

    def test_a_stale_model_manifest_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            model = Path(temporary) / "movement_gru.onnx"
            model.write_bytes(b"test model")
            model.with_suffix(".json").write_text(
                json.dumps(
                    {
                        "artifacts": {model.name: "wrong hash"},
                        "preprocessing": {"coordinate_system": "rgb_pixels_stereo_depth"},
                    }
                )
            )
            with self.assertRaisesRegex(ValueError, "do not match"):
                verify_model_manifest(model)

    @unittest.skipUnless(TRAINING_AVAILABLE, "Install the training dependencies")
    def test_interrupted_and_failed_training_keep_a_terminal_run_status(self):
        import torch

        from aurora.ml.train_gru import main

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            data = root / "data"
            data.mkdir()
            for subject in ("S01", "S02"):
                for run in range(3):
                    (data / f"{subject}_{run}.json").write_text(json.dumps(profile(subject, run)))
            for failure, status, exit_code in (
                (KeyboardInterrupt(), "interrupted", 130),
                (RuntimeError("training failed"), "failed", 1),
            ):
                with self.subTest(status=status):
                    output = root / status
                    with (
                        patch.object(torch.cuda, "is_available", return_value=False),
                        patch("aurora.ml.train_gru.run_epoch", side_effect=failure),
                        redirect_stdout(io.StringIO()),
                    ):
                        self.assertEqual(
                            main(["--data", str(data), "--output", str(output), "--epochs", "1"]),
                            exit_code,
                        )
                    manifest = json.loads((output / "training.json").read_text())
                    self.assertEqual(manifest["status"], status)
                    self.assertEqual(manifest["epochs_completed"], 0)
                    self.assertIn("finished_utc", manifest)
                    self.assertFalse((output / "movement_gru.onnx").exists())

    @unittest.skipUnless(TRAINING_AVAILABLE, "Install the training dependencies")
    def test_one_epoch_exports_a_complete_run_without_overwriting_the_app_model(self):
        import torch

        from aurora.ml.train_gru import main

        bundled = MODEL_DIRECTORY / "movement_gru.onnx"
        original_hash = sha256(bundled)
        threads = torch.get_num_threads()
        try:
            torch.set_num_threads(1)
            with tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                data = root / "data"
                data.mkdir()
                for subject in ("S01", "S02"):
                    for run in range(3):
                        (data / f"{subject}_{run}.json").write_text(
                            json.dumps(profile(subject, run))
                        )
                output = root / "run"
                arguments = [
                    "--data",
                    str(data),
                    "--output",
                    str(output),
                    "--epochs",
                    "1",
                    "--seed",
                    "12",
                ]
                with (
                    patch.object(torch.cuda, "is_available", return_value=False),
                    redirect_stdout(io.StringIO()) as log,
                ):
                    self.assertEqual(main(arguments), 0, log.getvalue())
                manifest = json.loads((output / "training.json").read_text())
                self.assertEqual(manifest["status"], "complete")
                self.assertEqual(manifest["settings"]["seed"], 12)
                self.assertEqual(manifest["epochs_completed"], 1)
                self.assertEqual(len(manifest["files"]), 6)
                training = {
                    entry["file"] for entry in manifest["files"] if entry["split"] == "training"
                }
                validation = {
                    entry["file"] for entry in manifest["files"] if entry["split"] == "validation"
                }
                self.assertEqual((len(training), len(validation)), (2, 4))
                self.assertFalse(training & validation)
                for entry in manifest["files"]:
                    self.assertEqual(entry["sha256"], sha256(data / entry["file"]))
                for name, digest in manifest["artifacts"].items():
                    self.assertEqual(digest, sha256(output / name))
                original = (output / "training.json").read_bytes()
                with redirect_stdout(io.StringIO()):
                    self.assertEqual(main(arguments), 1)
                self.assertEqual((output / "training.json").read_bytes(), original)
                self.assertEqual(sha256(bundled), original_hash)
        finally:
            torch.set_num_threads(threads)


if __name__ == "__main__":
    unittest.main()
