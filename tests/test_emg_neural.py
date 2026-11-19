# Majd
"""Train tiny networks and verify their portable inference bundles."""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from aurora.json_utils import write_json
from aurora.ml.emg_model import EmgModel, LiveEmgClassifier
from aurora.ml.train_emg import fit_model
from aurora.sensors.emg_window import EmgWindowSettings


@unittest.skipUnless(
    all(importlib.util.find_spec(name) for name in ("torch", "onnx")),
    "Optional neural training requires torch and onnx",
)
class EmgNeuralTests(unittest.TestCase):
    def test_full_comparison_in_a_fresh_process_isolates_neural_training(self):
        from test_emg_learning import recording

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for index in range(2):
                recording(root / "data", f"S0{index + 1}", seed=index)
            environment = {**os.environ, "PYTHONUTF8": "1"}
            result = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "aurora.ml.train_emg",
                    "--data",
                    str(root / "data"),
                    "--output",
                    str(root / "run"),
                    "--models",
                    "lda",
                    "ann",
                    "lstm",
                    "--features",
                    "basic",
                    "full",
                    "--epochs",
                    "1",
                    "--sequence-length",
                    "2",
                    "--rate",
                    "1000",
                ],
                env=environment,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            document = json.loads((root / "run/training.json").read_text(encoding="utf-8"))
            self.assertEqual(document["status"], "complete")
            self.assertEqual(len(document["folds"][0]["models"]), 6)

    def test_ann_and_lstm_export_round_trip(self):
        random = np.random.default_rng(42)
        x = random.normal(0, 1, (24, 2, 6))
        y = np.arange(len(x), dtype=np.int64) % 4
        with tempfile.TemporaryDirectory() as temp:
            for kind in ("ann", "lstm"):
                with self.subTest(kind=kind):
                    directory = Path(temp) / kind
                    model = fit_model(
                        x,
                        y,
                        kind,
                        list(range(6)),
                        EmgWindowSettings(rate_hz=1000),
                        directory,
                        epochs=2,
                        seed=42,
                    )
                    scores = model.predict(x)
                    self.assertEqual(scores.shape, (24, 4))
                    self.assertTrue(np.isfinite(scores).all())
                    np.testing.assert_allclose(scores.sum(axis=1), 1, rtol=1e-6)
                    np.testing.assert_allclose(scores[:1], model.predict(x[:1]), rtol=1e-5)
                    document = json.loads((directory / "model.json").read_text(encoding="utf-8"))
                    document["reviewed_for_live"] = True
                    write_json(directory / "model.json", document)
                    np.testing.assert_array_equal(EmgModel(directory).predict(x), scores)
                    live = LiveEmgClassifier(directory)
                    for value in 2048 + random.normal(0, 10, 1000):
                        live.update(value)
                    self.assertIsNotNone(live.phase)


if __name__ == "__main__":
    unittest.main()
