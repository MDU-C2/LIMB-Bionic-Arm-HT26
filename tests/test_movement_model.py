# Majd
"""Check that the app can load and run the saved movement model."""

from __future__ import annotations

import importlib.util
import unittest

import numpy as np

from aurora.gui.project_support import MOVEMENT_MODEL
from aurora.ml.recognition import DEFAULT_MODEL, MovementRecognizer


@unittest.skipUnless(importlib.util.find_spec("onnxruntime"), "ONNX Runtime is missing")
class MovementModelTests(unittest.TestCase):
    def test_gui_and_recognizer_load_the_same_model_and_produce_finite_output(self):
        self.assertEqual(MOVEMENT_MODEL, DEFAULT_MODEL)
        result = MovementRecognizer().embed(np.zeros((60, 69), dtype=np.float32))
        self.assertEqual(result.shape, (1, 128))
        self.assertTrue(np.all(np.isfinite(result)))

    def test_empty_or_nonfinite_movements_are_rejected_before_inference(self):
        recognizer = MovementRecognizer()
        for batch in (
            np.empty((0, 60, 69), dtype=np.float32),
            np.full((60, 69), np.nan, dtype=np.float32),
            np.full((60, 69), np.inf, dtype=np.float32),
        ):
            with self.subTest(shape=batch.shape), self.assertRaisesRegex(ValueError, "finite"):
                recognizer.embed(batch)


if __name__ == "__main__":
    unittest.main()
