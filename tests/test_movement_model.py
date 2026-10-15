# Majd
"""Check that the app can load and run the saved movement model."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "gui"))
sys.path.insert(0, str(ROOT / "src" / "simulation" / "ai"))

from movement_recognition import DEFAULT_MODEL, MovementRecognizer
from project_support import MOVEMENT_MODEL


@unittest.skipUnless(importlib.util.find_spec("onnxruntime"), "ONNX Runtime is missing")
class MovementModelTests(unittest.TestCase):
    def test_gui_and_recognizer_load_the_same_model_and_produce_finite_output(self):
        self.assertEqual(MOVEMENT_MODEL, DEFAULT_MODEL)
        result = MovementRecognizer().embed(np.zeros((60, 69), dtype=np.float32))
        self.assertEqual(result.shape, (1, 128))
        self.assertTrue(np.all(np.isfinite(result)))


if __name__ == "__main__":
    unittest.main()
