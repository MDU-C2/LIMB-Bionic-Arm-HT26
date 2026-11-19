# Majd
"""Keep the previous metadata readable when saving a replacement fails."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from aurora.json_utils import read_json_object, write_json


class MetadataWriteTests(unittest.TestCase):
    def test_non_finite_metadata_does_not_replace_the_previous_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "training.json"
            write_json(path, {"status": "running"})
            original = path.read_bytes()
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    write_json(path, {"score": value})
                self.assertEqual(path.read_bytes(), original)

    def test_metadata_reader_accepts_a_bom_and_rejects_non_objects(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "trial.json"
            path.write_text('{"trial_id": "T001"}', encoding="utf-8-sig")
            self.assertEqual(read_json_object(path), {"trial_id": "T001"})
            for text in ("[]", "null", '"not metadata"'):
                path.write_text(text, encoding="utf-8")
                with self.assertRaisesRegex(ValueError, "JSON object"):
                    read_json_object(path)

    def test_interrupted_write_keeps_the_previous_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "trial.json"
            original = {"status": "recording", "trial_id": "T001"}
            write_json(path, original)
            write_text = Path.write_text

            def interrupted(destination, text, **kwargs):
                write_text(destination, text[:5], **kwargs)
                raise OSError("Write interrupted")

            with patch.object(Path, "write_text", new=interrupted), self.assertRaises(OSError):
                write_json(path, {"status": "complete"})
            self.assertEqual(json.loads(path.read_text()), original)
            write_json(path, {"status": "complete"})
            self.assertEqual(json.loads(path.read_text()), {"status": "complete"})


if __name__ == "__main__":
    unittest.main()
