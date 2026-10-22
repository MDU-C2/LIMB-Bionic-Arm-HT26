# Majd
"""Keep the previous metadata readable when saving a replacement fails."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from aurora.json_utils import write_json


class MetadataWriteTests(unittest.TestCase):
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
