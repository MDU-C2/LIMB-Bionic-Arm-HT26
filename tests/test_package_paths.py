# Majd
"""Check package extraction through path aliases and reject unsafe entries."""

from __future__ import annotations

import io
import tarfile
import tempfile
import unittest
from pathlib import Path

from scripts.check_package import extract_source_archive


class PackagePathTests(unittest.TestCase):
    def test_valid_archive_accepts_a_destination_with_parent_segments(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "alias").mkdir()
            destination = root / "alias" / ".." / "source"
            destination.mkdir()
            archive = root / "source.tar.gz"
            content = b"package contents\n"
            with tarfile.open(archive, "w:gz") as output:
                entry = tarfile.TarInfo("package/README.md")
                entry.size = len(content)
                output.addfile(entry, io.BytesIO(content))
            extract_source_archive(archive, destination)
            self.assertEqual((root / "source/package/README.md").read_bytes(), content)

    def test_unsafe_entries_are_rejected_before_extracting_any_files(self):
        for name, kind in (
            ("../escape.txt", tarfile.REGTYPE),
            ("package/link", tarfile.SYMTYPE),
            ("package/link", tarfile.LNKTYPE),
            ("package/pipe", tarfile.FIFOTYPE),
        ):
            with self.subTest(name=name, kind=kind), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                destination = root / "source"
                destination.mkdir()
                archive = root / "source.tar.gz"
                with tarfile.open(archive, "w:gz") as output:
                    output.addfile(tarfile.TarInfo("package/safe.txt"), io.BytesIO())
                    entry = tarfile.TarInfo(name)
                    entry.type = kind
                    if kind in (tarfile.SYMTYPE, tarfile.LNKTYPE):
                        entry.linkname = "../escape.txt"
                    output.addfile(entry, io.BytesIO() if kind == tarfile.REGTYPE else None)
                with self.assertRaisesRegex(ValueError, "Unexpected source archive entry"):
                    extract_source_archive(archive, destination)
                self.assertEqual(list(destination.iterdir()), [])
                self.assertFalse((root / "escape.txt").exists())


if __name__ == "__main__":
    unittest.main()
