"""Stream file checksums without importing processing or loading whole recordings."""

import hashlib
from pathlib import Path


def file_digest(path: Path) -> str:
    checksum = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(block)
    return checksum.hexdigest()
