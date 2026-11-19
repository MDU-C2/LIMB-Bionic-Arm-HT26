# Jonas Gerne, Majd
"""Read numeric sensor CSV files without changing the source."""

from __future__ import annotations

import csv
from collections.abc import Iterator
from pathlib import Path


def iter_recording(path: Path) -> Iterator[dict[str, float | None]]:
    def numeric(value: str | None) -> float | None:
        try:
            return float(value) if value else None
        except (TypeError, ValueError):
            return None

    with path.open(newline="", encoding="utf-8-sig") as source:
        header = source.readline()
        source.seek(0)
        reader = csv.DictReader(
            source, delimiter=";" if header.count(";") > header.count(",") else ","
        )
        for row in reader:
            yield {key: numeric(value) for key, value in row.items() if key is not None}


def load_recording(path: Path) -> list[dict[str, float | None]]:
    return list(iter_recording(path))
