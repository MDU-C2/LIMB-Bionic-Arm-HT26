"""Shared paths and metadata for sensor recording programs."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = REPOSITORY_ROOT / "outputs" / "recordings"


def env_float(name: str, default: float) -> float:
    """Read a non-negative number from an environment variable."""
    value = float(os.environ.get(name, default))
    if value < 0:
        raise ValueError(f"{name} cannot be negative")
    return value


def safe_label(value: str) -> str:
    """Make a short label safe to use in a folder name."""
    label = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip()).strip("._-")
    return label or "session"


def create_session_directory(source: str, subject: str) -> Path:
    """Create one timestamped folder below the selected output directory."""
    output = Path(os.environ.get("AURORA_OUTPUT_DIR", DEFAULT_OUTPUT)).expanduser()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    session = output / f"{timestamp}_{safe_label(subject)}_{source}"
    session.mkdir(parents=True, exist_ok=False)
    return session


def utc_now() -> str:
    """Return an ISO timestamp for saved samples and metadata."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_metadata(session: Path, values: dict[str, object]) -> None:
    """Save final session details beside the recorded data."""
    metadata = {
        "format_version": 1,
        "finished_utc": utc_now(),
        **values,
    }
    (session / "meta.json").write_text(
        json.dumps(metadata, indent=2),
        encoding="utf-8",
    )
