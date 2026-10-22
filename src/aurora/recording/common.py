# Majd
"""Shared paths and metadata for sensor recording programs."""

from __future__ import annotations

import math
import os
import re
import signal
from datetime import datetime
from pathlib import Path

from aurora.json_utils import write_json
from aurora.paths import REPOSITORY_ROOT
from aurora.time_utils import utc_now

DEFAULT_OUTPUT = REPOSITORY_ROOT / "outputs" / "recordings"


def enable_ctrl_break_as_interrupt() -> None:
    """Let the GUI's Windows stop request reach recorder cleanup blocks."""
    if hasattr(signal, "SIGBREAK"):

        def interrupt(_number: int, _frame: object) -> None:
            raise KeyboardInterrupt

        signal.signal(signal.SIGBREAK, interrupt)


def env_float(name: str, default: float) -> float:
    """Read a non-negative number from an environment variable."""
    value = float(os.environ.get(name, default))
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be finite and non-negative")
    return value


def safe_label(value: str) -> str:
    """Make a short label safe to use in a folder name."""
    label = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip()).strip("._-")
    return label or "session"


def create_session_directory(source: str, subject: str) -> Path:
    """Create one timestamped folder below the selected output directory."""
    output = Path(os.environ.get("AURORA_OUTPUT_DIR", DEFAULT_OUTPUT)).expanduser()
    batch_value = os.environ.get("AURORA_SESSION_ID", "").strip()
    timestamp = (
        safe_label(batch_value) if batch_value else datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    )
    session = output / f"{timestamp}_{safe_label(subject)}_{safe_label(source)}"
    session.mkdir(parents=True, exist_ok=False)
    return session


def experiment_metadata() -> dict[str, str]:
    """Return the trial identifiers shared by all recording programs."""
    return {
        "trial_id": os.environ.get("AURORA_TRIAL_ID", "").strip(),
        "test_type": os.environ.get("AURORA_TEST_TYPE", "").strip(),
        "session_id": os.environ.get("AURORA_SESSION_ID", "").strip(),
        "batch_sources": os.environ.get("AURORA_BATCH_SOURCES", "").strip(),
    }


def write_metadata(session: Path, values: dict[str, object]) -> None:
    """Save final session details beside the recorded data."""
    metadata = {
        "format_version": 2,
        "finished_utc": utc_now(),
        **values,
    }
    write_json(session / "meta.json", metadata)
