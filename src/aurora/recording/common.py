# Majd
"""Shared paths and metadata for sensor recording programs."""

from __future__ import annotations

import _thread
import json
import math
import os
import re
import signal
import sys
import threading
from datetime import datetime
from pathlib import Path

from aurora.json_utils import read_json_object, write_json
from aurora.paths import REPOSITORY_ROOT
from aurora.time_utils import utc_now

DEFAULT_OUTPUT = REPOSITORY_ROOT / "outputs" / "recordings"


def announce_result(path, kind="recording"):
    """Publish a saved artifact to the supervising app, separately from log wording."""
    print(
        "[AURORA] "
        + json.dumps({"event": "saved", "path": str(Path(path).resolve()), "kind": kind}),
        flush=True,
    )


STOP_COMMAND = '{"command":"stop"}\n'


def enable_ctrl_break_as_interrupt(*, stdin_stop=False, on_stop=None) -> None:
    """Reach CLI cleanup blocks, or let a GUI schedule cleanup on its event loop."""

    def interrupt(_number: int, _frame: object) -> None:
        if on_stop is None:
            raise KeyboardInterrupt
        on_stop()

    for name in ("SIGBREAK", "SIGTERM") + (("SIGINT",) if on_stop else ()):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), interrupt)
    if stdin_stop and os.environ.get("AURORA_GUI_CONTROL") == "1":
        threading.Thread(
            target=_read_stop_command, args=(on_stop or _thread.interrupt_main,), daemon=True
        ).start()


def _read_stop_command(on_stop):
    """Receive a stop command, including in console-free Windows children."""
    pending = b""
    try:
        descriptor = sys.stdin.fileno()
        # This worker owns stdin. Nested helpers must use DEVNULL, so they cannot
        # consume commands or share a pending Windows pipe read.
        while block := os.read(descriptor, 4096):
            pending += block
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                try:
                    command = json.loads(line)
                except (ValueError, UnicodeError):
                    continue
                if isinstance(command, dict) and command.get("command") == "stop":
                    on_stop()
                    return
            if len(pending) > 4096:
                return
    except (OSError, ValueError, AttributeError):
        return


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


def verify_subject_files(paths, subject: str) -> None:
    """Reject an ID collision using saved identity, including case-insensitive paths."""
    for path in paths:
        try:
            document = read_json_object(path)
            saved = document.get("subject_id", document.get("user_id"))
        except (OSError, ValueError) as error:
            raise ValueError(f"Cannot verify the subject in {path.name}: {error}") from error
        if saved is None or str(saved) != subject:
            raise ValueError(
                f"This folder contains recordings for subject {saved!r}. "
                "Use that exact subject ID or choose another output folder."
            )


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
