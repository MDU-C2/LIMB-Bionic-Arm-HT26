"""Shared recording time reference for AURORA offline preprocessing.

This module DOES NOT rewrite EMG, IMU, or camera timestamps.
Relative time is a presentation coordinate, not proof of exact sensor sync.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path


START_KEY = "trial_started_host_monotonic_ns"


def relative_seconds(timestamp_ns: int, trial_start_ns: int) -> float:
    """Convert an existing host-monotonic timestamp to trial-relative seconds."""
    return (int(timestamp_ns) - int(trial_start_ns)) / 1e9


def get_trial_start(trial: Path) -> int:
    """Read common trial start from metadata, falling back to first available data.

    Metadata from BLE/camera should agree. A disagreement is treated as an
    error rather than hiding a potentially broken recording reference.
    The fallback is only a plot origin, NOT verified BLE/camera synchronization.
    """
    trial = Path(trial)
    starts = []
    for path in (trial / "ble" / "meta.json", trial / "camera" / "meta.json", trial / "meta.json"):
        if not path.is_file():
            continue
        meta = json.loads(path.read_text(encoding="utf-8-sig"))
        start = meta.get(START_KEY)
        if start is not None:
            starts.append((path, int(start)))

    if starts:
        different = {value for _, value in starts}
        if len(different) != 1:
            raise ValueError("Conflicting trial start timestamps in metadata: " + str(starts))
        return starts[0][1]

    # Backward compatibility with recordings that predate common-start metadata.
    first_times = []
    for file_name in ("emg.csv", "imu.csv"):
        path = trial / "ble" / file_name
        if not path.is_file():
            continue
        with path.open(newline="", encoding="utf-8-sig") as stream:
            for row in csv.DictReader(stream):
                value = row.get("synced_host_monotonic_ns")
                if value:
                    first_times.append(int(value))
                    break

    pose_path = trial / "camera" / "pose.json"
    if pose_path.is_file():
        pose = json.loads(pose_path.read_text(encoding="utf-8-sig"))
        if isinstance(pose, dict):
            for key in ("data", "observations"):
                for frame in pose.get(key, []):
                    value = frame.get("host_monotonic_ns")
                    if value is not None:
                        first_times.append(int(value))
                        break

    if not first_times:
        raise ValueError(f"No common trial start or timestamps found in {trial}")

    print("WARNING: missing common trial start metadata; using earliest available timestamp as plot origin")
    return min(first_times)
