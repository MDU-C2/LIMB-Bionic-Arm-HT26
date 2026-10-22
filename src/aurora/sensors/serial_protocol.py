# Jonas Gerne, Majd
"""Parse and align timestamped EMG/dual-IMU USB samples."""

from __future__ import annotations

import csv
import math
from bisect import bisect_left
from pathlib import Path
from statistics import median

AXES = ("ax", "ay", "az", "gx", "gy", "gz")
COLUMNS = (
    "timestamp_s",
    "timestamp_us",
    "host_monotonic_ns",
    "emg_raw",
    "imu1_timestamp_us",
    *(f"imu1_{axis}" for axis in AXES),
    "imu2_timestamp_us",
    *(f"imu2_{axis}" for axis in AXES),
)


def parse_sensor_line(text: str) -> tuple[str, int, tuple[int | float, ...]] | None:
    """Accept the original two-column EMG and tagged EMG/IMU lines."""
    parts = [part.strip() for part in text.split(",")]
    try:
        if len(parts) == 2:
            timestamp, value = (int(part) for part in parts)
            sensor = "emg"
            values = (value,)
        elif len(parts) == 3 and parts[0].upper() == "EMG":
            sensor = "emg"
            timestamp, value = int(parts[1]), int(parts[2])
            values = (value,)
        elif len(parts) == 8 and parts[0].upper() in {"IMU1", "IMU2"}:
            sensor = parts[0].lower()
            timestamp = int(parts[1])
            values = tuple(float(part) for part in parts[2:])
        else:
            return None
    except ValueError:
        return None
    if sensor == "emg" and not 0 <= values[0] <= 4095:
        return None
    if sensor != "emg" and not all(math.isfinite(value) for value in values):
        return None
    return sensor, timestamp, values


def merge_samples(
    samples: dict[str, list[tuple[int, tuple[int | float, ...]]]],
) -> list[dict[str, int | float]]:
    """Place each IMU sample near an EMG row without discarding unmatched data."""
    emg = sorted(samples.get("emg", []))
    rows: list[dict[str, int | float]] = [
        {"timestamp_us": timestamp, "emg_raw": values[0]} for timestamp, values in emg
    ]
    times = [timestamp for timestamp, _values in emg]
    intervals = [b - a for a, b in zip(times, times[1:], strict=False) if b > a]
    tolerance = max(1, int(median(intervals) / 2)) if intervals else 5000

    for sensor in ("imu1", "imu2"):
        field = f"{sensor}_timestamp_us"
        for timestamp, values in sorted(samples.get(sensor, [])):
            index = bisect_left(times, timestamp)
            candidates = (index - 1, index)
            nearest = min(
                (
                    candidate
                    for candidate in candidates
                    if 0 <= candidate < len(rows) and field not in rows[candidate]
                ),
                key=lambda candidate: abs(times[candidate] - timestamp),
                default=None,
            )
            if nearest is None or abs(times[nearest] - timestamp) > tolerance:
                row: dict[str, int | float] = {"timestamp_us": timestamp}
                times.insert(index, timestamp)
                rows.insert(index, row)
            else:
                row = rows[nearest]
            row[field] = timestamp
            row.update(
                {f"{sensor}_{axis}": value for axis, value in zip(AXES, values, strict=False)}
            )

    if rows:
        start = times[0]
        for row in rows:
            row["timestamp_s"] = (row["timestamp_us"] - start) / 1_000_000
    return rows


def save_csv(path: Path, rows: list[dict[str, int | float]]) -> None:
    """Use the source project's semicolon CSV layout for compatibility."""
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=COLUMNS, delimiter=";")
        writer.writeheader()
        writer.writerows(rows)
