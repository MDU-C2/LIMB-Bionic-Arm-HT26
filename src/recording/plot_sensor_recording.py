# Jonas Gerne, Majd
"""Plot sensor recordings and processed EMG CSV files."""

from __future__ import annotations

import argparse
import csv
from collections.abc import Iterator
from pathlib import Path

from common import DEFAULT_OUTPUT
from high_rate_serial import AXES


def iter_recording(path: Path) -> Iterator[dict[str, float | None]]:
    """Read numeric CSV rows without keeping the whole recording in memory."""

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
    """Load CSV rows for plotting and keep comma/semicolon compatibility."""
    return list(iter_recording(path))


def latest_recording(root: Path = DEFAULT_OUTPUT) -> Path | None:
    files = root.rglob("recording.csv") if root.is_dir() else ()
    return max(files, key=lambda path: path.stat().st_mtime, default=None)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--file", type=Path, help="CSV file; defaults to the newest high-rate recording"
    )
    parser.add_argument("--sensor", choices=("all", "emg", "imu1", "imu2"), default="all")
    parser.add_argument("--save", type=Path, help="Save a PNG instead of opening a window")
    args = parser.parse_args()
    path = args.file or latest_recording()
    if path is None or not path.is_file():
        print("No high-rate recording.csv was found. Pass --file PATH.")
        return 2
    rows = load_recording(path)
    if not rows:
        print(f"No samples in {path}")
        return 1

    import matplotlib.pyplot as plt

    selections = ("emg", "imu1", "imu2") if args.sensor == "all" else (args.sensor,)
    plots = []
    for sensor in selections:
        if sensor == "emg":
            for title, columns in (
                ("EMG raw [ADC counts]", ("emg_raw",)),
                ("EMG filtered [ADC counts]", ("emg_filtered",)),
                ("EMG window features [ADC counts]", ("rms", "mav")),
            ):
                if any(row.get(column) is not None for row in rows for column in columns):
                    plots.append((title, "timestamp_us", columns))
        elif sensor != "emg" and any(
            row.get(f"{sensor}_{axis}") is not None for axis in AXES for row in rows
        ):
            plots.extend(
                (
                    (
                        f"{sensor.upper()} acceleration [mg]",
                        f"{sensor}_timestamp_us",
                        tuple(f"{sensor}_{axis}" for axis in AXES[:3]),
                    ),
                    (
                        f"{sensor.upper()} gyroscope [mdps]",
                        f"{sensor}_timestamp_us",
                        tuple(f"{sensor}_{axis}" for axis in AXES[3:]),
                    ),
                )
            )
    if not plots:
        print(f"No {args.sensor} data in {path}")
        return 1

    origin = next(
        (
            row["timestamp_us"] - row["timestamp_s"] * 1_000_000
            for row in rows
            if row.get("timestamp_us") is not None and row.get("timestamp_s") is not None
        ),
        min(
            (row["timestamp_us"] for row in rows if row.get("timestamp_us") is not None), default=0
        ),
    )
    figure, axes = plt.subplots(
        len(plots), 1, figsize=(13, 3 * len(plots)), sharex=True, squeeze=False
    )
    for axis, (title, time_column, columns) in zip(axes.flat, plots, strict=False):
        for column in columns:
            points = [
                (
                    (row[time_column] - origin) / 1_000_000
                    if row.get(time_column) is not None
                    else row["timestamp_s"],
                    row[column],
                )
                for row in rows
                if row.get(column) is not None
                and (row.get(time_column) is not None or row.get("timestamp_s") is not None)
            ]
            if points:
                axis.plot(
                    *zip(*points, strict=False), label=column.split("_")[-1].upper(), linewidth=0.9
                )
        axis.set_title(title)
        axis.grid(True)
        if axis.lines:
            axis.legend()
    axes[-1, 0].set_xlabel("Time [s]")
    figure.suptitle(path.name)
    figure.tight_layout()
    if args.save:
        figure.savefig(args.save, dpi=150)
        print(f"Saved {args.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
