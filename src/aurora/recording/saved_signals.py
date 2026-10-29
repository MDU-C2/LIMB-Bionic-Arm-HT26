# Majd
"""Load saved signals and their clocks for inspection before processing."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from aurora.json_utils import read_json_object as read_json
from aurora.recording.sensor_csv import iter_recording, load_recording
from aurora.sensors.serial_protocol import AXES


@dataclass
class Trace:
    label: str
    time_s: np.ndarray
    values: np.ndarray
    sample_time_s: np.ndarray | None = None


@dataclass
class Panel:
    title: str
    unit: str
    clock: str
    traces: list[Trace] = field(default_factory=list)


@dataclass
class SavedSignals:
    panels: list[Panel]
    events: list[tuple[float, str]]
    title: str


def video_frame_times(directory: Path):
    """Read receipt times and encoded indices, including fixed-rate video copies."""
    path = directory / "frame_times.jsonl"
    if not path.is_file():
        return None  # Older annotated videos have no per-frame mapping.
    with path.open(encoding="utf-8") as source:
        rows = [json.loads(line) for line in source if line.strip()]
    stamps, indices = [], []
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("Video frame mapping entries must be JSON objects.")
        stamp, index = row.get("host_monotonic_ns"), row.get("video_frame_index")
        if type(stamp) is not int or stamp <= 0 or type(index) is not int or index < 0:
            raise ValueError("Video frame mapping needs positive timestamps and frame indices.")
        if stamps and (stamp <= stamps[-1] or index <= indices[-1]):
            raise ValueError("Video frame mapping must increase in time and frame index.")
        stamps.append(stamp)
        indices.append(index)
    if not stamps:
        raise ValueError("Video frame mapping contains no received frames.")
    return np.asarray(stamps, dtype=np.int64), np.asarray(indices, dtype=np.int64)


def source_files(path: Path) -> tuple[list[Path], dict]:
    if path.is_dir():
        path = path / "trial.json" if (path / "trial.json").is_file() else path
    if path.name == "trial.json":
        metadata = read_json(path)
        files = [path.parent / "ble" / name for name in ("emg.csv", "imu.csv")]
        files.append(path.parent / "camera" / "pose.json")
        return [p for p in files if p.is_file()], metadata
    if path.is_dir():
        return [
            p for p in (path / "emg.csv", path / "imu.csv", path / "recording.csv") if p.is_file()
        ], {}
    if path.suffix.lower() == ".csv" or path.name == "pose.json":
        return [path], {}
    raise ValueError("Choose trial.json, a trial folder, pose.json or a sensor CSV.")


def emg_file(path: Path) -> Path:
    files, _metadata = source_files(path)
    for candidate in files:
        if candidate.suffix.lower() != ".csv":
            continue
        row = next(iter_recording(candidate), {})
        if "emg_raw" in row or (candidate.name == "emg.csv" and "value" in row):
            return candidate
    raise ValueError("Choose a trial, raw emg.csv or USB recording.csv.")


def _trace(
    label: str, rows: list[dict], column: str, time_column: str, scale: float, origin: float
) -> Trace:
    selected = [
        row
        for row in rows
        if row.get(time_column) is not None
        and row.get(column) is not None
        and np.isfinite(row[time_column])
        and np.isfinite(row[column])
    ]
    values = np.asarray([(row[time_column], row[column]) for row in selected], dtype=float).reshape(
        -1, 2
    )
    sample_times = None
    for key in ("device_time", "timestamp_us"):
        if selected and all(row.get(key) is not None and np.isfinite(row[key]) for row in selected):
            sample_times = np.asarray([row[key] for row in selected], dtype=float)
            sample_times = (sample_times - sample_times[0]) / 1e6
            break
    return Trace(label, (values[:, 0] - origin) * scale, values[:, 1], sample_times)


def load_signals(path: Path, sensor: str = "all") -> SavedSignals:
    files, trial = source_files(Path(path))
    csv = {p: load_recording(p) for p in files if p.suffix.lower() == ".csv"}
    all_rows = [row for rows in csv.values() for row in rows]
    synced = bool(all_rows) and all(
        row.get("synced_host_monotonic_ns") is not None
        and np.isfinite(row["synced_host_monotonic_ns"])
        and row["synced_host_monotonic_ns"] > 0
        for row in all_rows
    )
    if synced:
        time_column, scale = "synced_host_monotonic_ns", 1e-9
        origin = trial.get("started_host_monotonic_ns", min(row[time_column] for row in all_rows))
        clock = "Time from trial start [s]" if trial else "Time from first sample [s]"
    else:
        time_column = (
            "device_time"
            if any(row.get("device_time") is not None for row in all_rows)
            else "timestamp_us"
        )
        scale = 1e-6
        times = [row[time_column] for row in all_rows if row.get(time_column) is not None]
        origin = min(times, default=0)
        clock = "ESP32 time from first sample [s]"
        if not times:
            time_column, scale, origin, clock = "timestamp_s", 1.0, 0, "Recorded time [s]"
    if not isinstance(origin, (int, float)) or not np.isfinite(origin):
        raise ValueError("The recording has no finite start timestamp.")
    panels = []
    for source, rows in csv.items():
        if sensor in ("all", "emg"):
            columns = (
                ("Raw EMG", "emg_raw"),
                ("Filtered EMG", "emg_filtered"),
                ("EMG features", "rms"),
            )
            if source.name == "emg.csv":
                columns = (("Raw EMG", "value"),)
            for title, column in columns:
                if not any(row.get(column) is not None for row in rows):
                    continue
                panel = Panel(title, "ADC counts", clock)
                for channel in sorted(
                    {row.get("sensor", 0) for row in rows if row.get(column) is not None}, key=str
                ):
                    selected = [row for row in rows if row.get("sensor", 0) == channel]
                    panel.traces.append(
                        _trace(
                            f"Channel {int(channel or 0)}",
                            selected,
                            column,
                            time_column,
                            scale,
                            origin,
                        )
                    )
                if column == "rms" and any(row.get("mav") is not None for row in rows):
                    panel.traces[0].label = "RMS"
                    panel.traces.append(_trace("MAV", rows, "mav", time_column, scale, origin))
                panels.append(panel)
        units = ("recorded units", "recorded units")
        if (source.parent / "meta.json").is_file():
            units = read_json(source.parent / "meta.json").get("imu_units", units)
        if not isinstance(units, (list, tuple)) or len(units) != 2:
            units = ("recorded units", "recorded units")
        ble = any(row.get("accel_x") is not None for row in rows)
        roles = (0, 1) if ble else ("imu1", "imu2")
        for index, role in enumerate(roles, 1):
            if sensor not in ("all", f"imu{index}"):
                continue
            selected = [row for row in rows if row.get("sensor") == role] if ble else rows
            for kind, axes, unit in (
                ("acceleration", AXES[:3], units[0] if ble else "mg"),
                ("gyroscope", AXES[3:], units[1] if ble else "mdps"),
            ):
                columns = (
                    [f"{'accel' if kind == 'acceleration' else 'gyro'}_{axis[-1]}" for axis in axes]
                    if ble
                    else [f"{role}_{axis}" for axis in axes]
                )
                if not any(row.get(column) is not None for row in selected for column in columns):
                    continue
                aligned = source.name == "imu_aligned.csv"
                if aligned:
                    unit = "g" if kind == "acceleration" else "dps"
                panel = Panel(
                    f"{'Upper arm' if index == 1 else 'Forearm'} {'aligned ' if aligned else ''}{kind}",
                    unit,
                    clock,
                )
                imu_time = (
                    time_column if ble or time_column != "timestamp_us" else f"{role}_timestamp_us"
                )
                panel.traces = [
                    _trace(axis[-1].upper(), selected, column, imu_time, scale, origin)
                    for axis, column in zip(axes, columns, strict=True)
                ]
                panels.append(panel)
    if sensor in ("all", "camera"):
        for source in files:
            if source.name != "pose.json":
                continue
            observations = read_json(source).get("observations", [])
            camera_origin = trial.get("started_host_monotonic_ns")
            camera_clock = (
                "Time from trial start [s]"
                if camera_origin is not None
                else "Camera time from recording start [s]"
            )
            groups = {}
            for observation in observations:
                angles = observation.get("joint_angles_deg", observation.get("angles_deg", {}))
                stamp = observation.get("host_monotonic_ns")
                time_s = (
                    (stamp - camera_origin) / 1e9
                    if camera_origin is not None and stamp is not None
                    else observation.get("time_s")
                )
                if time_s is None:
                    continue
                for name, value in angles.items():
                    if isinstance(value, (int, float)) and np.isfinite(value):
                        groups.setdefault(name, []).append((time_s, value))
            if groups:
                panel = Panel("Saved camera angle estimates", "degrees", camera_clock)
                for name, points in groups.items():
                    values = np.asarray(points)
                    panel.traces.append(Trace(name.replace("_", " "), values[:, 0], values[:, 1]))
                panels.append(panel)
    panels = [panel for panel in panels if any(len(trace.values) for trace in panel.traces)]
    if not panels:
        raise ValueError("No plottable samples with timestamps were found in this selection.")
    events = [
        (event["time_s"], f"Operator {event['marker']}")
        for event in trial.get("operator_events", [])
        if "time_s" in event and "marker" in event
    ]
    subject = trial.get("subject_id", trial.get("subject", trial.get("user_id")))
    title = (
        " / ".join(
            str(value)
            for value in (subject, trial.get("trial_id"), trial.get("pace_condition"))
            if value
        )
        or Path(path).name
    )
    return SavedSignals(panels, events, title)


def with_gaps(trace: Trace) -> tuple[np.ndarray, np.ndarray]:
    """Break the plotted line at missing samples without resampling."""
    intervals = np.diff(trace.sample_time_s if trace.sample_time_s is not None else trace.time_s)
    positive = intervals[intervals > 0]
    if not len(positive):
        return trace.time_s, trace.values
    breaks = (
        np.flatnonzero(
            (intervals <= 0)
            | (intervals > 1.5 * np.median(positive))
            | (np.diff(trace.time_s) <= 0)
        )
        + 1
    )
    return np.insert(trace.time_s, breaks, np.nan), np.insert(trace.values, breaks, np.nan)
