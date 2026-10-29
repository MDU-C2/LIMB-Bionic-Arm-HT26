# Majd
"""Fit a saved subject reference and export aligned copies of IMU samples."""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from aurora.file_utils import file_digest as digest
from aurora.json_utils import read_json_object
from aurora.sensors.arm_alignment import (
    METHOD,
    ROLES,
    AlignmentError,
    ImuSeries,
    fit_alignment,
    rotation_matrix,
)


def reference_windows(manifest):
    """Use actual cue transitions, not nominal timer durations or GUI polling."""
    steps = manifest["protocol"]["reference_steps"]
    starts = {}
    for event in manifest["cue_events"]:
        step = event.get("reference_step")
        if event.get("phase") == "reference" and type(step) is int and 0 <= step < len(steps):
            starts.setdefault(step, event["host_monotonic_ns"])
    if len(starts) != len(steps):
        raise AlignmentError("Preparation was interrupted or missed a reference step")
    windows = {}
    for index, step in enumerate(steps):
        # Reading, repositioning and countdowns must stay outside the fit windows.
        end = next(
            (
                event["host_monotonic_ns"]
                for event in manifest["cue_events"]
                if event["host_monotonic_ns"] > starts[index]
                and (event.get("phase") != "reference" or event.get("reference_step") != index)
            ),
            manifest["recording_ended_host_monotonic_ns"],
        )
        # Allow the operator/subject to change pose. Motion sign uses the first
        # movement after this short allowance; still references get longer settling.
        settle = 1.0 if step["cue"] in ("STILL", "RETURN") else 0.25
        windows[step["cue"]] = (starts[index] + round(settle * 1e9), end - 200_000_000)
    return windows


def fit_subject_reference(directory: Path, manifest: dict):
    """Return a passed or failed profile; raw files are read-only inputs."""
    sources = (directory / "ble/imu.csv", directory / "camera/pose.json")
    result = {
        "format_version": 1,
        "method": METHOD,
        "status": "failed",
        "subject_id": manifest["subject_id"],
        "side": manifest["side"],
        "connection_generation": manifest["connection_generation"],
        "mounting_session_id": manifest.get("mounting_session_id"),
        "accuracy_validated": False,
    }
    try:
        result["source_sha256"] = {p.relative_to(directory).as_posix(): digest(p) for p in sources}
        result["source_code_sha256"] = {
            "subject_calibration.py": digest(Path(__file__)),
            "arm_alignment.py": digest(Path(__file__).parents[1] / "sensors/arm_alignment.py"),
        }
        if not manifest.get("complete"):
            raise AlignmentError("Preparation capture is incomplete; check streams and repeat")
        if not manifest.get("stereo_depth_enabled") or manifest.get("camera_intrinsics") is None:
            raise AlignmentError("Stereo depth and camera intrinsics are required for alignment")
        units = tuple(manifest.get("imu_units", ()))
        if units not in (("g", "dps"), ("mg", "mdps")):
            raise AlignmentError("IMU units are unavailable; use the current sensor firmware")
        scale = 1.0 if units == ("g", "dps") else 0.001
        windows = reference_windows(manifest)
        stages = {name: {role: [] for role in ROLES} for name in windows}
        with sources[0].open(encoding="utf-8", newline="") as source:
            rows = list(csv.DictReader(source))
        synced = bool(rows) and all(row.get("synced_host_monotonic_ns") for row in rows)
        clock = "synced_host_monotonic_ns" if synced else "host_monotonic_ns"
        for row in rows:
            role = ROLES[int(row["sensor"])]
            stamp = int(row[clock])
            for name, (start, end) in windows.items():
                if start <= stamp < end:
                    stages[name][role].append(
                        [
                            int(row["device_time"]) / 1e6,
                            *[
                                float(row[f"{kind}_{axis}"]) * scale
                                for kind in ("accel", "gyro")
                                for axis in "xyz"
                            ],
                        ]
                    )
                    break
        for roles in stages.values():
            for role, samples in roles.items():
                array = np.asarray(samples, dtype=float).reshape(-1, 7)
                roles[role] = ImuSeries(array[:, 0], array[:, 1:4], array[:, 4:7])
        camera = {name: [] for name in windows}
        for frame in read_json_object(sources[1]).get("data", []):
            stamp = frame.get("host_monotonic_ns")
            if type(stamp) is not int:
                continue
            for name, (start, end) in windows.items():
                if start <= stamp < end and "keypoints_camera_m" in frame:
                    camera[name].append(frame["keypoints_camera_m"])
                    break
        result.update(fit_alignment(stages, camera, side=manifest["side"]))
        result["cue_assignment_clock"] = clock
        result["reference_windows_host_ns"] = {name: list(value) for name, value in windows.items()}
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        result.update(status="failed", message=str(error))
    return result


def export_aligned_imu(source: Path, destination: Path, profile: dict, *, units):
    """Rotate a separate copy; keep raw values, timestamps and packet fields intact.

    This exports segment-frame vectors, not joint angles. Gyroscope bias from
    preparation is subtracted only in this derived file.
    """
    if profile.get("status") != "passed" or profile.get("method") != METHOD:
        raise ValueError("A passed sensor-to-arm calibration is required")
    matrices = {role: rotation_matrix(profile["sensor_to_segment"][role]) for role in ROLES}
    biases = {role: np.asarray(profile["gyro_bias_dps"][role], dtype=float) for role in ROLES}
    if any(bias.shape != (3,) or not np.isfinite(bias).all() for bias in biases.values()):
        raise ValueError("Invalid saved gyroscope bias")
    units = tuple(units)
    if units not in (("g", "dps"), ("mg", "mdps")):
        raise ValueError("Aligned export requires known IMU units")
    scale = 1.0 if units == ("g", "dps") else 0.001
    rows_written = 0
    with (
        source.open(encoding="utf-8", newline="") as original,
        destination.open("x", encoding="utf-8", newline="") as output,
    ):
        reader = csv.DictReader(original)
        writer = csv.DictWriter(output, fieldnames=reader.fieldnames)
        writer.writeheader()
        for row in reader:
            role = ROLES[int(row["sensor"])]
            for kind in ("accel", "gyro"):
                vector = np.asarray([float(row[f"{kind}_{axis}"]) * scale for axis in "xyz"])
                if not np.isfinite(vector).all():
                    raise ValueError("IMU export contains invalid vectors")
                if kind == "gyro":
                    vector -= biases[role]
                aligned = matrices[role] @ vector
                for axis, value in zip("xyz", aligned, strict=True):
                    row[f"{kind}_{axis}"] = format(value, ".12g")
            writer.writerow(row)
            rows_written += 1
    return {
        "file": destination.name,
        "rows": rows_written,
        "units": ["g", "dps"],
        "source_sha256": digest(source),
        "frame": profile["segment_frame"],
        "gyro_bias_subtracted": True,
    }
