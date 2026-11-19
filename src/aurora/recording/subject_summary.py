"""Summarize saved capture health without changing the recordings."""

from __future__ import annotations

from pathlib import Path

from aurora.json_utils import read_json_object
from aurora.time_utils import utc_now

COUNTERS = (
    "imu_received",
    "imu_lost",
    "emg_received",
    "emg_lost",
    "decode_errors",
    "emg_samples",
    "emg_rail_samples",
    "video_gaps",
    "video_queue_losses",
    "depth_gaps",
    "depth_queue_losses",
)


def _count(value):
    return value if type(value) is int and value >= 0 else None


def loss_percent(received, lost):
    if received is None or lost is None or received + lost == 0:
        return None
    return 100 * lost / (received + lost)


def _capture(path, directory):
    row = {
        "path": path.relative_to(directory).as_posix(),
        "id": path.name.split("_", 1)[0],
        "reference": path.parent.name == "references",
        "pace": None,
        "complete": False,
        "issues": [],
        **dict.fromkeys(COUNTERS),
    }
    try:
        data = read_json_object(path / "trial.json")
        row.update(
            id=data["trial_id"],
            reference=data["test_type"] == "subject_reference",
            pace=data.get("pace_condition"),
            complete=data.get("complete") is True,
        )
        health, capture = data.get("packet_health", {}), data.get("capture_health", {})
        for sensor in ("imu", "emg"):
            packet = health.get(sensor, {})
            row[f"{sensor}_received"] = _count(
                packet.get("received_packets", data.get("packet_counts", {}).get(sensor))
            )
            row[f"{sensor}_lost"] = _count(packet.get("missing_packets"))
            if row[f"{sensor}_received"] == 0:
                row["issues"].append(f"No {sensor.upper()} packets were received")
            if row[f"{sensor}_lost"]:
                percent = loss_percent(row[f"{sensor}_received"], row[f"{sensor}_lost"])
                suffix = f" ({percent:.2f}%)" if percent is not None else ""
                row["issues"].append(
                    f"{sensor.upper()}: {row[f'{sensor}_lost']} missing packets{suffix}"
                )
            if packet.get("sample_time_gaps"):
                row["issues"].append(
                    f"{sensor.upper()}: {packet['sample_time_gaps']} sample-time gaps"
                )
            if packet.get("gyro_saturation_samples"):
                row["issues"].append(
                    f"IMU: {packet['gyro_saturation_samples']} gyro saturation samples"
                )
            for key, label in (
                ("duplicate_packets", "duplicate packets"),
                ("reordered_packets", "reordered packets"),
            ):
                if packet.get(key):
                    row["issues"].append(f"{sensor.upper()}: {packet[key]} {label}")
        errors = [_count(health.get(sensor, {}).get("decode_errors")) for sensor in ("imu", "emg")]
        row["decode_errors"] = sum(errors) if None not in errors else None
        if row["decode_errors"]:
            row["issues"].append(f"{row['decode_errors']} packet decode errors")
        if any(row[key] is None for key in ("imu_lost", "emg_lost", "decode_errors")):
            row["issues"].append("Some packet-health information was not reported")
        row["emg_samples"] = _count(health.get("emg", {}).get("sample_count"))
        row["emg_rail_samples"] = _count(health.get("emg", {}).get("adc_rail_samples"))
        if row["emg_rail_samples"]:
            row["issues"].append(
                f"EMG: {row['emg_rail_samples']} samples at ADC limits; inspect contact and signal"
            )
        for key, source, label in (
            ("video_gaps", "camera_sequence_gaps", "Video sequence gaps"),
            (
                "video_queue_losses",
                "raw_frame_queue_overruns",
                "Video frames lost from the host queue",
            ),
            ("depth_gaps", "depth_sequence_gaps", "Depth sequence gaps"),
            (
                "depth_queue_losses",
                "depth_frame_queue_overruns",
                "Depth frames lost from the host queue",
            ),
        ):
            row[key] = _count(capture.get(source))
            if row[key]:
                row["issues"].append(f"{label}: {row[key]}")
        for key, label in (
            ("camera_error", "Camera error"),
            ("tracking_error", "Tracking error; raw capture may still be available"),
        ):
            if capture.get(key):
                row["issues"].append(f"{label}: {capture[key]}")
        if capture.get("tracking_queue_overruns"):
            row["issues"].append(
                f"Tracking results lost from the host queue: {capture['tracking_queue_overruns']}"
            )
        if data.get("camera_video_frame_count") == 0:
            row["issues"].append("No video frames were saved")
        if data.get("stereo_depth_enabled") and data.get("camera_depth_frame_count") == 0:
            row["issues"].append("No depth frames were saved")
        for key, label in (
            ("write_error", "File-write error"),
            ("shutdown_error", "Capture cleanup error"),
            ("alignment_export_error", "Aligned export error"),
        ):
            if data.get(key):
                row["issues"].append(f"{label}: {data[key]}")
        for key, label in (
            ("stream_interrupted", "A sensor stream was interrupted"),
            ("depth_interrupted", "Depth was interrupted"),
            ("subject_reference_invalidated", "BLE reconnected; reference invalidated"),
            ("raw_packet_queue_overruns", "Raw BLE writer queue overflowed"),
            ("device_clock_resets", "The device clock reset"),
        ):
            if data.get(key):
                row["issues"].append(label)
        checks = data.get("reference_checks") or data.get("calibration_status", {})
        if data.get("sensor_to_arm_alignment") != "fitted":
            row["issues"].append("Arm alignment unavailable; raw signals kept")
            if row["reference"] and checks.get("alignment_message"):
                row["issues"].append("Alignment check: " + checks["alignment_message"])
        if checks and not all(checks.get(key) for key in ("emg_rest_ready", "emg_grip_ready")):
            row["issues"].append("EMG amplitude reference needs review")
        if not row["complete"]:
            row["issues"].append(
                "Capture incomplete: " + str(data.get("stop_reason", "reason not reported"))
            )
    except (OSError, ValueError, KeyError, TypeError, AttributeError) as error:
        row["complete"] = False
        row["issues"].append(f"Could not read capture metadata: {error}")
    return row


def summarize_subject(directory: Path, subject_id: str, capture_paths, *, session_errors=()):
    rows = [_capture(Path(path), directory) for path in dict.fromkeys(capture_paths)]

    def totals(reference):
        selected = [row for row in rows if row["reference"] == reference]
        return {
            key: sum(row[key] for row in selected)
            if all(row[key] is not None for row in selected)
            else None
            for key in COUNTERS
        }

    trials = [row for row in rows if not row["reference"]]
    return {
        "format_version": 1,
        "subject_id": subject_id,
        "created_utc": utc_now(),
        "scope": "current_recording_session",
        "captures": rows,
        "trial_count": len(trials),
        "complete_trials": sum(row["complete"] for row in trials),
        "trials_with_warnings": sum(bool(row["issues"]) for row in trials),
        "reference_count": len(rows) - len(trials),
        "trial_totals": totals(False),
        "reference_totals": totals(True),
        "session_errors": list(session_errors),
    }
