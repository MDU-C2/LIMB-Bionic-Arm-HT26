# Jonas Gerne, Majd
"""Record USB diagnostics and export each timestamped sensor measurement."""

from __future__ import annotations

import argparse
import csv
import json

from aurora.recording.record_serial_sensors import parse_args as serial_args
from aurora.recording.record_serial_sensors import record
from aurora.sensors.serial_protocol import AXES, COLUMNS, parse_sensor_line


def parse_args() -> argparse.Namespace:
    return serial_args(preview=False, description=__doc__)


def export_samples(session):
    """Stream derived rows after capture closes; never hold a trial in memory."""
    counts = dict.fromkeys(("emg", "imu1", "imu2"), 0)
    quality = dict.fromkeys(
        ("duplicates", "reordered_samples", "device_clock_resets", "invalid_sensor_lines"), 0
    )
    previous, origin, epoch = {}, None, 0
    with (
        (session / "serial.jsonl").open(encoding="utf-8") as source,
        (session / "recording.csv").open("w", newline="", encoding="utf-8") as output,
        (session / "raw_serial.txt").open("w", encoding="utf-8") as text_output,
    ):
        writer = csv.DictWriter(
            output, fieldnames=(*COLUMNS, "host_time", "device_clock_epoch"), delimiter=";"
        )
        writer.writeheader()
        for line in source:
            record = json.loads(line)
            text_output.write(record["text"] + "\n")
            parsed = parse_sensor_line(record["text"])
            if parsed is None:
                if record["text"].split(",", 1)[0].upper() in ("EMG", "IMU1", "IMU2"):
                    quality["invalid_sensor_lines"] += 1
                continue
            sensor, stamp, values = parsed
            last = previous.get(sensor)
            if last is not None:
                if stamp < last - 500_000:
                    epoch += 1
                    previous.clear()
                    origin = None
                    quality["device_clock_resets"] += 1
                elif stamp == last:
                    quality["duplicates"] += 1
                elif stamp < last:
                    quality["reordered_samples"] += 1
            previous[sensor] = stamp
            if origin is None:
                origin = stamp
            row = dict(
                timestamp_us=stamp,
                timestamp_s=(stamp - origin) / 1e6,
                host_monotonic_ns=record["host_monotonic_ns"],
                host_time=record["host_time"],
                device_clock_epoch=epoch,
            )
            if sensor == "emg":
                row["emg_raw"] = values[0]
            else:
                row[f"{sensor}_timestamp_us"] = stamp
                row.update(
                    {f"{sensor}_{axis}": value for axis, value in zip(AXES, values, strict=True)}
                )
            writer.writerow(row)
            counts[sensor] += 1
    return dict(
        samples=counts,
        rows=sum(counts.values()),
        sample_quality=quality,
        recording_mode="diagnostic_raw_serial",
        csv_layout="one_row_per_received_measurement",
        imu_units=["mg", "mdps"],
        sample_data_complete=bool(sum(counts.values())) and not any(quality.values()),
        clock_alignment="device sample times and measured host line receipt; no clock fit",
        transport_loss="not observable from unsequenced USB lines",
    )


def main() -> int:
    return record(parse_args(), source="high_rate_serial", sample_exporter=export_samples)


if __name__ == "__main__":
    raise SystemExit(main())
