# Jonas Gerne, Majd
"""Record high-rate USB EMG and dual IMUs into an aligned CSV."""

from __future__ import annotations

import argparse
import math
import os
import signal
import time

from aurora.recording.common import (
    create_session_directory,
    enable_ctrl_break_as_interrupt,
    env_float,
    experiment_metadata,
    write_metadata,
)
from aurora.sensors.serial_protocol import merge_samples, parse_sensor_line, save_csv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=os.environ.get("AURORA_SERIAL_PORT", ""))
    parser.add_argument(
        "--baud", type=int, default=int(os.environ.get("AURORA_SERIAL_BAUD", "115200"))
    )
    parser.add_argument("--duration", type=float, default=env_float("AURORA_DURATION_SECONDS", 0))
    parser.add_argument(
        "--subject-id",
        "--subject",
        dest="subject",
        default=os.environ.get("AURORA_SUBJECT", "session"),
    )
    return parser.parse_args()


def main() -> int:
    enable_ctrl_break_as_interrupt()
    args = parse_args()
    if not args.port or args.baud <= 0 or not math.isfinite(args.duration) or args.duration < 0:
        print("Provide a serial port, positive baud rate, and finite non-negative duration.")
        return 2
    try:
        import serial
    except ImportError:
        print("Install pyserial to record the USB stream.")
        return 2

    samples: dict[str, list[tuple[int, tuple[int | float, ...]]]] = {
        "emg": [],
        "imu1": [],
        "imu2": [],
    }
    device_to_host_offset_ns: int | None = None
    stopping = False

    def request_stop(_number: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True

    for name in ("SIGINT", "SIGTERM"):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), request_stop)
    started = time.monotonic()
    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as device:
            session = create_session_directory("high_rate_serial", args.subject)
            print(f"Recording {args.port} to {session}; stop with Ctrl+C")
            with (session / "raw_serial.txt").open("w", encoding="utf-8") as raw_output:
                try:
                    while not stopping and (
                        args.duration == 0 or time.monotonic() - started < args.duration
                    ):
                        raw = device.readline()
                        if not raw:
                            continue
                        line = raw.decode("utf-8", errors="replace").strip()
                        raw_output.write(line + "\n")
                        parsed = parse_sensor_line(line)
                        if parsed is not None:
                            sensor, timestamp, values = parsed
                            samples[sensor].append((timestamp, values))
                            observed_offset_ns = time.monotonic_ns() - timestamp * 1000
                            if (
                                device_to_host_offset_ns is None
                                or observed_offset_ns < device_to_host_offset_ns
                            ):
                                device_to_host_offset_ns = observed_offset_ns
                except KeyboardInterrupt:
                    print("Stopping recording...")
    except (serial.SerialException, OSError) as error:
        print(f"Serial recording failed: {error}")
        return 1

    rows = merge_samples(samples)
    if device_to_host_offset_ns is not None:
        for row in rows:
            row["host_monotonic_ns"] = int(row["timestamp_us"]) * 1000 + device_to_host_offset_ns
    save_csv(session / "recording.csv", rows)
    write_metadata(
        session,
        {
            "source": "high_rate_serial",
            "subject_id": args.subject,
            "port": args.port,
            "baud": args.baud,
            "duration_seconds": round(time.monotonic() - started, 3),
            "clock_domain": "host_monotonic_ns",
            "device_to_host_monotonic_offset_ns": device_to_host_offset_ns,
            "clock_alignment": "minimum USB receive delay; approximate",
            "samples": {sensor: len(values) for sensor, values in samples.items()},
            "rows": len(rows),
            **experiment_metadata(),
        },
    )
    print(f"Saved {len(rows)} aligned rows to {session / 'recording.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
