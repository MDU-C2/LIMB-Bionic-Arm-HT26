"""Record newline-delimited sensor data from a serial device."""

from __future__ import annotations

import argparse
import base64
import json
import os
import time

from common import create_session_directory, env_float, utc_now, write_metadata


def parse_args() -> argparse.Namespace:
    """Read GUI defaults and optional command-line overrides."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=os.environ.get("AURORA_SERIAL_PORT", ""))
    parser.add_argument(
        "--baud",
        type=int,
        default=int(os.environ.get("AURORA_SERIAL_BAUD", "115200")),
    )
    parser.add_argument("--subject", default=os.environ.get("AURORA_SUBJECT", "session"))
    parser.add_argument(
        "--duration",
        type=float,
        default=env_float("AURORA_DURATION_SECONDS", 0.0),
        help="Seconds to record; 0 records until stopped.",
    )
    return parser.parse_args()


def available_ports() -> list[str]:
    """Return serial ports reported by pyserial."""
    from serial.tools import list_ports

    return [port.device for port in list_ports.comports()]


def main() -> int:
    """Read serial lines and keep both decoded JSON and original bytes."""
    args = parse_args()
    try:
        import serial
    except ImportError:
        print("pyserial is missing. Recreate the simulation environment.")
        return 2

    if not args.port:
        ports = available_ports()
        print("Choose a serial port in the Recording tab.")
        print("Available ports:", ", ".join(ports) if ports else "none")
        return 2
    if args.baud <= 0 or args.duration < 0:
        print("Baud must be positive and duration cannot be negative.")
        return 2

    session = create_session_directory("serial", args.subject)
    output_path = session / "serial.jsonl"
    started = time.monotonic()
    lines_written = 0
    print(f"Recording {args.port} at {args.baud} baud")
    print(f"Saving to {session}")

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as device, output_path.open(
            "w", encoding="utf-8", buffering=1
        ) as output:
            while args.duration == 0 or time.monotonic() - started < args.duration:
                raw = device.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace").strip()
                record: dict[str, object] = {
                    "host_time": utc_now(),
                    "text": text,
                    "raw_base64": base64.b64encode(raw).decode("ascii"),
                }
                try:
                    record["data"] = json.loads(text)
                except json.JSONDecodeError:
                    pass
                output.write(json.dumps(record, separators=(",", ":")) + "\n")
                lines_written += 1
                if lines_written % 100 == 0:
                    print(f"Recorded {lines_written} serial messages")
    except KeyboardInterrupt:
        print("Stopping serial recording...")
    except serial.SerialException as error:
        print(f"Serial recording failed: {error}")
        return 1
    finally:
        write_metadata(
            session,
            {
                "source": "serial",
                "subject": args.subject,
                "port": args.port,
                "baud": args.baud,
                "duration_seconds": round(time.monotonic() - started, 3),
                "messages": lines_written,
            },
        )

    print(f"Saved {lines_written} messages to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
