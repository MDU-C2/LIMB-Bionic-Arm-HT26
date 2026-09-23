"""Record LIMB EMG, IMU, and piezo notifications over BLE."""

from __future__ import annotations

import argparse
import asyncio
import base64
import csv
import json
import os
from pathlib import Path
import struct
import time
from typing import Callable

from common import create_session_directory, env_float, experiment_metadata, utc_now, write_metadata
from ble_dataset import LabeledBleCapture


SERVICE_UUID = "23011525-1212-efde-1523-785feabcd122"
SENSOR_UUIDS = {
    "emg": "24011525-1212-efde-1523-785feabcd122",
    "imu": "25011525-1212-efde-1523-785feabcd122",
    "piezo": "26011525-1212-efde-1523-785feabcd122",
}
IMU_COLUMNS = (
    "accel_x",
    "accel_y",
    "accel_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "temperature",
    "pitch",
    "roll",
)


def parse_args() -> argparse.Namespace:
    """Read GUI defaults and optional command-line overrides."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=os.environ.get("AURORA_BLE_DEVICE", "LIMBServer"))
    parser.add_argument("--address", default="", help="Connect to a known BLE address directly.")
    parser.add_argument(
        "--preview", action="store_true",
        help="Show live EMG and IMU feedback without saving a session.",
    )
    parser.add_argument(
        "--preview-sensor",
        choices=("all", "emg", "imu", "piezo"),
        default="all",
        help="Choose which BLE sensor window to open first.",
    )
    parser.add_argument("--scan-timeout", type=float, default=12.0)
    parser.add_argument("--subject", default=os.environ.get("AURORA_SUBJECT", "session"))
    parser.add_argument(
        "--duration",
        type=float,
        default=env_float("AURORA_DURATION_SECONDS", 0.0),
        help="Seconds to record; 0 records until stopped (labeled capture auto-stops).",
    )
    parser.add_argument(
        "--dataset-label",
        default="",
        help="Capture 80 labeled 100 ms windows (20 rest, 40 movement, 20 rest).",
    )
    return parser.parse_args()


class BleRecorder:
    """Decode known LIMB packet revisions while preserving every raw packet."""

    def __init__(
        self,
        session: Path | None,
        sample_sink: Callable[[str, int, list[list[object]]], None] | None = None,
    ) -> None:
        self.session = session
        self.sample_sink = sample_sink
        self.imu_units = ("device units", "device units")
        self.counts = {name: 0 for name in SENSOR_UUIDS}
        self.raw_file = (
            (session / "packets.jsonl").open("w", encoding="utf-8", buffering=1)
            if session is not None else None
        )
        self.files = {}
        if session is not None:
            self.files = {
                "emg": (session / "emg.csv").open("w", newline="", encoding="utf-8"),
                "imu": (session / "imu.csv").open("w", newline="", encoding="utf-8"),
                "piezo": (session / "piezo.csv").open("w", newline="", encoding="utf-8"),
            }
        self.writers = {name: csv.writer(file) for name, file in self.files.items()}
        if self.writers:
            self.writers["emg"].writerow(
                (
                    "host_time", "host_monotonic_ns", "sequence", "device_time",
                    "sensor", "sample", "value",
                )
            )
            self.writers["piezo"].writerow(
                (
                    "host_time", "host_monotonic_ns", "sequence", "device_time",
                    "sensor", "sample", "value",
                )
            )
            self.writers["imu"].writerow(
                (
                    "host_time", "host_monotonic_ns", "sequence", "device_time",
                    "sensor", "sample", *IMU_COLUMNS,
                )
            )

    def close(self) -> None:
        """Flush and close all session files."""
        if self.raw_file is not None:
            self.raw_file.close()
        for file in self.files.values():
            file.close()

    def handle(self, sensor: str, data: bytearray) -> None:
        """Save one notification and decode it when its packet layout is known."""
        host_time = utc_now()
        host_monotonic_ns = time.monotonic_ns()
        payload = bytes(data)
        self.counts[sensor] += 1
        if self.raw_file is not None:
            self.raw_file.write(
                json.dumps(
                    {
                        "host_time": host_time,
                        "host_monotonic_ns": host_monotonic_ns,
                        "sensor": sensor,
                        "uuid": SENSOR_UUIDS[sensor],
                        "size": len(payload),
                        "data_base64": base64.b64encode(payload).decode("ascii"),
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )
        try:
            if sensor == "emg":
                self._decode_emg(host_time, host_monotonic_ns, payload)
            elif sensor == "imu":
                self._decode_imu(host_time, host_monotonic_ns, payload)
            else:
                self._decode_piezo(host_time, host_monotonic_ns, payload)
        except (ValueError, struct.error) as error:
            print(f"Could not decode {sensor} packet ({len(payload)} bytes): {error}")

    def _write_values(
        self,
        sensor: str,
        host_time: str,
        host_monotonic_ns: int,
        sequence: int,
        device_time: int | str,
        channels: list[list[int]],
    ) -> None:
        writer = self.writers.get(sensor)
        if writer is None:
            return
        for sensor_id, values in enumerate(channels):
            for sample_id, value in enumerate(values):
                writer.writerow(
                    (host_time, host_monotonic_ns, sequence, device_time,
                     sensor_id, sample_id, value)
                )

    def _decode_emg(self, host_time: str, host_monotonic_ns: int, data: bytes) -> None:
        if len(data) == struct.calcsize("<HIQ80H"):
            _, sequence, device_time, *values = struct.unpack("<HIQ80H", data)
            channels = [values[:40], values[40:]]
        elif len(data) == struct.calcsize("<40HI"):
            *values, sequence = struct.unpack("<40HI", data)
            device_time, channels = "", [values]
        elif len(data) >= 6 and (len(data) - 4) % 2 == 0:
            sequence = int.from_bytes(data[:4], "little")
            device_time = ""
            channels = [list(struct.unpack(f"<{(len(data) - 4) // 2}H", data[4:]))]
        else:
            raise ValueError("unknown EMG packet layout")
        self._write_values("emg", host_time, host_monotonic_ns, sequence, device_time, channels)
        if self.sample_sink is not None:
            self.sample_sink("emg", sequence, channels)

    def _decode_piezo(self, host_time: str, host_monotonic_ns: int, data: bytes) -> None:
        if len(data) == struct.calcsize("<HIQ10H"):
            _, sequence, device_time, *values = struct.unpack("<HIQ10H", data)
        elif len(data) >= 6 and (len(data) - 4) % 2 == 0:
            sequence = int.from_bytes(data[:4], "little")
            device_time = ""
            values = list(struct.unpack(f"<{(len(data) - 4) // 2}H", data[4:]))
        else:
            raise ValueError("unknown piezo packet layout")
        self._write_values("piezo", host_time, host_monotonic_ns, sequence, device_time, [values])
        if self.sample_sink is not None:
            self.sample_sink("piezo", sequence, [values])

    def _decode_imu(self, host_time: str, host_monotonic_ns: int, data: bytes) -> None:
        samples: list[tuple[int, list[float]]]
        if len(data) == struct.calcsize("<HIQ12h"):
            _, sequence, device_time, *raw = struct.unpack("<HIQ12h", data)
            samples = [(0, [value / 1000.0 for value in raw[:6]])]
            samples.append((1, [value / 1000.0 for value in raw[6:]]))
            self.imu_units = ("g", "dps")
        elif len(data) == struct.calcsize("<9fI"):
            *values, sequence = struct.unpack("<9fI", data)
            device_time = ""
            samples = [(0, list(values))]
            self.imu_units = ("firmware units", "firmware units")
        elif len(data) >= 28 and (len(data) - 4) % 24 == 0:
            sequence = int.from_bytes(data[:4], "little")
            device_time = ""
            values = struct.unpack(f"<{(len(data) - 4) // 4}f", data[4:])
            samples = [(0, list(values[index : index + 6])) for index in range(0, len(values), 6)]
            self.imu_units = ("mg", "mdps")
        else:
            raise ValueError("unknown IMU packet layout")

        writer = self.writers.get("imu")
        sensor_sample_counts: dict[int, int] = {}
        for sensor_id, values in samples:
            sample_id = sensor_sample_counts.get(sensor_id, 0)
            sensor_sample_counts[sensor_id] = sample_id + 1
            padded = values + [""] * (len(IMU_COLUMNS) - len(values))
            if writer is not None:
                writer.writerow(
                    (
                        host_time, host_monotonic_ns, sequence, device_time,
                        sensor_id, sample_id, *padded[: len(IMU_COLUMNS)],
                    )
                )
        if self.sample_sink is not None:
            channels: list[list[object]] = []
            for sensor_id, values in samples:
                while len(channels) <= sensor_id:
                    channels.append([])
                channels[sensor_id].append(values)
            self.sample_sink("imu", sequence, channels)


async def record(args: argparse.Namespace) -> int:
    """Connect to a LIMB server and record every available sensor stream."""
    if getattr(args, "preview", False):
        if args.dataset_label:
            print("Preview cannot be combined with labeled capture.")
            return 2
        from ble_preview import run_ble_preview

        return await run_ble_preview(args, BleRecorder, SENSOR_UUIDS)
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("bleak is missing. Recreate the simulation environment.")
        return 2

    if args.duration < 0 or args.scan_timeout <= 0:
        print("Duration cannot be negative and scan timeout must be positive.")
        return 2
    try:
        dataset_label = LabeledBleCapture.validate_label(args.dataset_label)
    except ValueError as error:
        print(error)
        return 2

    print(f"Scanning for {args.device!r}...")
    device = args.address or await BleakScanner.find_device_by_name(
        args.device,
        timeout=args.scan_timeout,
    )
    if not device:
        print(f"BLE device {args.device!r} was not found.")
        return 1

    session = create_session_directory("ble", args.subject)
    capture = LabeledBleCapture(session, dataset_label, args.subject) if dataset_label else None
    recorder = BleRecorder(session, capture.add_packet if capture is not None else None)
    started = time.monotonic()
    subscribed: list[str] = []
    print(f"Saving to {session}")

    try:
        async with BleakClient(device) as client:
            print(f"Connected to {getattr(device, 'address', device)}")
            for sensor, uuid in SENSOR_UUIDS.items():
                try:
                    await client.start_notify(
                        uuid,
                        lambda _sender, data, name=sensor: recorder.handle(name, data),
                    )
                    subscribed.append(sensor)
                    print(f"Recording {sensor.upper()}")
                except Exception as error:
                    print(f"{sensor.upper()} is unavailable: {error}")

            if not subscribed:
                print("The device has none of the known LIMB sensor characteristics.")
                return 1

            if capture is not None:
                if not {"emg", "imu"}.issubset(subscribed):
                    print("Labeled capture needs both EMG and IMU characteristics.")
                    return 1
                print(
                    f"Labeled capture: 20 rest, 40 label {dataset_label}, 20 rest "
                    "(80 windows total)."
                )
                for remaining in (3, 2, 1):
                    print(f"Starting in {remaining}...")
                    await asyncio.sleep(1.0)
                capture.start()
                started = time.monotonic()
                print("Recording now. Keep the first 20 windows at rest.")

            while args.duration == 0 or time.monotonic() - started < args.duration:
                await asyncio.sleep(1.0)
                summary = ", ".join(f"{name} {recorder.counts[name]}" for name in subscribed)
                print(f"Packets: {summary}")
                if capture is not None:
                    if capture.emg_complete:
                        break
                    if capture.seconds_since_progress > 10.0:
                        print("No complete EMG window for 10 seconds; capture is incomplete.")
                        break
    finally:
        recorder.close()
        if capture is not None:
            capture.save()
        write_metadata(
            session,
            {
                "source": "ble",
                "subject": args.subject,
                "device": args.device,
                "service_uuid": SERVICE_UUID,
                "characteristics": SENSOR_UUIDS,
                "duration_seconds": round(time.monotonic() - started, 3),
                "packet_counts": recorder.counts,
                **experiment_metadata(),
                **({"labeled_capture": capture.summary()} if capture is not None else {}),
            },
        )

    print(f"Saved BLE session to {session}")
    if capture is not None and not capture.complete:
        print("Labeled capture is incomplete; review the raw files before using it.")
        return 1
    return 0


def main() -> int:
    """Run the asynchronous recorder with clean Ctrl+C handling."""
    args = parse_args()
    try:
        return asyncio.run(record(args))
    except KeyboardInterrupt:
        print("Stopping BLE recording...")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
