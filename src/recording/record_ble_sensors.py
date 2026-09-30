"""Record synchronized AURORA IMU and EMG notifications over BLE."""

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

from ble_dataset import LabeledBleCapture
from clock_sync import BleClockSyncClient, ClockSynchronizer, TIME_SYNC_UUID
from common import (
    create_session_directory,
    env_float,
    experiment_metadata,
    utc_now,
    write_metadata,
)


SERVICE_UUID = "23011525-1212-efde-1523-785feabcd122"
SENSOR_UUIDS = {
    "emg": "24011525-1212-efde-1523-785feabcd122",
    "imu": "25011525-1212-efde-1523-785feabcd122",
    "piezo": "26011525-1212-efde-1523-785feabcd122",
}
TIMESTAMPED_PACKETS = {
    "emg": struct.Struct("<HIQ80H"),
    "imu": struct.Struct("<HIQ12h"),
    "piezo": struct.Struct("<HIQ10H"),
}
DEFAULT_SENSORS = ("imu", "emg")
CURRENT_EMG_PACKET = struct.Struct("<HIQH")
LEGACY_EMG_PACKET = struct.Struct("<40HI")
LEGACY_IMU_PACKET = struct.Struct("<9fI")
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
    parser.add_argument(
        "--sensors",
        nargs="+",
        choices=tuple(SENSOR_UUIDS),
        default=None,
        help=(
            "Sensor streams to request. Each characteristic is tried independently, "
            "so an unavailable sensor does not stop the others."
        ),
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
        clock_sync: ClockSynchronizer | None = None,
        sensors: tuple[str, ...] | None = None,
    ) -> None:
        self.session = session
        self.sample_sink = sample_sink
        self.clock_sync = clock_sync
        self.sensors = DEFAULT_SENSORS if sensors is None else sensors
        self.imu_units = ("device units", "device units")
        self.counts = {name: 0 for name in self.sensors}
        self.raw_file = (
            (session / "packets.jsonl").open("w", encoding="utf-8", buffering=1)
            if session is not None else None
        )
        self.files = {}
        if session is not None:
            self.files = {
                name: (session / f"{name}.csv").open(
                    "w", newline="", encoding="utf-8"
                )
                for name in self.sensors
            }
        self.writers = {name: csv.writer(file) for name, file in self.files.items()}
        if "emg" in self.writers:
            self.writers["emg"].writerow(
                (
                    "host_time", "host_monotonic_ns", "sequence", "device_time",
                    "synced_host_monotonic_ns",
                    "sensor", "sample", "value",
                )
            )
        if "piezo" in self.writers:
            self.writers["piezo"].writerow(
                (
                    "host_time", "host_monotonic_ns", "sequence", "device_time",
                    "synced_host_monotonic_ns",
                    "sensor", "sample", "value",
                )
            )
        if "imu" in self.writers:
            self.writers["imu"].writerow(
                (
                    "host_time", "host_monotonic_ns", "sequence", "device_time",
                    "synced_host_monotonic_ns",
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
        device_time = self._packet_device_time(sensor, payload)
        synced_time = self._synced_time(device_time)
        if self.raw_file is not None:
            self.raw_file.write(
                json.dumps(
                    {
                        "host_time": host_time,
                        "host_monotonic_ns": host_monotonic_ns,
                        **(
                            {"device_time_us": device_time}
                            if device_time is not None else {}
                        ),
                        **(
                            {"synced_host_monotonic_ns": synced_time}
                            if isinstance(synced_time, int) else {}
                        ),
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

    @staticmethod
    def _packet_device_time(sensor: str, payload: bytes) -> int | None:
        if sensor == "emg" and len(payload) == CURRENT_EMG_PACKET.size:
            return CURRENT_EMG_PACKET.unpack(payload)[2]
        packet = TIMESTAMPED_PACKETS.get(sensor)
        if packet is None or len(payload) != packet.size:
            return None
        return struct.unpack_from("<Q", payload, 6)[0]

    def _synced_time(self, device_time: int | str | None) -> int | str:
        if self.clock_sync is None:
            return ""
        mapped = self.clock_sync.to_host_ns(device_time)
        return mapped if mapped is not None else ""

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
        synced_time = self._synced_time(device_time)
        for sensor_id, values in enumerate(channels):
            for sample_id, value in enumerate(values):
                writer.writerow(
                    (host_time, host_monotonic_ns, sequence, device_time, synced_time,
                     sensor_id, sample_id, value)
                )

    def _decode_emg(
        self, host_time: str, host_monotonic_ns: int, data: bytes
    ) -> None:
        if len(data) == CURRENT_EMG_PACKET.size:
            format_flags, sequence, device_time, value = CURRENT_EMG_PACKET.unpack(data)
            version = format_flags >> 8
            if version != 1:
                raise ValueError(f"unsupported current EMG packet version {version}")
            channels = [[value]] if format_flags & 0x01 else []
        elif len(data) == TIMESTAMPED_PACKETS["emg"].size:
            _, sequence, device_time, *values = TIMESTAMPED_PACKETS["emg"].unpack(data)
            channels = [values[:40], values[40:]]
        elif len(data) == LEGACY_EMG_PACKET.size:
            *values, sequence = LEGACY_EMG_PACKET.unpack(data)
            device_time, channels = "", [values]
        elif len(data) >= 6 and (len(data) - 4) % 2 == 0:
            sequence = int.from_bytes(data[:4], "little")
            device_time = ""
            channels = [list(struct.unpack(f"<{(len(data) - 4) // 2}H", data[4:]))]
        else:
            raise ValueError("unknown EMG packet layout")
        self._write_values(
            "emg", host_time, host_monotonic_ns, sequence, device_time, channels
        )
        if self.sample_sink is not None:
            self.sample_sink("emg", sequence, channels)

    def _decode_piezo(
        self, host_time: str, host_monotonic_ns: int, data: bytes
    ) -> None:
        if len(data) == TIMESTAMPED_PACKETS["piezo"].size:
            _, sequence, device_time, *values = TIMESTAMPED_PACKETS["piezo"].unpack(data)
        elif len(data) >= 6 and (len(data) - 4) % 2 == 0:
            sequence = int.from_bytes(data[:4], "little")
            device_time = ""
            values = list(struct.unpack(f"<{(len(data) - 4) // 2}H", data[4:]))
        else:
            raise ValueError("unknown piezo packet layout")
        self._write_values(
            "piezo", host_time, host_monotonic_ns, sequence, device_time, [values]
        )
        if self.sample_sink is not None:
            self.sample_sink("piezo", sequence, [values])

    def _decode_imu(
        self, host_time: str, host_monotonic_ns: int, data: bytes
    ) -> None:
        samples: list[tuple[int, list[float]]]
        if len(data) == TIMESTAMPED_PACKETS["imu"].size:
            format_flags, sequence, device_time, *raw = TIMESTAMPED_PACKETS["imu"].unpack(data)
            # AURORA v1 stores its version in the high byte and a connected
            # role mask in the low byte. HT25 used the same packet size without
            # those flags, so legacy packets continue to expose both channels.
            connected_mask = (
                format_flags & 0xFF if format_flags >> 8 == 1 else 0x03
            )
            samples = []
            if connected_mask & 0x01:
                samples.append((0, [value / 1000.0 for value in raw[:6]]))
            if connected_mask & 0x02:
                samples.append((1, [value / 1000.0 for value in raw[6:]]))
            self.imu_units = ("g", "dps")
        elif len(data) == LEGACY_IMU_PACKET.size:
            *values, sequence = LEGACY_IMU_PACKET.unpack(data)
            device_time = ""
            samples = [(0, list(values))]
            self.imu_units = ("firmware units", "firmware units")
        elif len(data) >= 28 and (len(data) - 4) % 24 == 0:
            sequence = int.from_bytes(data[:4], "little")
            device_time = ""
            values = struct.unpack(f"<{(len(data) - 4) // 4}f", data[4:])
            samples = [
                (0, list(values[index : index + 6]))
                for index in range(0, len(values), 6)
            ]
            self.imu_units = ("mg", "mdps")
        else:
            raise ValueError("unknown IMU packet layout")

        writer = self.writers.get("imu")
        synced_time = self._synced_time(device_time)
        sensor_sample_counts: dict[int, int] = {}
        for sensor_id, values in samples:
            sample_id = sensor_sample_counts.get(sensor_id, 0)
            sensor_sample_counts[sensor_id] = sample_id + 1
            padded = values + [""] * (len(IMU_COLUMNS) - len(values))
            if writer is not None:
                writer.writerow(
                    (
                        host_time, host_monotonic_ns, sequence, device_time,
                        synced_time,
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
    requested_value = getattr(args, "sensors", None)
    requested = DEFAULT_SENSORS if requested_value is None else tuple(
        dict.fromkeys(requested_value)
    )
    if not requested or any(name not in SENSOR_UUIDS for name in requested):
        print("Choose at least one sensor: imu or emg (piezo is legacy-only).")
        return 2
    if getattr(args, "preview", False):
        if args.dataset_label:
            print("Preview cannot be combined with labeled capture.")
            return 2
        from ble_preview import run_ble_preview

        def preview_recorder(session, sample_sink):
            return BleRecorder(session, sample_sink, sensors=requested)

        return await run_ble_preview(
            args,
            preview_recorder,
            {name: SENSOR_UUIDS[name] for name in requested},
        )
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
    capture = (
        LabeledBleCapture(session, dataset_label, args.subject)
        if dataset_label else None
    )
    clock_sync = ClockSynchronizer()
    recorder = BleRecorder(
        session,
        capture.add_packet if capture is not None else None,
        clock_sync,
        requested,
    )
    started = time.monotonic()
    subscribed: list[str] = []
    unavailable: dict[str, str] = {}
    sync_client: BleClockSyncClient | None = None
    print(f"Saving to {session}")

    try:
        async with BleakClient(device) as client:
            print(f"Connected to {getattr(device, 'address', device)}")
            sync_client = BleClockSyncClient(client, clock_sync)
            sync_available = await sync_client.start()
            if sync_available:
                print(
                    "Device clock synchronized to host monotonic time "
                    f"({len(clock_sync.samples)} exchanges)"
                )
            else:
                print(
                    "Time-sync characteristic unavailable; preserving arrival times only"
                )
            for sensor in requested:
                uuid = SENSOR_UUIDS[sensor]
                try:
                    await client.start_notify(
                        uuid,
                        lambda _sender, data, name=sensor: recorder.handle(name, data),
                    )
                    subscribed.append(sensor)
                    print(f"Recording {sensor.upper()}")
                except Exception as error:
                    unavailable[sensor] = str(error)
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
                summary = ", ".join(
                    f"{name} {recorder.counts[name]}" for name in subscribed
                )
                print(f"Packets: {summary}")
                if capture is not None:
                    if capture.emg_complete:
                        break
                    if capture.seconds_since_progress > 10.0:
                        print(
                            "No complete EMG window for 10 seconds; capture is incomplete."
                        )
                        break
    finally:
        if sync_client is not None:
            await sync_client.close()
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
                "characteristics": {
                    name: SENSOR_UUIDS[name] for name in requested
                },
                "requested_sensors": list(requested),
                "subscribed_sensors": subscribed,
                "unavailable_sensors": unavailable,
                "time_sync_characteristic": TIME_SYNC_UUID,
                "time_sync": clock_sync.summary(),
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
