# Majd, Christian
"""Decode LIMB BLE packets and optionally save their samples."""

from __future__ import annotations

import base64
import csv
import json
import struct
import time
from collections.abc import Callable
from pathlib import Path

from clock_sync import ClockSynchronizer
from common import (
    utc_now,
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


# Continuous ADC packets use a 22-byte header followed by 1-40 samples.
HIGH_RATE_EMG_HEADER = struct.Struct("<HIQIHH")


HIGH_RATE_IMU_PACKET = struct.Struct("<HIQQ12h")


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
        self.sequence_gaps = {name: 0 for name in self.sensors}
        self.missing_packets = {name: 0 for name in self.sensors}
        self.decode_errors = {name: 0 for name in self.sensors}
        self.last_sequences: dict[str, int] = {}
        self.raw_file = (
            (session / "packets.jsonl").open("w", encoding="utf-8", buffering=1)
            if session is not None
            else None
        )
        self.files = {}
        if session is not None:
            self.files = {
                name: (session / f"{name}.csv").open("w", newline="", encoding="utf-8")
                for name in self.sensors
            }
        self.writers = {name: csv.writer(file) for name, file in self.files.items()}
        if "emg" in self.writers:
            self.writers["emg"].writerow(
                (
                    "host_time",
                    "host_monotonic_ns",
                    "sequence",
                    "device_time",
                    "synced_host_monotonic_ns",
                    "sensor",
                    "sample",
                    "value",
                )
            )
        if "piezo" in self.writers:
            self.writers["piezo"].writerow(
                (
                    "host_time",
                    "host_monotonic_ns",
                    "sequence",
                    "device_time",
                    "synced_host_monotonic_ns",
                    "sensor",
                    "sample",
                    "value",
                )
            )
        if "imu" in self.writers:
            self.writers["imu"].writerow(
                (
                    "host_time",
                    "host_monotonic_ns",
                    "sequence",
                    "device_time",
                    "synced_host_monotonic_ns",
                    "sensor",
                    "sample",
                    *IMU_COLUMNS,
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
                        **({"device_time_us": device_time} if device_time is not None else {}),
                        **(
                            {"synced_host_monotonic_ns": synced_time}
                            if isinstance(synced_time, int)
                            else {}
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
            self.decode_errors[sensor] += 1
            print(f"Could not decode {sensor} packet ({len(payload)} bytes): {error}")

    def _track_sequence(self, sensor: str, sequence: int) -> None:
        previous = self.last_sequences.get(sensor)
        self.last_sequences[sensor] = sequence
        if previous is None:
            return
        difference = (sequence - previous) & 0xFFFFFFFF
        if difference != 1:
            self.sequence_gaps[sensor] += 1
            # Duplicates, reordered packets and resets are not forward packet loss.
            if 1 < difference < 0x80000000:
                self.missing_packets[sensor] += difference - 1

    def packet_health(self) -> dict[str, dict[str, int]]:
        """Report sequence discontinuities separately from malformed packets."""
        return {
            name: {
                "received_packets": self.counts[name],
                "sequence_gaps": self.sequence_gaps[name],
                "missing_packets": self.missing_packets[name],
                "decode_errors": self.decode_errors[name],
            }
            for name in self.sensors
        }

    @staticmethod
    def _packet_device_time(sensor: str, payload: bytes) -> int | None:
        if sensor == "emg" and len(payload) == CURRENT_EMG_PACKET.size:
            return CURRENT_EMG_PACKET.unpack(payload)[2]
        if sensor == "emg" and len(payload) >= HIGH_RATE_EMG_HEADER.size:
            flags, _sequence, first_us, period_us, count, _offset = (
                HIGH_RATE_EMG_HEADER.unpack_from(payload)
            )
            if (
                flags >> 8 == 1
                and period_us > 0
                and 0 < count <= 40
                and len(payload) == HIGH_RATE_EMG_HEADER.size + 2 * count
            ):
                return first_us
        if sensor == "imu" and len(payload) == HIGH_RATE_IMU_PACKET.size:
            flags, _sequence, first_us, second_us, *_values = HIGH_RATE_IMU_PACKET.unpack(payload)
            if flags >> 8 == 1:
                return first_us if flags & 0x01 else second_us if flags & 0x02 else None
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
                    (
                        host_time,
                        host_monotonic_ns,
                        sequence,
                        device_time,
                        synced_time,
                        sensor_id,
                        sample_id,
                        value,
                    )
                )

    def _decode_emg(self, host_time: str, host_monotonic_ns: int, data: bytes) -> None:
        if len(data) >= HIGH_RATE_EMG_HEADER.size + 2:
            flags, sequence, first_us, period_us, count, offset = HIGH_RATE_EMG_HEADER.unpack_from(
                data
            )
            if (
                flags >> 8 == 1
                and period_us > 0
                and 0 < count <= 40
                and len(data) == HIGH_RATE_EMG_HEADER.size + 2 * count
            ):
                values = list(struct.unpack_from(f"<{count}H", data, HIGH_RATE_EMG_HEADER.size))
                writer = self.writers.get("emg")
                if writer is not None:
                    for index, value in enumerate(values):
                        device_us = first_us + index * period_us
                        writer.writerow(
                            (
                                host_time,
                                host_monotonic_ns,
                                sequence,
                                device_us,
                                self._synced_time(device_us),
                                0,
                                offset + index,
                                value,
                            )
                        )
                self._track_sequence("emg", sequence)
                if self.sample_sink is not None:
                    self.sample_sink("emg", sequence, [values])
                return
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
        self._write_values("emg", host_time, host_monotonic_ns, sequence, device_time, channels)
        self._track_sequence("emg", sequence)
        if self.sample_sink is not None:
            self.sample_sink("emg", sequence, channels)

    def _decode_piezo(self, host_time: str, host_monotonic_ns: int, data: bytes) -> None:
        if len(data) == TIMESTAMPED_PACKETS["piezo"].size:
            _, sequence, device_time, *values = TIMESTAMPED_PACKETS["piezo"].unpack(data)
        elif len(data) >= 6 and (len(data) - 4) % 2 == 0:
            sequence = int.from_bytes(data[:4], "little")
            device_time = ""
            values = list(struct.unpack(f"<{(len(data) - 4) // 2}H", data[4:]))
        else:
            raise ValueError("unknown piezo packet layout")
        self._write_values("piezo", host_time, host_monotonic_ns, sequence, device_time, [values])
        self._track_sequence("piezo", sequence)
        if self.sample_sink is not None:
            self.sample_sink("piezo", sequence, [values])

    def _decode_imu(self, host_time: str, host_monotonic_ns: int, data: bytes) -> None:
        if len(data) == HIGH_RATE_IMU_PACKET.size:
            flags, sequence, first_us, second_us, *raw = HIGH_RATE_IMU_PACKET.unpack(data)
            if flags >> 8 != 1:
                raise ValueError("unsupported Jonas IMU packet version")
            connected_mask = flags & 0x03
            samples = []
            for sensor_id, (device_us, offset) in enumerate(((first_us, 0), (second_us, 6))):
                if not connected_mask & (1 << sensor_id):
                    continue
                values = [value / 1000.0 for value in raw[offset : offset + 3]] + [
                    value / 100.0 for value in raw[offset + 3 : offset + 6]
                ]
                samples.append((sensor_id, device_us, values))
            writer = self.writers.get("imu")
            for sensor_id, device_us, values in samples:
                if writer is not None:
                    writer.writerow(
                        (
                            host_time,
                            host_monotonic_ns,
                            sequence,
                            device_us,
                            self._synced_time(device_us),
                            sensor_id,
                            0,
                            *(values + [""] * (len(IMU_COLUMNS) - 6)),
                        )
                    )
            self.imu_units = ("g", "dps")
            self._track_sequence("imu", sequence)
            if self.sample_sink is not None:
                channels = [[], []]
                for sensor_id, _device_us, values in samples:
                    channels[sensor_id] = [values]
                self.sample_sink("imu", sequence, channels)
            return
        samples: list[tuple[int, list[float]]]
        if len(data) == TIMESTAMPED_PACKETS["imu"].size:
            format_flags, sequence, device_time, *raw = TIMESTAMPED_PACKETS["imu"].unpack(data)
            # AURORA v1 stores its version in the high byte and a connected
            # role mask in the low byte. HT25 used the same packet size without
            # those flags, so legacy packets continue to expose both channels.
            connected_mask = format_flags & 0xFF if format_flags >> 8 == 1 else 0x03
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
            samples = [(0, list(values[index : index + 6])) for index in range(0, len(values), 6)]
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
                        host_time,
                        host_monotonic_ns,
                        sequence,
                        device_time,
                        synced_time,
                        sensor_id,
                        sample_id,
                        *padded[: len(IMU_COLUMNS)],
                    )
                )
        self._track_sequence("imu", sequence)
        if self.sample_sink is not None:
            channels: list[list[object]] = []
            for sensor_id, values in samples:
                while len(channels) <= sensor_id:
                    channels.append([])
                channels[sensor_id].append(values)
            self.sample_sink("imu", sequence, channels)
