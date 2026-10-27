# Majd, Christian
"""Decode AURORA BLE packets and earlier LIMB formats."""

from __future__ import annotations

import base64
import contextlib
import csv
import json
import struct
import time
from collections.abc import Callable
from pathlib import Path

from aurora.sensors.clock_sync import ClockMapping, ClockSynchronizer
from aurora.sensors.emg_packets import CURRENT_EMG_PACKET as CURRENT_EMG_PACKET
from aurora.sensors.emg_packets import HIGH_RATE_EMG_HEADER as HIGH_RATE_EMG_HEADER
from aurora.sensors.emg_packets import LEGACY_EMG_PACKET as LEGACY_EMG_PACKET
from aurora.sensors.emg_packets import decode_emg
from aurora.sensors.imu_packets import HIGH_RATE_IMU_PACKET as HIGH_RATE_IMU_PACKET
from aurora.sensors.imu_packets import LEGACY_IMU_PACKET as LEGACY_IMU_PACKET
from aurora.sensors.imu_packets import decode_imu
from aurora.time_utils import utc_now

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
    """Decode current and inherited packets while preserving every raw packet."""

    def __init__(
        self,
        session: Path | None,
        sample_sink: Callable[[str, int, list[list[object]]], None] | None = None,
        clock_sync: ClockSynchronizer | None = None,
        sensors: tuple[str, ...] | None = None,
        *,
        legacy_emg_prefix: bool = False,
        legacy_emg_fixed: bool = False,
    ) -> None:
        self.session = session
        self.sample_sink = sample_sink
        self.clock_sync = clock_sync
        self.legacy_emg_prefix = legacy_emg_prefix
        self.legacy_emg_fixed = legacy_emg_fixed
        self.connection_generation = None
        self.last_packet = None
        self.last_host_monotonic_ns = None
        self.sensors = DEFAULT_SENSORS if sensors is None else sensors
        self.imu_units = ("device units", "device units")
        self.counts = {name: 0 for name in self.sensors}
        self.sequence_gaps = {name: 0 for name in self.sensors}
        self.missing_packets = {name: 0 for name in self.sensors}
        self.decode_errors = {name: 0 for name in self.sensors}
        self.duplicate_packets = {name: 0 for name in self.sensors}
        self.reordered_packets = {name: 0 for name in self.sensors}
        self.sample_time_gaps = {name: 0 for name in self.sensors}
        self.gyro_saturation_samples = 0
        self._emg_end_us = None
        self.last_sequences: dict[str, int] = {}
        self.raw_file = None
        self.files = {}
        try:
            self._open_files()
        except Exception:
            with contextlib.suppress(Exception):
                self.close()
            raise

    def _open_files(self):
        self.raw_file = (
            (self.session / "packets.jsonl").open("w", encoding="utf-8", buffering=1)
            if self.session is not None
            else None
        )
        self.files = {}
        if self.session is not None:
            for name in self.sensors:
                self.files[name] = (self.session / f"{name}.csv").open(
                    "w", newline="", encoding="utf-8"
                )
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
                    "connection_generation",
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
                    "connection_generation",
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
                    "connection_generation",
                    "gyro_saturated",
                )
            )

    def close(self) -> None:
        """Close independent files even when one flush fails; failed closes can retry."""
        errors = []
        for file in (self.raw_file, *self.files.values()):
            if file is not None:
                try:
                    file.close()
                except Exception as error:
                    errors.append(str(error))
        if errors:
            raise RuntimeError("; ".join(errors))

    def handle(
        self,
        sensor: str,
        data: bytes | bytearray,
        *,
        host_time: str | None = None,
        host_monotonic_ns: int | None = None,
    ) -> None:
        """Save one notification and decode it when its packet layout is known."""
        host_time = utc_now() if host_time is None else host_time
        host_monotonic_ns = time.monotonic_ns() if host_monotonic_ns is None else host_monotonic_ns
        self.last_host_monotonic_ns = host_monotonic_ns
        payload = bytes(data)
        self.counts[sensor] += 1
        device_time = (
            None
            if sensor == "emg" and (self.legacy_emg_prefix or self.legacy_emg_fixed)
            else self._packet_device_time(sensor, payload)
        )
        synced_time = self._synced_time(device_time)
        if self.raw_file is not None:
            self.raw_file.write(
                json.dumps(
                    {
                        "host_time": host_time,
                        "host_monotonic_ns": host_monotonic_ns,
                        **(
                            {"connection_generation": self.connection_generation}
                            if self.connection_generation is not None
                            else {}
                        ),
                        **({"device_time_us": device_time} if device_time is not None else {}),
                        **(
                            {"synced_host_monotonic_ns": synced_time}
                            if isinstance(synced_time, int)
                            else {}
                        ),
                        "sensor": sensor,
                        **(
                            {"clock_fit": self.clock_sync.provenance()}
                            if isinstance(self.clock_sync, ClockMapping)
                            else {}
                        ),
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
        if previous is None:
            self.last_sequences[sensor] = sequence
            return
        difference = (sequence - previous) & 0xFFFFFFFF
        if difference != 1:
            self.sequence_gaps[sensor] += 1
            if difference == 0:
                self.duplicate_packets[sensor] += 1
            elif difference >= 0x80000000:
                self.reordered_packets[sensor] += 1
            # Duplicates, reordered packets and resets are not forward packet loss.
            if 1 < difference < 0x80000000:
                self.missing_packets[sensor] += difference - 1
        if 0 < difference < 0x80000000:
            self.last_sequences[sensor] = sequence

    def reset_sequence_tracking(self) -> None:
        """Start from the next connection's first packet, keeping the totals."""
        self.last_sequences.clear()
        self._emg_end_us = None

    def packet_health(self) -> dict[str, dict[str, int]]:
        """Report sequence discontinuities separately from malformed packets."""
        return {
            name: {
                "received_packets": self.counts[name],
                "sequence_gaps": self.sequence_gaps[name],
                "missing_packets": self.missing_packets[name],
                "decode_errors": self.decode_errors[name],
                "duplicate_packets": self.duplicate_packets[name],
                "reordered_packets": self.reordered_packets[name],
                "sample_time_gaps": self.sample_time_gaps[name],
                "gyro_saturation_samples": self.gyro_saturation_samples if name == "imu" else 0,
            }
            for name in self.sensors
        }

    @staticmethod
    def _packet_device_time(sensor: str, payload: bytes) -> int | None:
        try:
            if sensor == "emg":
                return decode_emg(payload).first_time_us
            if sensor == "imu":
                packet = decode_imu(payload)
                return next(
                    (
                        stamp
                        for samples, stamp in zip(
                            packet.channels, packet.sample_times_us, strict=True
                        )
                        if samples
                    ),
                    None,
                )
            packet = TIMESTAMPED_PACKETS.get(sensor)
            if packet is not None and len(payload) == packet.size:
                return struct.unpack_from("<Q", payload, 6)[0]
        except (ValueError, struct.error):
            pass
        return None

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
                        self.connection_generation,
                    )
                )

    def _decode_emg(self, host_time: str, host_monotonic_ns: int, data: bytes) -> None:
        packet = decode_emg(
            data, legacy_prefix=self.legacy_emg_prefix, legacy_fixed=self.legacy_emg_fixed
        )
        self.last_packet = packet
        channels = [list(channel) for channel in packet.channels]
        previous = self.last_sequences.get("emg")
        new_packet = previous is None or 0 < (packet.sequence - previous) & 0xFFFFFFFF < 0x80000000
        if new_packet and packet.first_time_us is not None and packet.period_us is not None:
            if self._emg_end_us is not None and abs(packet.first_time_us - self._emg_end_us) > 1:
                self.sample_time_gaps["emg"] += 1
            self._emg_end_us = packet.first_time_us + len(channels[0]) * packet.period_us
        writer = self.writers.get("emg")
        if writer is not None:
            for sensor_id, values in enumerate(channels):
                for index, value in enumerate(values):
                    device_us = (
                        packet.first_time_us + index * (packet.period_us or 0)
                        if packet.first_time_us is not None
                        else ""
                    )
                    writer.writerow(
                        (
                            host_time,
                            host_monotonic_ns,
                            packet.sequence,
                            device_us,
                            self._synced_time(device_us),
                            sensor_id,
                            packet.offset + index,
                            value,
                            self.connection_generation,
                        )
                    )
        self._track_sequence("emg", packet.sequence)
        if self.sample_sink is not None:
            self.sample_sink("emg", packet.sequence, channels)

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
        packet = decode_imu(data)
        self.gyro_saturation_samples += len(packet.gyro_saturated_roles)
        self.last_packet = packet
        self.imu_units = packet.units
        channels = [[list(values) for values in samples] for samples in packet.channels]
        writer = self.writers.get("imu")
        if writer is not None:
            for sensor_id, samples in enumerate(channels):
                device_us = packet.sample_times_us[sensor_id]
                for sample_id, values in enumerate(samples):
                    padded = values + [""] * (len(IMU_COLUMNS) - len(values))
                    writer.writerow(
                        (
                            host_time,
                            host_monotonic_ns,
                            packet.sequence,
                            device_us if device_us is not None else "",
                            self._synced_time(device_us),
                            sensor_id,
                            sample_id,
                            *padded[: len(IMU_COLUMNS)],
                            self.connection_generation,
                            int(sensor_id in packet.gyro_saturated_roles),
                        )
                    )
        self._track_sequence("imu", packet.sequence)
        if self.sample_sink is not None:
            # Older single-role consumers expect no trailing absent channel.
            while channels and not channels[-1]:
                channels.pop()
            self.sample_sink("imu", packet.sequence, channels)
