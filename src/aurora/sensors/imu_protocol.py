# Majd, Christian
"""Read current and older IMU packets in one format."""

from __future__ import annotations

import json
import math
import re
import time
from typing import Any

from aurora.sensors.serial_protocol import parse_sensor_line

STANDARD_GRAVITY = 9.80665
ROLE_LABELS = {"shoulder": "Shoulder IMU", "wrist": "Forearm IMU"}
LEGACY_ACCEL_G_PER_LSB = 1.0 / 4096.0
LEGACY_GYRO_DPS_PER_LSB = 1.0 / 65.5


class ImuStreamDecoder:
    """Decode current JSON or the older line-oriented two-IMU serial stream."""

    _identity = re.compile(r"^IMU\s*(?P<sensor>[12])\s+WHO_AM_I:\s*0x(?P<value>[0-9a-fA-F]+)$")
    _sample = re.compile(
        r"^IMU\s*(?P<sensor>[12])\s+(?P<kind>ACC|GYRO):\s*"
        r"(?P<x>-?\d+)\s+(?P<y>-?\d+)\s+(?P<z>-?\d+)$"
    )
    _legacy_emg = re.compile(r"^(?P<time>\d+),(?P<raw>\d+)$")

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.connection_generation = getattr(self, "connection_generation", 0) + 1
        self.identities: dict[int, int] = {}
        self.samples: dict[int, dict[str, tuple[int, int, int]]] = {}
        self.sequence = 0
        self.jonas_imus: dict[str, dict[str, Any]] = {}
        self.jonas_emg: dict[str, Any] = {}
        self.latest_time_us: int | None = None
        self.legacy_receipts: dict[int, int] = {}

    def feed(self, text: str, *, host_monotonic_ns=None) -> dict[str, Any] | None:
        """Consume one serial line and return a complete packet when available."""
        receipt_ns = time.monotonic_ns() if host_monotonic_ns is None else host_monotonic_ns
        line = text.strip()
        jonas = parse_sensor_line(line)
        if jonas is not None:
            sensor, timestamp, values = jonas
            # Each cached channel retains its own device acquisition time. A
            # device clock reset invalidates the cache rather than extending it.
            if self.latest_time_us is not None and timestamp < self.latest_time_us - 500_000:
                self.reset()
            self.latest_time_us = max(timestamp, self.latest_time_us or timestamp)
            role = "shoulder" if sensor == "imu1" else "wrist"
            previous = self.jonas_emg if sensor == "emg" else self.jonas_imus.get(role, {})
            if "sample_time_us" in previous and timestamp <= previous["sample_time_us"]:
                return None
            if sensor == "emg":
                self.jonas_emg = {
                    "connected": True,
                    "gpio": 0,
                    "adc_channel": 0,
                    "adc_raw": values[0],
                    "sample_time_us": timestamp,
                    "host_monotonic_ns": receipt_ns,
                }
            else:
                role = "shoulder" if sensor == "imu1" else "wrist"
                self.jonas_imus[role] = {
                    "connected": True,
                    "model": "LSM6DSO32",
                    "address": "0x6B" if sensor == "imu1" else "0x6A",
                    "sample_time_us": timestamp,
                    "host_monotonic_ns": receipt_ns,
                    "accel_g": dict(
                        zip(("x", "y", "z"), (value / 1000 for value in values[:3]), strict=False)
                    ),
                    "gyro_dps": dict(
                        zip(("x", "y", "z"), (value / 1000 for value in values[3:]), strict=False)
                    ),
                }
            packet = {
                "schema": "aurora.high_rate_serial.v1",
                "device": "ESP32-C3",
                "sample_time_us": timestamp,
                "sequence": self.sequence,
                "updated_sensor": sensor,
                "connection_generation": self.connection_generation,
                "imus": {
                    role: {
                        **value,
                        "connected": 0 <= self.latest_time_us - value["sample_time_us"] <= 500_000,
                    }
                    for role, value in self.jonas_imus.items()
                },
                "emg": {
                    **self.jonas_emg,
                    "connected": bool(self.jonas_emg)
                    and 0 <= self.latest_time_us - self.jonas_emg["sample_time_us"] <= 500_000,
                },
            }
            self.sequence = (self.sequence + 1) & 0xFFFFFFFF
            return packet
        if line.startswith("{"):
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                value = None
            if isinstance(value, dict):
                return value

        emg = self._legacy_emg.match(line)
        if emg is not None:
            raw = int(emg.group("raw"))
            if 0 <= raw <= 4095:
                return {
                    "schema": "aurora.legacy_emg_csv.v1",
                    "device": "ESP32-C3 legacy EMG firmware",
                    "sample_time_us": int(emg.group("time")),
                    "emg": {
                        "connected": True,
                        "adc_channel": 0,
                        "gpio": 0,
                        "adc_raw": raw,
                    },
                }

        identity = self._identity.match(line)
        if identity is not None:
            self.identities[int(identity.group("sensor"))] = int(identity.group("value"), 16)
            return None

        sample = self._sample.match(line)
        if sample is None:
            return None
        sensor_id = int(sample.group("sensor"))
        kind = sample.group("kind").lower()
        values = tuple(int(sample.group(axis)) for axis in ("x", "y", "z"))
        self.samples.setdefault(sensor_id, {})[kind] = values
        if kind != "gyro" or "acc" not in self.samples[sensor_id]:
            return None
        self.legacy_receipts[sensor_id] = receipt_ns
        packet = self._packet(receipt_ns)
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        return packet

    def _packet(self, now_ns) -> dict[str, Any]:
        imus: dict[str, dict[str, Any]] = {}
        for sensor_id, role, address in (
            (1, "shoulder", "0x6B"),
            (2, "wrist", "0x6A"),
        ):
            raw = self.samples.get(sensor_id, {})
            accel = raw.get("acc")
            gyro = raw.get("gyro")
            identity = self.identities.get(sensor_id)
            complete = accel is not None and gyro is not None
            has_signal = bool(complete and any((*accel, *gyro)))
            receipt = self.legacy_receipts.get(sensor_id, 0)
            connected = identity in (None, 0x6C) and has_signal and now_ns - receipt <= 500_000_000
            sensor: dict[str, Any] = {
                "connected": connected,
                "model": "LSM6DSO32",
                "address": address,
                "serial_format": "legacy_text",
                "host_monotonic_ns": receipt,
            }
            if connected and accel is not None and gyro is not None:
                sensor["accel_g"] = {
                    axis: raw_value * LEGACY_ACCEL_G_PER_LSB
                    for axis, raw_value in zip(("x", "y", "z"), accel, strict=False)
                }
                sensor["gyro_dps"] = {
                    axis: raw_value * LEGACY_GYRO_DPS_PER_LSB
                    for axis, raw_value in zip(("x", "y", "z"), gyro, strict=False)
                }
            elif identity not in (None, 0x6C):
                sensor["error"] = f"WHO_AM_I 0x{identity:02X}"
            elif not complete:
                sensor["error"] = "waiting for complete legacy sample"
            else:
                sensor["error"] = "legacy stream reports zero values"
            imus[role] = sensor
        return {
            "schema": "aurora.legacy_dual_imu_text.v1",
            "device": "ESP32-C3 legacy firmware",
            "sequence": self.sequence,
            "imus": imus,
        }


def _axes(value: object, scale: float = 1.0) -> dict[str, float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        value = dict(zip(("x", "y", "z"), value[:3], strict=False))
    if not isinstance(value, dict):
        return None
    result: dict[str, float] = {}
    for axis in ("x", "y", "z"):
        sample = value.get(axis)
        if isinstance(sample, bool) or not isinstance(sample, (int, float)):
            return None
        sample = float(sample) * scale
        if not math.isfinite(sample):
            return None
        result[axis] = sample
    return result


def _sensor_axes(
    value: dict[str, Any],
    native_key: str,
    legacy_keys: tuple[str, ...],
    legacy_scale: float,
) -> dict[str, float] | None:
    """Prefer normalized axes, otherwise scale the first legacy representation."""
    native = value.get(native_key)
    if native is not None:
        return _axes(native)
    legacy = next((value[key] for key in legacy_keys if key in value), None)
    return _axes(legacy, legacy_scale)


def normalize_imu(value: object, role: str) -> dict[str, Any]:
    """Return one IMU in g and degrees/second.

    ``accel``/``gyro`` is the old LIMB25 Adafruit packet (m/s^2 and rad/s).
    ``accel_g``/``gyro_dps`` is the explicit current packet.
    """
    if not isinstance(value, dict):
        return {"role": role, "connected": False, "error": "missing from packet"}

    accel = _sensor_axes(
        value,
        "accel_g",
        ("accel", "acceleration", "accelerometer", "accel_m_s2"),
        1.0 / STANDARD_GRAVITY,
    )
    gyro = _sensor_axes(
        value,
        "gyro_dps",
        ("gyro", "gyroscope", "angular_velocity", "gyro_rad_s"),
        180.0 / math.pi,
    )
    connected = bool(value.get("connected", accel is not None and gyro is not None))
    result: dict[str, Any] = {
        "role": role,
        "connected": connected,
        "model": str(value.get("model", "LSM6DSO32")),
        "address": str(value.get("address", "?")),
    }
    if accel is not None:
        result["accel_g"] = accel
    if gyro is not None:
        result["gyro_dps"] = gyro
    for key in (
        "temperature_c",
        "sda_pin",
        "scl_pin",
        "error",
        "sample_time_us",
        "host_monotonic_ns",
        "connection_generation",
        "gyro_saturated",
        "validity",
    ):
        if key in value:
            result[key] = value[key]
    if connected and (accel is None or gyro is None):
        result["connected"] = False
        result["error"] = "packet is missing complete acceleration or gyroscope axes"
    return result


def extract_imus(packet: object) -> dict[str, dict[str, Any]]:
    """Extract role-named sensors from current, legacy dual, or single packets."""
    if not isinstance(packet, dict):
        return {}

    imus = packet.get("imus")
    if isinstance(imus, dict):
        return {role: normalize_imu(imus.get(role), role) for role in ("shoulder", "wrist")}

    if "imu1" in packet or "imu2" in packet:
        return {
            "shoulder": normalize_imu(packet.get("imu1"), "shoulder"),
            "wrist": normalize_imu(packet.get("imu2"), "wrist"),
        }

    if "imu" in packet:
        # Keep the old single-IMU stream useful during the hardware transition.
        # HT25's one-IMU controller mounted that sensor on the upper arm.
        return {"shoulder": normalize_imu(packet.get("imu"), "shoulder")}
    return {}


def connected_roles(packet: object) -> tuple[str, ...]:
    """Return the connected physical IMU roles in deterministic order."""
    sensors = extract_imus(packet)
    return tuple(
        role for role in ("shoulder", "wrist") if sensors.get(role, {}).get("connected") is True
    )


def i2c_wiring_hint(i2c: object) -> str | None:
    """Translate firmware pull-up checks into an actionable wiring message."""
    if not isinstance(i2c, dict):
        return None
    sda = i2c.get("external_sda_pullup")
    scl = i2c.get("external_scl_pullup")
    if not isinstance(sda, bool) or not isinstance(scl, bool):
        return None
    sda_pin = i2c.get("sda_pin", "?")
    scl_pin = i2c.get("scl_pin", "?")
    if sda and scl:
        return f"External pull-ups reach SDA GPIO{sda_pin} and SCL GPIO{scl_pin}."
    if sda:
        return (
            f"SDA reaches GPIO{sda_pin}, but SCL does not reach GPIO{scl_pin}. "
            "Check the SCL conductor and sensor power/ground."
        )
    if scl:
        return (
            f"SCL reaches GPIO{scl_pin}, but SDA does not reach GPIO{sda_pin}. "
            "Check the SDA conductor and sensor power/ground."
        )
    return (
        f"Neither external pull-up reaches SDA GPIO{sda_pin}/SCL GPIO{scl_pin}. "
        "Check 3.3 V, ground, and both I2C conductors."
    )


def imu_configuration(packet: object) -> tuple[str, tuple[str, ...]]:
    """Return ``none``, ``single`` or ``dual`` plus the connected roles."""
    roles = connected_roles(packet)
    mode = "dual" if len(roles) == 2 else "single" if len(roles) == 1 else "none"
    return mode, roles
