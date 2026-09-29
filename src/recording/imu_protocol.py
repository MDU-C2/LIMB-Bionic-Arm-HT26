"""Normalize the serial IMU packets used by AURORA and LIMB-HT25.

The current firmware names the physical sensor roles explicitly.  The legacy
LIMB-HT25 dual reader used ``imu1`` and ``imu2`` and Adafruit SI units, so this
module accepts both formats while exposing one stable representation to the GUI
and simulation.
"""

from __future__ import annotations

import json
import math
import re
from typing import Any


STANDARD_GRAVITY = 9.80665
ROLE_LABELS = {"shoulder": "Shoulder IMU", "wrist": "Wrist IMU"}
LEGACY_ACCEL_G_PER_LSB = 1.0 / 4096.0
LEGACY_GYRO_DPS_PER_LSB = 1.0 / 65.5


class ImuStreamDecoder:
    """Decode current JSON or the older line-oriented two-IMU serial stream."""

    _identity = re.compile(
        r"^IMU\s*(?P<sensor>[12])\s+WHO_AM_I:\s*0x(?P<value>[0-9a-fA-F]+)$"
    )
    _sample = re.compile(
        r"^IMU\s*(?P<sensor>[12])\s+(?P<kind>ACC|GYRO):\s*"
        r"(?P<x>-?\d+)\s+(?P<y>-?\d+)\s+(?P<z>-?\d+)$"
    )

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.identities: dict[int, int] = {}
        self.samples: dict[int, dict[str, tuple[int, int, int]]] = {}
        self.sequence = 0

    def feed(self, text: str) -> dict[str, Any] | None:
        """Consume one serial line and return a complete packet when available."""
        line = text.strip()
        if line.startswith("{"):
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                value = None
            if isinstance(value, dict):
                return value

        identity = self._identity.match(line)
        if identity is not None:
            self.identities[int(identity.group("sensor"))] = int(
                identity.group("value"), 16
            )
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

        packet = self._packet()
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        return packet

    def _packet(self) -> dict[str, Any]:
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
            connected = identity in (None, 0x6C) and has_signal
            sensor: dict[str, Any] = {
                "connected": connected,
                "model": "LSM6DSO32",
                "address": address,
                "serial_format": "legacy_text",
            }
            if connected and accel is not None and gyro is not None:
                sensor["accel_g"] = {
                    axis: raw_value * LEGACY_ACCEL_G_PER_LSB
                    for axis, raw_value in zip(("x", "y", "z"), accel)
                }
                sensor["gyro_dps"] = {
                    axis: raw_value * LEGACY_GYRO_DPS_PER_LSB
                    for axis, raw_value in zip(("x", "y", "z"), gyro)
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
        value = dict(zip(("x", "y", "z"), value[:3]))
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
    for key in ("temperature_c", "sda_pin", "scl_pin", "error"):
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
        return {
            role: normalize_imu(imus.get(role), role)
            for role in ("shoulder", "wrist")
        }

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
        role for role in ("shoulder", "wrist")
        if sensors.get(role, {}).get("connected") is True
    )


def imu_configuration(packet: object) -> tuple[str, tuple[str, ...]]:
    """Return ``none``, ``single`` or ``dual`` plus the connected roles."""
    roles = connected_roles(packet)
    mode = "dual" if len(roles) == 2 else "single" if len(roles) == 1 else "none"
    return mode, roles
