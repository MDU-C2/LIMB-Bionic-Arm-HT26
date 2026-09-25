"""Normalize the serial IMU packets used by AURORA and LIMB-HT25.

The current firmware names the physical sensor roles explicitly.  The legacy
LIMB-HT25 dual reader used ``imu1`` and ``imu2`` and Adafruit SI units, so this
module accepts both formats while exposing one stable representation to the GUI
and simulation.
"""

from __future__ import annotations

import math
from typing import Any


STANDARD_GRAVITY = 9.80665
ROLE_LABELS = {"shoulder": "Shoulder IMU", "wrist": "Wrist IMU"}


def _axes(value: object, scale: float = 1.0) -> dict[str, float] | None:
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


def normalize_imu(value: object, role: str) -> dict[str, Any]:
    """Return one IMU in g and degrees/second.

    ``accel``/``gyro`` is the old LIMB25 Adafruit packet (m/s^2 and rad/s).
    ``accel_g``/``gyro_dps`` is the explicit current packet.
    """
    if not isinstance(value, dict):
        return {"role": role, "connected": False, "error": "missing from packet"}

    explicit_units = "accel_g" in value or "gyro_dps" in value
    accel = _axes(
        value.get("accel_g") if explicit_units else value.get("accel"),
        1.0 if explicit_units else 1.0 / STANDARD_GRAVITY,
    )
    gyro = _axes(
        value.get("gyro_dps") if explicit_units else value.get("gyro"),
        1.0 if explicit_units else 180.0 / math.pi,
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
        # The single-IMU packet remains visible during the two-sensor hardware
        # transition.  It cannot drive the elbow fusion on its own.
        return {"wrist": normalize_imu(packet.get("imu"), "wrist")}
    return {}


def connected_roles(packet: object) -> tuple[str, ...]:
    """Return the connected physical IMU roles in deterministic order."""
    sensors = extract_imus(packet)
    return tuple(
        role for role in ("shoulder", "wrist")
        if sensors.get(role, {}).get("connected") is True
    )
