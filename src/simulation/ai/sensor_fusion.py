"""Dual-IMU orientation and camera correction for live arm control.

This keeps the useful parts of LIMB-HT25's ``sensor_logic.py``: complementary
accel/gyro orientation, a calibrated upper-arm reference, and elbow flexion
from the wrist-versus-shoulder angle.  Camera angles provide the low-frequency
correction, matching the old repository's IMU/vision complementary approach.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


CONTROL_NAMES = (
    "elbow_flexion",
    "shoulder_flexion",
    "shoulder_abduction",
    "shoulder_rotation_proxy",
)


def _vector(value: object) -> tuple[float, float, float] | None:
    if not isinstance(value, dict):
        return None
    try:
        result = tuple(float(value[axis]) for axis in ("x", "y", "z"))
    except (KeyError, TypeError, ValueError):
        return None
    return result if all(math.isfinite(sample) for sample in result) else None


@dataclass
class Orientation:
    roll: float
    pitch: float
    yaw: float


class ComplementaryOrientation:
    """Estimate Euler orientation in degrees from one six-axis IMU."""

    def __init__(self, alpha: float = 0.98) -> None:
        self.alpha = min(1.0, max(0.0, alpha))
        self.value: Orientation | None = None

    def reset(self) -> None:
        self.value = None

    def update(
        self,
        accel_g: tuple[float, float, float],
        gyro_dps: tuple[float, float, float],
        dt: float,
    ) -> Orientation:
        ax, ay, az = accel_g
        gx, gy, gz = gyro_dps
        accel_pitch = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
        accel_roll = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
        dt = min(0.1, max(0.001, float(dt)))
        if self.value is None:
            self.value = Orientation(accel_roll, accel_pitch, 0.0)
        else:
            self.value = Orientation(
                self.alpha * (self.value.roll + gx * dt) + (1.0 - self.alpha) * accel_roll,
                self.alpha * (self.value.pitch + gy * dt) + (1.0 - self.alpha) * accel_pitch,
                self.value.yaw + gz * dt,
            )
        return self.value


class DualImuArmEstimator:
    """Convert shoulder and wrist IMUs into calibrated arm joint angles."""

    def __init__(self, alpha: float = 0.98) -> None:
        self.filters = {
            "shoulder": ComplementaryOrientation(alpha),
            "wrist": ComplementaryOrientation(alpha),
        }
        self.offsets: dict[str, Orientation] | None = None

    def reset_calibration(self) -> None:
        self.offsets = None
        for orientation_filter in self.filters.values():
            orientation_filter.reset()

    def update(
        self, sensors: dict[str, dict[str, Any]], dt: float
    ) -> dict[str, float] | None:
        orientations: dict[str, Orientation] = {}
        for role in ("shoulder", "wrist"):
            sensor = sensors.get(role)
            if not isinstance(sensor, dict) or sensor.get("connected") is not True:
                return None
            accel = _vector(sensor.get("accel_g"))
            gyro = _vector(sensor.get("gyro_dps"))
            if accel is None or gyro is None:
                return None
            orientations[role] = self.filters[role].update(accel, gyro, dt)

        if self.offsets is None:
            self.offsets = {
                role: Orientation(value.roll, value.pitch, value.yaw)
                for role, value in orientations.items()
            }

        shoulder = orientations["shoulder"]
        wrist = orientations["wrist"]
        shoulder_zero = self.offsets["shoulder"]
        wrist_zero = self.offsets["wrist"]
        shoulder_roll = shoulder.roll - shoulder_zero.roll
        shoulder_pitch = shoulder.pitch - shoulder_zero.pitch
        shoulder_yaw = shoulder.yaw - shoulder_zero.yaw
        wrist_pitch = wrist.pitch - wrist_zero.pitch

        return {
            "elbow_flexion": abs(wrist_pitch - shoulder_pitch),
            "shoulder_flexion": abs(shoulder_pitch),
            "shoulder_abduction": abs(shoulder_roll),
            "shoulder_rotation_proxy": shoulder_yaw,
        }


def camera_control_angles(value: object) -> dict[str, float]:
    """Keep finite camera angles that map directly to the four DMP joints."""
    if not isinstance(value, dict):
        return {}
    result: dict[str, float] = {}
    for name in CONTROL_NAMES:
        sample = value.get(name)
        if isinstance(sample, bool) or not isinstance(sample, (int, float)):
            continue
        sample = float(sample)
        if math.isfinite(sample):
            result[name] = sample
    return result


def fuse_control_angles(
    imu_angles: dict[str, float] | None,
    camera_angles: dict[str, float] | None,
    camera_weight: float = 0.25,
) -> dict[str, float] | None:
    """Fuse fast IMU motion with camera drift correction in joint-angle space."""
    imu = imu_angles or {}
    camera = camera_angles or {}
    if not imu and not camera:
        return None
    weight = min(1.0, max(0.0, float(camera_weight)))
    result: dict[str, float] = {}
    for name in CONTROL_NAMES:
        imu_value = imu.get(name)
        camera_value = camera.get(name)
        if imu_value is not None and camera_value is not None:
            result[name] = (1.0 - weight) * imu_value + weight * camera_value
        elif imu_value is not None:
            result[name] = imu_value
        elif camera_value is not None:
            result[name] = camera_value
    return result or None


def interactive_control_targets(angles: object) -> dict[str, float] | None:
    """Map fused physical arm angles onto the interactive right-arm model.

    The signs follow the existing right-arm URDF mapping: elbow flexion and
    shoulder abduction use negative logical angles, while flexion and upper-arm
    rotation retain their physical signs.
    """
    fused = camera_control_angles(angles)
    if not fused:
        return None
    mapping = {
        "elbow_flexion": ("elbow_x", -1.0),
        "shoulder_flexion": ("shoulder_y", 1.0),
        "shoulder_abduction": ("shoulder_z", -1.0),
        "shoulder_rotation_proxy": ("shoulder_x", 1.0),
    }
    return {
        logical_name: fused[source_name] * sign
        for source_name, (logical_name, sign) in mapping.items()
        if source_name in fused
    }
