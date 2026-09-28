"""One/two-IMU orientation and camera correction for live arm control.

This keeps the useful parts of LIMB-HT25's tested two-IMU simulator: the
mounted sensor's Y/Z gravity tilt drives shoulder elevation, gyro Z drives the
shoulder base, and elbow flexion is the forearm tilt relative to the upper arm.
Camera angles provide the absolute-pose correction.
"""

from __future__ import annotations

import math
from typing import Any


CONTROL_NAMES = (
    "elbow_flexion",
    "shoulder_flexion",
    "shoulder_abduction",
    "shoulder_rotation_proxy",
)
FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")


def _vector(value: object) -> tuple[float, float, float] | None:
    if not isinstance(value, dict):
        return None
    try:
        result = tuple(float(value[axis]) for axis in ("x", "y", "z"))
    except (KeyError, TypeError, ValueError):
        return None
    return result if all(math.isfinite(sample) for sample in result) else None


class DualImuArmEstimator:
    """Convert whichever arm IMUs are connected into calibrated joint angles.

    A shoulder sensor supplies elevation and left/right movement. With both
    sensors, their relative mounted tilt also supplies elbow flexion. A lone
    wrist sensor is deliberately monitor-only: treating a forearm sensor as an
    upper-arm sensor makes every elbow bend look like shoulder motion.

    ``alpha`` is the retained fraction in the acceleration EMA. The default
    0.85 is the same 15% new-sample smoothing used by HT25's two-IMU simulator;
    alpha=0 keeps deterministic tests and diagnostics unsmoothed.
    """

    def __init__(self, alpha: float = 0.85, gyro_deadzone_dps: float = 1.0) -> None:
        self.alpha = min(0.999, max(0.0, float(alpha)))
        self.gyro_deadzone_dps = max(0.0, float(gyro_deadzone_dps))
        self.filtered_accel: dict[str, tuple[float, float, float]] = {}
        self.tilt_offsets: dict[str, float] = {}
        self.gyro_bias_z: dict[str, float] = {}
        self.yaw_delta: dict[str, float] = {}
        self.elbow_zero = 0.0
        self.active_roles: tuple[str, ...] = ()

    def reset_calibration(self) -> None:
        self.filtered_accel = {}
        self.tilt_offsets = {}
        self.gyro_bias_z = {}
        self.yaw_delta = {}
        self.elbow_zero = 0.0
        self.active_roles = ()

    @staticmethod
    def _mounted_tilt(accel: tuple[float, float, float]) -> float:
        """Return HT25's installed-sensor Y/Z tilt in degrees."""
        _ax, ay, az = accel
        return math.degrees(math.atan2(ay, az))

    @staticmethod
    def _angle_delta(value: float, zero: float) -> float:
        """Small signed angular difference, safe across the +/-180 wrap."""
        return (value - zero + 180.0) % 360.0 - 180.0

    def _filter_accel(
        self, role: str, accel: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        previous = self.filtered_accel.get(role)
        if previous is None:
            filtered = accel
        else:
            filtered = tuple(
                self.alpha * old + (1.0 - self.alpha) * new
                for old, new in zip(previous, accel)
            )
        self.filtered_accel[role] = filtered
        return filtered

    def _yaw_rate(self, gyro_z: float, role: str) -> float:
        corrected = gyro_z - self.gyro_bias_z.get(role, gyro_z)
        if abs(corrected) <= self.gyro_deadzone_dps:
            return 0.0
        return math.copysign(
            abs(corrected) - self.gyro_deadzone_dps,
            corrected,
        )

    def update(
        self, sensors: dict[str, dict[str, Any]], dt: float
    ) -> dict[str, float] | None:
        tilts: dict[str, float] = {}
        gyro_z: dict[str, float] = {}
        for role in ("shoulder", "wrist"):
            sensor = sensors.get(role)
            if not isinstance(sensor, dict) or sensor.get("connected") is not True:
                continue
            accel = _vector(sensor.get("accel_g"))
            gyro = _vector(sensor.get("gyro_dps"))
            if accel is None or gyro is None:
                continue
            tilts[role] = self._mounted_tilt(self._filter_accel(role, accel))
            gyro_z[role] = gyro[2]

        if not tilts:
            self.active_roles = ()
            return None
        active_roles = tuple(
            role for role in ("shoulder", "wrist") if role in tilts
        )
        new_roles = tuple(role for role in active_roles if role not in self.active_roles)
        for role in new_roles:
            self.tilt_offsets[role] = tilts[role]
            self.gyro_bias_z[role] = gyro_z[role]
            self.yaw_delta[role] = 0.0

        dt = min(0.1, max(0.001, float(dt)))
        for role in active_roles:
            if role not in new_roles:
                self.yaw_delta[role] = self.yaw_delta.get(role, 0.0) + (
                    self._yaw_rate(gyro_z[role], role) * dt
                )

        shoulder_tilt = (
            self._angle_delta(tilts["shoulder"], self.tilt_offsets["shoulder"])
            if "shoulder" in tilts
            else None
        )
        wrist_tilt = (
            self._angle_delta(tilts["wrist"], self.tilt_offsets["wrist"])
            if "wrist" in tilts
            else None
        )

        if "wrist" in new_roles and shoulder_tilt is not None:
            # Adding the second sensor must not make a moving shoulder appear
            # as an instant elbow bend.
            self.elbow_zero = (wrist_tilt or 0.0) - shoulder_tilt
        elif "shoulder" in new_roles and wrist_tilt is not None:
            self.elbow_zero = wrist_tilt - (shoulder_tilt or 0.0)

        self.active_roles = active_roles
        if shoulder_tilt is None:
            return {}

        result = {
            "shoulder_flexion": abs(shoulder_tilt),
            # HT25 maps integrated upper-arm gyro Z to the shoulder base.
            "shoulder_abduction": abs(self.yaw_delta.get("shoulder", 0.0)),
        }
        if wrist_tilt is not None:
            result["elbow_flexion"] = abs(
                (wrist_tilt - shoulder_tilt) - self.elbow_zero
            )
        # A six-axis IMU cannot provide drift-free humeral axial rotation.
        # Keep that channel camera-only instead of driving the wrong joint.
        return camera_control_angles(result)


def camera_control_angles(value: object) -> dict[str, float]:
    """Keep finite camera angles and enforce the built arm's usable ranges."""
    if not isinstance(value, dict):
        return {}
    result: dict[str, float] = {}
    for name in CONTROL_NAMES:
        sample = value.get(name)
        if isinstance(sample, bool) or not isinstance(sample, (int, float)):
            continue
        sample = float(sample)
        if not math.isfinite(sample):
            continue
        if name == "elbow_flexion":
            sample = min(60.0, max(0.0, sample))
        elif name == "shoulder_flexion":
            sample = min(90.0, max(0.0, sample))
        elif name == "shoulder_abduction":
            # Camera geometry defines outward as positive for either selected
            # arm.  Do not mirror an across-body/adduction estimate into an
            # outward command.  IMU mounting polarity is handled before this
            # normalization by the estimator's magnitude convention.
            sample = min(40.0, max(0.0, sample))
        elif name == "shoulder_rotation_proxy":
            sample = min(60.0, max(-60.0, sample))
        result[name] = sample
    return result


def fuse_control_angles(
    imu_angles: dict[str, float] | None,
    camera_angles: dict[str, float] | None,
    camera_weight: float = 0.25,
) -> dict[str, float] | None:
    """Fuse fast IMU motion with camera drift correction in joint-angle space."""
    imu = camera_control_angles(imu_angles)
    camera = camera_control_angles(camera_angles)
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
    return camera_control_angles(result) or None


class ControlAngleSmoother:
    """Low-pass camera targets without inventing values for missing channels."""

    def __init__(self, time_constant_s: float = 0.10) -> None:
        self.time_constant_s = max(0.001, float(time_constant_s))
        self.values: dict[str, float] = {}

    def reset(self) -> None:
        self.values = {}

    def update(self, angles: object, dt: float) -> dict[str, float]:
        measured = camera_control_angles(angles)
        if not measured:
            return {}
        dt = min(0.25, max(0.001, float(dt)))
        new_weight = 1.0 - math.exp(-dt / self.time_constant_s)
        result: dict[str, float] = {}
        for name, target in measured.items():
            previous = self.values.get(name, target)
            filtered = previous + new_weight * (target - previous)
            self.values[name] = filtered
            result[name] = filtered
        return camera_control_angles(result)


def camera_hand_curl(value: object) -> float | None:
    """Convert MediaPipe finger-flexion proxies to one stable 0..1 grip value."""
    if not isinstance(value, dict):
        return None
    samples = []
    for name in FINGER_NAMES:
        sample = value.get(name)
        if isinstance(sample, bool) or not isinstance(sample, (int, float)):
            continue
        sample = float(sample)
        if math.isfinite(sample):
            samples.append(min(1.0, max(0.0, sample / 90.0)))
    if len(samples) < 3:
        return None
    return sum(samples) / len(samples)


def interactive_control_targets(angles: object) -> dict[str, float] | None:
    """Map fused physical arm angles onto the interactive left-arm model.

    Camera/IMU shoulder flexion is anatomical: zero is arm-down and 90 degrees
    is horizontal-forward.  The imported CAD uses the opposite zero: zero is
    horizontal and positive motion lowers the arm.  The remaining channels
    differ only by their mirrored joint signs.
    """
    fused = camera_control_angles(angles)
    if not fused:
        return None
    result: dict[str, float] = {}
    if "elbow_flexion" in fused:
        result["elbow_x"] = fused["elbow_flexion"]
    if "shoulder_flexion" in fused:
        result["shoulder_y"] = 90.0 - fused["shoulder_flexion"]
    if "shoulder_abduction" in fused:
        result["shoulder_z"] = -fused["shoulder_abduction"]
    if "shoulder_rotation_proxy" in fused:
        result["shoulder_x"] = fused["shoulder_rotation_proxy"]
    return result or None
