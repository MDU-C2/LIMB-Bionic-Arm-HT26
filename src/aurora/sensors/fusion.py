# Christian, Majd
"""Combine mounted IMU readings with camera angles for live arm control."""

from __future__ import annotations

import math
from typing import Any

from aurora.sensors.imu_twist import DEFAULT_TWIST_AXIS, ImuTwistEstimator

CONTROL_NAMES = (
    "elbow_flexion",
    "shoulder_flexion",
    "shoulder_abduction",
    "shoulder_rotation_proxy",
    "wrist_rotation",
)
FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")
REFERENCE_IMU_PERIOD_S = 0.02
MAX_IMU_PERIOD_S = 0.35
DEFAULT_GYRO_DEADZONE_DPS = 2.5
DEFAULT_GYRO_ABDUCTION_GAIN = 1.0
# A six-axis IMU has no absolute heading reference.  Its integrated gyro-Z
# value is useful through short camera gaps, but must not pull a valid absolute
# camera left/right angle away from the observed pose.
CAMERA_ABSOLUTE_CHANNELS = frozenset(("shoulder_abduction",))


def fresh_imu_angles(angles, roles):
    """Only publish angles supported by the currently fresh sensor roles."""
    if not angles or "shoulder" not in roles:
        return None
    result = dict(angles)
    if "wrist" not in roles:
        result.pop("elbow_flexion", None)
        result.pop("wrist_rotation", None)
    return result or None


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
    wrist sensor is monitor-only: treating a forearm sensor as an
    upper-arm sensor makes every elbow bend look like shoulder motion.
    Signed mounting axes remap both vectors to a common frame. The default Y
    axis must run along each limb; twist waits for half a second of rest.

    ``alpha`` is the retained fraction at the 50 Hz reference rate. It is
    converted to a time constant so the response stays consistent when the
    sensor sample rate changes.
    ``alpha=0`` keeps deterministic tests and diagnostics unsmoothed.
    """

    def __init__(
        self,
        alpha: float = 0.85,
        gyro_deadzone_dps: float = DEFAULT_GYRO_DEADZONE_DPS,
        gyro_abduction_gain: float = DEFAULT_GYRO_ABDUCTION_GAIN,
        shoulder_twist_axis: str = DEFAULT_TWIST_AXIS,
        wrist_twist_axis: str = DEFAULT_TWIST_AXIS,
        side: str = "left",
    ) -> None:
        if side not in ("left", "right"):
            raise ValueError("Choose left or right arm")
        self.side = self.initial_side = side
        self.alpha = min(0.999, max(0.0, float(alpha)))
        self.accel_time_constant_s = (
            -REFERENCE_IMU_PERIOD_S / math.log(self.alpha) if self.alpha > 0.0 else 0.0
        )
        self.gyro_deadzone_dps = max(0.0, float(gyro_deadzone_dps))
        self.twist = ImuTwistEstimator(
            shoulder_twist_axis, wrist_twist_axis, self.gyro_deadzone_dps
        )
        # Human angular rates stay in degrees/s. Robot travel belongs to mapping.
        self.gyro_abduction_gain = max(0.0, float(gyro_abduction_gain))
        self.initial_twist_axes = dict(self.twist.axes)
        self.clear_alignment()

    def clear_alignment(self):
        self.side = self.initial_side
        self.alignment_profile = None
        self.sensor_rotations, self.sensor_biases = {}, {}
        self.twist.axes = dict(self.initial_twist_axes)
        self.reference_angles = {}
        self.reset_calibration()

    def set_alignment(self, profile) -> None:
        """Apply a validated sensor-to-segment fit to the live/offline estimator."""
        import numpy as np

        from aurora.sensors.arm_alignment import METHOD, ROLES, rotation_matrix

        if (
            profile.get("format_version") != 1
            or profile.get("method") != METHOD
            or profile.get("status") != "passed"
            or profile.get("matrix_direction") != "segment_from_sensor"
        ):
            raise ValueError("A passed version-1 sensor-to-segment alignment is required")
        rotations = {role: rotation_matrix(profile["sensor_to_segment"][role]) for role in ROLES}
        biases = {role: np.asarray(profile["gyro_bias_dps"][role], dtype=float) for role in ROLES}
        if any(value.shape != (3,) or not np.isfinite(value).all() for value in biases.values()):
            raise ValueError("Alignment gyro biases must be finite three-vectors")
        references = {
            role: rotation_matrix(profile["camera_from_segment_at_reference"][role])
            for role in ROLES
        }
        elbow = float(profile["reference_elbow_flexion_deg"])
        if profile.get("side") not in ("left", "right"):
            raise ValueError("Alignment must identify the selected arm")
        if not math.isfinite(elbow) or not 0 <= elbow <= 180:
            raise ValueError("Invalid anatomical elbow reference")
        self.sensor_rotations, self.sensor_biases = rotations, biases
        self.alignment_profile = profile
        self.side = profile["side"]
        self.alignment_references = references
        self.twist.axes = dict.fromkeys(ROLES, "y")
        self.reset_calibration()

    def reset_calibration(self, reference_angles: object = None) -> None:
        if reference_angles is not None and self.alignment_profile is None:
            self.reference_angles = camera_control_angles(reference_angles)
        if self.alignment_profile is not None:
            self.reference_angles = {
                "shoulder_flexion": 0.0,
                "shoulder_abduction": 0.0,
                "elbow_flexion": self.alignment_profile["reference_elbow_flexion_deg"],
                "wrist_rotation": 0.0,
            }
        self.twist.reset(
            shoulder_angle=-self.reference_angles.get("shoulder_rotation_proxy", 0.0),
            wrist_angle=self.reference_angles.get("wrist_rotation", 0.0),
        )
        self.filtered_accel = {}
        self.tilt_offsets = {}
        self.gyro_bias_z = {}
        self.yaw_delta = {}
        self.elbow_zero = 0.0
        self.active_roles = ()
        self.camera_offsets = {}
        self.last_angles = {}
        if self.alignment_profile is not None:
            # Restore the measured reference geometry, not the current pose.
            up_camera = -self.alignment_references["shoulder"][:, 1]
            for role, orientation in self.alignment_references.items():
                gravity = orientation.T @ up_camera
                self.tilt_offsets[role] = self._mounted_tilt(gravity)
                self.gyro_bias_z[role], self.yaw_delta[role] = 0.0, 0.0
            self.active_roles = ("shoulder", "wrist")
            self.twist.bias = dict.fromkeys(self.active_roles, (0.0, 0.0, 0.0))
            self.twist.dual_ready = True

    @staticmethod
    def _mounted_tilt(accel: tuple[float, float, float]) -> float:
        """Calculate tilt from the mounted sensor's Y and Z axes."""
        _ax, ay, az = accel
        return math.degrees(math.atan2(ay, az))

    @staticmethod
    def _angle_delta(value: float, zero: float) -> float:
        """Small signed angular difference, safe across the +/-180 wrap."""
        return (value - zero + 180.0) % 360.0 - 180.0

    def _filter_accel(
        self,
        role: str,
        accel: tuple[float, float, float],
        dt: float,
    ) -> tuple[float, float, float]:
        previous = self.filtered_accel.get(role)
        if previous is None:
            filtered = accel
        else:
            retained = (
                math.exp(-dt / self.accel_time_constant_s)
                if self.accel_time_constant_s > 0.0
                else 0.0
            )
            filtered = tuple(
                retained * old + (1.0 - retained) * new
                for old, new in zip(previous, accel, strict=False)
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
        self,
        sensors: dict[str, dict[str, Any]],
        dt: float,
        *,
        sample_dt_s: dict[str, float] | None = None,
    ) -> dict[str, float] | None:
        if any(
            isinstance(sensor, dict) and sensor.get("gyro_saturated") for sensor in sensors.values()
        ):
            return dict(self.last_angles) or None
        if not math.isfinite(dt) or dt <= 0 or dt > 0.5:
            self.reset_calibration()
            dt = 0.01
        # Use elapsed packet time rather than assuming a sample rate.  The cap
        # rejects reconnection gaps without halving legitimate 5 Hz gyro motion.
        dt = min(MAX_IMU_PERIOD_S, max(0.001, float(dt)))
        role_dt = {
            role: dt if sample_dt_s is None else sample_dt_s.get(role, 0.0)
            for role in ("shoulder", "wrist")
        }
        tilts: dict[str, float] = {}
        gyro_z: dict[str, float] = {}
        gyros: dict[str, tuple[float, float, float]] = {}
        for role in ("shoulder", "wrist"):
            sensor = sensors.get(role)
            if not isinstance(sensor, dict) or sensor.get("connected") is not True:
                continue
            accel = _vector(sensor.get("accel_g"))
            gyro = _vector(sensor.get("gyro_dps"))
            if accel is None or gyro is None:
                continue
            if role in self.sensor_rotations:
                matrix = self.sensor_rotations[role]
                accel = tuple(matrix @ accel)
                gyro = tuple(matrix @ (gyro - self.sensor_biases[role]))
            else:
                accel = self.twist.remap(role, accel)
                gyro = self.twist.remap(role, gyro)
            filtered = (
                self._filter_accel(role, accel, role_dt[role])
                if role_dt[role] > 0
                else self.filtered_accel.get(role, accel)
            )
            tilts[role] = self._mounted_tilt(filtered)
            gyro_z[role] = gyro[2]
            gyros[role] = gyro

        if not tilts:
            self.twist.update({}, dt)
            self.active_roles = ()
            return None
        active_roles = tuple(role for role in ("shoulder", "wrist") if role in tilts)
        new_roles = tuple(role for role in active_roles if role not in self.active_roles)
        for role in new_roles:
            self.tilt_offsets[role] = tilts[role]
            self.gyro_bias_z[role] = gyro_z[role]
            self.yaw_delta[role] = 0.0

        for role in active_roles:
            if role not in new_roles:
                self.yaw_delta[role] = self.yaw_delta.get(role, 0.0) + (
                    self._yaw_rate(gyro_z[role], role) * self.gyro_abduction_gain * role_dt[role]
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
        relative_bend = self.reference_angles.get("elbow_flexion", 0.0) + (
            (wrist_tilt - shoulder_tilt) - self.elbow_zero
            if wrist_tilt is not None and shoulder_tilt is not None
            else 0.0
        )
        rotations = self.twist.update(gyros, dt, math.radians(relative_bend), sample_dt_s=role_dt)
        if shoulder_tilt is None:
            return {}

        result = {
            "shoulder_flexion": self.reference_angles.get("shoulder_flexion", 0.0) + shoulder_tilt,
            # Preserve the gyro sign.  Taking abs() here made both real motion
            # directions command the robot outward/left.
            "shoulder_abduction": (
                self.reference_angles.get("shoulder_abduction", 0.0)
                + (1.0 if self.side == "left" else -1.0) * self.yaw_delta.get("shoulder", 0.0)
            ),
        }
        if wrist_tilt is not None:
            result["elbow_flexion"] = relative_bend
        result.update(rotations)
        for name, offset in self.camera_offsets.items():
            if name in result:
                result[name] += offset
        self.last_angles = camera_control_angles(result)
        return dict(self.last_angles)

    def correct_angles(self, camera_angles: object, weight: float) -> dict[str, float]:
        """Anchor the IMU prediction once per fresh camera observation."""
        measured = camera_control_angles(camera_angles)
        weight = min(1.0, max(0.0, weight))
        referenced = self.referenced_channels()
        for name in ("shoulder_flexion", "elbow_flexion", "shoulder_abduction"):
            if weight > 0 and name in measured and name in self.last_angles:
                gain = weight if name in referenced else 1.0
                correction = gain * (measured[name] - self.last_angles[name])
                self.camera_offsets[name] = self.camera_offsets.get(name, 0.0) + correction
                self.last_angles[name] += correction
        if "shoulder_rotation_proxy" in measured:
            rotation = self.correct_rotation(measured["shoulder_rotation_proxy"], weight)
            if rotation is not None:
                self.last_angles["shoulder_rotation_proxy"] = rotation
        return dict(self.last_angles)

    def referenced_channels(self) -> set[str]:
        """Name channels anchored by a human pose or fitted segment reference."""
        if self.alignment_profile is not None:
            return set(CONTROL_NAMES)
        names = set(self.reference_angles) | set(self.camera_offsets)
        if self.twist.camera_aligned:
            names.add("shoulder_rotation_proxy")
        return names

    def correct_rotation(self, camera_proxy: float, weight: float) -> float | None:
        """Anchor axial gyro state to a fresh camera target; preserve relative wrist twist."""
        if not self.twist.correct_shoulder(camera_proxy, weight):
            return None
        return -self.twist.shoulder_angle


def camera_control_angles(value: object) -> dict[str, float]:
    """Keep measured angles intact until alignment and fusion are finished."""
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
        result[name] = sample
    return result


def fuse_control_angles(
    imu_angles: dict[str, float] | None,
    camera_angles: dict[str, float] | None,
    camera_weight: float = 0.25,
    *,
    camera_corrected: bool = False,
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
        if camera_corrected and camera_value is not None and weight == 1.0:
            result[name] = camera_value
        elif camera_corrected and imu_value is not None:
            # The camera already corrected this state. Do not blend its held
            # observation again and suppress motion between camera frames.
            result[name] = imu_value
        elif name == "shoulder_rotation_proxy" and imu_value is not None:
            # Fresh camera observations already correct the integrated gyro state.
            # Mixing a held camera angle again would suppress motion between frames.
            result[name] = imu_value
        elif camera_value is not None and name in CAMERA_ABSOLUTE_CHANNELS and weight > 0.0:
            result[name] = camera_value
        elif imu_value is not None and camera_value is not None:
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
        dt = min(1.0, max(0.001, float(dt)))
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
        result["elbow_x"] = min(60.0, max(0.0, fused["elbow_flexion"]))
    if "shoulder_flexion" in fused:
        result["shoulder_y"] = min(75.0, max(0.0, 90.0 - fused["shoulder_flexion"]))
    if "shoulder_abduction" in fused:
        result["shoulder_z"] = min(-5.0, max(-40.0, -fused["shoulder_abduction"]))
    if "shoulder_rotation_proxy" in fused:
        # Camera axial-positive is a medial/right movement for the tracked
        # left arm.  The mirrored left-arm URDF uses the opposite joint sign.
        result["shoulder_x"] = min(60.0, max(-60.0, -fused["shoulder_rotation_proxy"]))
    if "wrist_rotation" in fused:
        result["wrist_rotation"] = min(140.0, max(0.0, fused["wrist_rotation"]))
    return result or None


def retarget_reference(angles, human_reference, robot_reference):
    """Shift command references explicitly while preserving anatomical estimates."""
    measured = camera_control_angles(angles)
    human = camera_control_angles(human_reference)
    robot = camera_control_angles(robot_reference)
    return {
        name: value + robot[name] - human[name] if name in robot and name in human else value
        for name, value in measured.items()
    }


def anatomical_reference_angles(joints: dict[str, float]) -> dict[str, float]:
    """Invert the joint mapping so calibration preserves the displayed pose."""
    return camera_control_angles(
        {
            "shoulder_flexion": 90.0 - joints["shoulder_y"],
            "shoulder_abduction": -joints["shoulder_z"],
            "shoulder_rotation_proxy": -joints["shoulder_x"],
            "elbow_flexion": joints["elbow_x"],
            "wrist_rotation": joints["wrist_rotation"],
        }
    )
