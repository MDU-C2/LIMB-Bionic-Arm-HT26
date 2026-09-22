"""Map camera pose recordings to safe LIMB simulation commands."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np


SIMULATION_ROOT = Path(__file__).resolve().parents[1]
if str(SIMULATION_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATION_ROOT))

from sim.joint_limits import (
    DMP_JOINT_NAMES,
    FINGER_LIMITS_DEG,
    HARDWARE_JOINT_LIMITS_DEG,
    clamp_dmp_vector,
)


class CameraArmMapper:
    """Estimate four physical arm angles from shoulder, elbow, and wrist points."""

    def __init__(
        self,
        fx: float = 615.0,
        fy: float = 615.0,
        cx: float = 320.0,
        cy: float = 240.0,
    ) -> None:
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.elbow_is_straight = True

    def _camera_point(self, value: object) -> np.ndarray | None:
        try:
            pixel_x, pixel_y, depth = np.asarray(value, dtype=float)
        except (TypeError, ValueError):
            return None
        if not np.all(np.isfinite((pixel_x, pixel_y, depth))) or depth <= 0:
            return None
        return np.array(
            [
                (pixel_x - self.cx) * depth / self.fx,
                -(pixel_y - self.cy) * depth / self.fy,
                depth,
            ]
        )

    def calculate(
        self,
        shoulder_value: object,
        elbow_value: object,
        wrist_value: object,
    ) -> np.ndarray | None:
        """Return elbow, shoulder flexion, abduction, and rotation in radians."""
        shoulder = self._camera_point(shoulder_value)
        elbow = self._camera_point(elbow_value)
        wrist = self._camera_point(wrist_value)
        if shoulder is None or elbow is None or wrist is None:
            return None

        upper_arm = elbow - shoulder
        forearm = wrist - elbow
        upper_length = np.linalg.norm(upper_arm)
        forearm_length = np.linalg.norm(forearm)
        if upper_length < 1e-6 or forearm_length < 1e-6:
            return None

        cosine = np.dot(upper_arm, forearm) / (upper_length * forearm_length)
        elbow_flexion = float(np.arccos(np.clip(cosine, -1.0, 1.0)))
        enter_bend = np.radians(12.0)
        exit_bend = np.radians(5.0)
        if self.elbow_is_straight and elbow_flexion < enter_bend:
            elbow_flexion = 0.0
        elif not self.elbow_is_straight and elbow_flexion < exit_bend:
            elbow_flexion = 0.0
            self.elbow_is_straight = True
        else:
            self.elbow_is_straight = False

        upper_x, upper_y, upper_z = upper_arm
        shoulder_flexion = np.arctan2(-upper_y, abs(upper_z) + 1e-6)
        shoulder_abduction = np.arctan2(abs(upper_x), abs(upper_y) + 1e-6)
        if shoulder_flexion < np.radians(5.0):
            shoulder_flexion = 0.0
        if shoulder_abduction < np.radians(8.0):
            shoulder_abduction = 0.0

        rotation = 0.0
        upper_arm_down = shoulder_flexion < np.radians(10.0)
        upper_arm_down &= shoulder_abduction < np.radians(10.0)
        if upper_arm_down and elbow_flexion > np.radians(50.0):
            rotation_limit = HARDWARE_JOINT_LIMITS_DEG["upper_arm_rotation"].upper
            rotation = np.clip((wrist[2] - elbow[2]) / 0.25, -1.0, 1.0)
            rotation *= np.radians(rotation_limit)

        return clamp_dmp_vector(
            np.array(
                [elbow_flexion, shoulder_flexion, shoulder_abduction, rotation],
                dtype=float,
            )
        )


class MotionSmoother:
    """Smooth camera angles and apply firmware speed and acceleration limits."""

    def __init__(self, alpha: float = 25.0) -> None:
        self.alpha = alpha
        self.beta = alpha / 4.0
        self.filtered: np.ndarray | None = None
        self.filter_velocity = np.zeros(4)
        self.command = np.radians(
            [HARDWARE_JOINT_LIMITS_DEG[name].home for name in DMP_JOINT_NAMES]
        )
        self.command_velocity = np.zeros(4)

    def step(self, target: np.ndarray, dt: float) -> np.ndarray:
        """Advance one safe command step toward a camera-derived target."""
        dt = float(np.clip(dt, 0.001, 0.1))
        target = clamp_dmp_vector(target)
        if self.filtered is None:
            self.filtered = self.command.copy()

        substeps = max(1, int(np.ceil(dt / 0.005)))
        sub_dt = dt / substeps
        for _ in range(substeps):
            acceleration = self.alpha * (
                self.beta * (target - self.filtered) - self.filter_velocity
            )
            self.filter_velocity += acceleration * sub_dt
            self.filtered += self.filter_velocity * sub_dt
        desired = clamp_dmp_vector(self.filtered)

        for index, name in enumerate(DMP_JOINT_NAMES):
            limit = HARDWARE_JOINT_LIMITS_DEG[name]
            delta = desired[index] - self.command[index]
            maximum = np.radians(limit.speed_positive if delta >= 0 else limit.speed_negative)
            if limit.acceleration is not None:
                acceleration = np.radians(limit.acceleration)
                lower = np.radians(limit.lower)
                upper = np.radians(limit.upper)
                boundary_distance = (
                    upper - self.command[index]
                    if delta >= 0
                    else self.command[index] - lower
                )
                stopping_distance = max(0.0, min(abs(delta), boundary_distance))
                acceleration_step = acceleration * dt
                stopping_speed = max(
                    0.0,
                    np.sqrt(acceleration_step**2 + 2.0 * acceleration * stopping_distance)
                    - acceleration_step,
                )
                requested_velocity = float(np.sign(delta) * min(maximum, stopping_speed))
                requested_velocity = float(
                    np.clip(
                        requested_velocity,
                        self.command_velocity[index] - acceleration_step,
                        self.command_velocity[index] + acceleration_step,
                    )
                )
            else:
                requested_velocity = float(np.clip(delta / dt, -maximum, maximum))
            self.command[index] += requested_velocity * dt
            self.command_velocity[index] = requested_velocity

        self.command = clamp_dmp_vector(self.command)
        return self.command.copy()


FINGER_LANDMARKS = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}


def estimate_finger_curls(hand: object) -> dict[str, float] | None:
    """Estimate scale-independent finger curl values from 21 hand landmarks."""
    if not isinstance(hand, list):
        return None
    points: dict[int, np.ndarray] = {}
    for value in hand:
        if not isinstance(value, dict) or "id" not in value:
            continue
        try:
            point = np.asarray([value["x"], value["y"]], dtype=float)
        except (KeyError, TypeError, ValueError):
            continue
        if np.all(np.isfinite(point)):
            points[int(value["id"])] = point
    if len(points) < 21:
        return None

    curls: dict[str, float] = {}
    for name, ids in FINGER_LANDMARKS.items():
        joint_angles = []
        for first, middle, last in zip(ids, ids[1:], ids[2:]):
            left = points[first] - points[middle]
            right = points[last] - points[middle]
            length = np.linalg.norm(left) * np.linalg.norm(right)
            if length > 1e-6:
                joint_angles.append(
                    np.arccos(np.clip(np.dot(left, right) / length, -1.0, 1.0))
                )
        if not joint_angles:
            return None
        mean_angle = float(np.mean(joint_angles))
        curls[name] = float(np.clip((np.pi - mean_angle) / np.radians(100.0), 0.0, 1.0))
    return curls


class FingerMotionLimiter:
    """Apply finger servo speed limits to normalized curl commands."""

    def __init__(self) -> None:
        self.command = {name: 0.0 for name in FINGER_LIMITS_DEG}

    def step(self, target: dict[str, float] | None, dt: float) -> dict[str, float]:
        if target is None:
            return self.command.copy()
        dt = max(0.001, float(dt))
        for name, limit in FINGER_LIMITS_DEG.items():
            requested = float(np.clip(target.get(name, self.command[name]), 0.0, 1.0))
            range_degrees = limit.upper - limit.lower
            speed = limit.speed_positive if requested >= self.command[name] else limit.speed_negative
            maximum_step = speed * dt / range_degrees
            self.command[name] += float(
                np.clip(requested - self.command[name], -maximum_step, maximum_step)
            )
        return self.command.copy()
