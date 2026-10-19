# Majd, Christian
"""Source-backed command and physical limits for the LIMB robot arm."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class JointLimit:
    """Position and motion limits for one actuator, in degrees."""

    lower: float
    upper: float
    speed_positive: float
    speed_negative: float
    acceleration: float | None = None
    home: float = 0.0

    def clamp(self, angle: float) -> float:
        return max(self.lower, min(self.upper, angle))

    def speed(self, direction: float) -> float:
        return self.speed_positive if direction >= 0 else self.speed_negative


# The firmware accepts 90 degrees of shoulder travel. The assembled arm was
# reported to reach about 75 degrees, which is the limit used by the simulator.
FIRMWARE_COMMAND_LIMITS_DEG = {
    "shoulder_up_down": JointLimit(0.0, 90.0, 10.0, 20.0, 15.0),
    "shoulder_left_right": JointLimit(5.0, 40.0, 20.0, 10.0, 15.0, 5.0),
    "upper_arm_rotation": JointLimit(-60.0, 60.0, 40.0, 40.0, 20.0),
    "elbow_up_down": JointLimit(0.0, 60.0, 40.0, 40.0, 20.0),
    "lower_arm_rotation": JointLimit(0.0, 140.0, 100.0, 100.0),
}

# Keep the command range and the reported travel separate.
HARDWARE_JOINT_LIMITS_DEG = {
    **FIRMWARE_COMMAND_LIMITS_DEG,
    "shoulder_up_down": JointLimit(0.0, 75.0, 10.0, 20.0, 15.0),
}

FINGER_LIMITS_DEG = {
    "thumb": JointLimit(0.0, 30.0, 40.0, 40.0),
    "index": JointLimit(0.0, 85.0, 40.0, 40.0),
    "middle": JointLimit(0.0, 90.0, 40.0, 40.0),
    "ring": JointLimit(0.0, 50.0, 40.0, 40.0),
    "pinky": JointLimit(0.0, 90.0, 120.0, 120.0),
}


def shared_grip_step(direction: float, dt: float, max_curl: float = 1.5) -> float:
    """Limit grip speed using the slowest finger servo."""
    normalized_speed = min(
        limit.speed(direction) / (limit.upper - limit.lower) for limit in FINGER_LIMITS_DEG.values()
    )
    return normalized_speed * max_curl * max(0.0, dt)


def finger_joint_angles_rad(curl: float) -> dict[str, float]:
    """Map hand curl to a stable open-to-grip pose for the imported hand."""
    if not math.isfinite(curl):
        raise ValueError("finger curl must be finite")
    curl = max(0.0, min(1.5, curl))
    progress = curl / 1.5
    angles = {
        # Move all thumb joints so it closes with the hand.
        "thumb_1": -0.10 - 0.38 * progress,
        "thumb_2": 1.10 * progress,
        "thumb_3": 0.60 * progress,
        "index_1": 1.15 * progress,
        "index_2": 0.95 * progress,
        "middle_1": 1.18 * progress,
        "middle_2": 0.98 * progress,
        # Keep these base joints nearly straight so the last two fingers do not drop.
        "ring_1": 0.10 * progress,
        "ring_2": 0.90 * progress,
        "ring_3": 0.55 * progress,
        "pinky_1": 0.10 * progress,
        "pinky_2": 0.85 * progress,
        "pinky_3": 0.50 * progress,
    }
    return angles


def _reverse(limit: JointLimit) -> JointLimit:
    """Reverse a hardware angle for an opposite URDF axis."""
    return JointLimit(
        lower=-limit.upper,
        upper=-limit.lower,
        speed_positive=limit.speed_negative,
        speed_negative=limit.speed_positive,
        acceleration=limit.acceleration,
        home=-limit.home,
    )


# Logical names used by the interactive right-arm model.
RIGHT_ARM_LIMITS_DEG = {
    "shoulder_x": HARDWARE_JOINT_LIMITS_DEG["upper_arm_rotation"],
    "shoulder_y": HARDWARE_JOINT_LIMITS_DEG["shoulder_up_down"],
    "shoulder_z": _reverse(HARDWARE_JOINT_LIMITS_DEG["shoulder_left_right"]),
    "elbow_x": _reverse(HARDWARE_JOINT_LIMITS_DEG["elbow_up_down"]),
    "wrist_rotation": HARDWARE_JOINT_LIMITS_DEG["lower_arm_rotation"],
}

# The maintained interactive scene uses the mirrored left-arm URDF. Shoulder
# abduction remains a negative URDF angle, while its elbow flexion is positive.
LEFT_ARM_HARDWARE_SIGN = {
    "shoulder_x": 1.0,
    "shoulder_y": 1.0,
    "shoulder_z": -1.0,
    "elbow_x": 1.0,
    "wrist_rotation": 1.0,
}
LEFT_ARM_LIMITS_DEG = {
    "shoulder_x": HARDWARE_JOINT_LIMITS_DEG["upper_arm_rotation"],
    "shoulder_y": HARDWARE_JOINT_LIMITS_DEG["shoulder_up_down"],
    "shoulder_z": _reverse(HARDWARE_JOINT_LIMITS_DEG["shoulder_left_right"]),
    "elbow_x": HARDWARE_JOINT_LIMITS_DEG["elbow_up_down"],
    "wrist_rotation": HARDWARE_JOINT_LIMITS_DEG["lower_arm_rotation"],
}


def max_velocity_rad_s(
    logical_name: str,
    limits: dict[str, JointLimit] = RIGHT_ARM_LIMITS_DEG,
) -> float:
    """Return the largest allowed speed for an interactive arm joint."""
    limit = limits[logical_name]
    return math.radians(max(limit.speed_positive, limit.speed_negative))


# DMP columns use physical actuator angles rather than right-arm URDF signs.
DMP_JOINT_NAMES = (
    "elbow_up_down",
    "shoulder_up_down",
    "shoulder_left_right",
    "upper_arm_rotation",
)
JOINT_LIMITS_RAD = np.deg2rad(
    np.array(
        [
            [
                HARDWARE_JOINT_LIMITS_DEG[name].lower,
                HARDWARE_JOINT_LIMITS_DEG[name].upper,
            ]
            for name in DMP_JOINT_NAMES
        ],
        dtype=float,
    )
)


def clamp_dmp_vector(q_rad: np.ndarray) -> np.ndarray:
    """Return a limited copy of a `(4,)` vector or `(..., 4)` trajectory."""
    q = np.asarray(q_rad, dtype=float)
    if q.ndim == 0 or q.shape[-1] != 4:
        raise ValueError(f"Expected final dimension 4, got {q.shape}")
    if not np.all(np.isfinite(q)):
        raise ValueError("Trajectory contains NaN or infinity")
    return np.clip(q, JOINT_LIMITS_RAD[:, 0], JOINT_LIMITS_RAD[:, 1])
