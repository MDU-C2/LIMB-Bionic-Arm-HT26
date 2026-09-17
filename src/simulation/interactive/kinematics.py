"""Reachability and two-link inverse kinematics for the LIMB task scene."""

from __future__ import annotations

import math
from typing import Sequence

from sim.joint_limits import RIGHT_ARM_LIMITS_DEG
from sim.robot_model import FOREARM_LENGTH_M, RIGHT_GRIP_OFFSET_M, UPPER_ARM_LENGTH_M


SHOULDER_ORIGIN = (0.0, 0.0, 0.0)
HAND_GRIP_LENGTH_M = RIGHT_GRIP_OFFSET_M[0]
LOWER_CHAIN_LENGTH_M = FOREARM_LENGTH_M + HAND_GRIP_LENGTH_M

MAX_REACH_M = UPPER_ARM_LENGTH_M + LOWER_CHAIN_LENGTH_M
ELBOW_FLEXION_LIMIT = RIGHT_ARM_LIMITS_DEG["elbow_x"]
MIN_ELBOW_ANGLE_RAD = math.radians(180.0 + ELBOW_FLEXION_LIMIT.lower)
MIN_REACH_M = math.sqrt(
    UPPER_ARM_LENGTH_M**2
    + LOWER_CHAIN_LENGTH_M**2
    - 2.0 * UPPER_ARM_LENGTH_M * LOWER_CHAIN_LENGTH_M * math.cos(MIN_ELBOW_ANGLE_RAD)
)


def _target_components(target: Sequence[float]) -> tuple[float, float, float]:
    """Validate and return a three-dimensional target position."""
    if len(target) != 3:
        raise ValueError("target must contain x, y, and z")
    x, y, z = (float(value) for value in target)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise ValueError("target coordinates must be finite")
    return x, y, z


def is_reachable(target: Sequence[float]) -> tuple[bool, str]:
    """Check whether the target fits the real arm's joint limits."""
    try:
        target_x, target_y, target_z = _target_components(target)
    except (TypeError, ValueError) as error:
        return False, f"INVALID TARGET: {error}"

    x = target_x - SHOULDER_ORIGIN[0]
    y = target_y - SHOULDER_ORIGIN[1]
    z = target_z - SHOULDER_ORIGIN[2]
    distance = math.sqrt(x * x + y * y + z * z)

    if distance > MAX_REACH_M:
        return False, f"TOO FAR: target {distance:.2f} m (max {MAX_REACH_M:.2f} m)"
    if distance < MIN_REACH_M:
        return False, f"DEAD ZONE: target {distance:.2f} m (min {MIN_REACH_M:.2f} m)"

    # The right-arm URDF's neutral direction is rotated 90 degrees from world X.
    azimuth = math.degrees(math.atan2(y, x)) - 90.0
    azimuth = (azimuth + 180.0) % 360.0 - 180.0
    azimuth_limit = RIGHT_ARM_LIMITS_DEG["shoulder_z"]
    if not azimuth_limit.lower <= azimuth <= azimuth_limit.upper:
        return False, f"AZIMUTH OUT OF LIMIT: {azimuth:.1f} deg"

    horizontal_distance = math.hypot(x, y)
    shoulder, elbow = calculate_ik_angles(horizontal_distance, z)
    shoulder_limit = RIGHT_ARM_LIMITS_DEG["shoulder_y"]
    if not shoulder_limit.lower <= shoulder <= shoulder_limit.upper:
        return False, f"SHOULDER OUT OF LIMIT: {shoulder:.1f} deg"
    if not ELBOW_FLEXION_LIMIT.lower <= elbow <= ELBOW_FLEXION_LIMIT.upper:
        return False, f"ELBOW OUT OF LIMIT: {elbow:.1f} deg"

    return True, f"OK: target is reachable ({distance:.2f} m)"


def calculate_ik_angles(
    horizontal_distance: float,
    height_difference: float,
) -> tuple[float, float]:
    """Return shoulder elevation and elbow flexion in degrees.

    This preserves the V-shaped two-link solution used by the final LIMB25
    simulator. The target distance is clamped to the model's reachable shell.
    """
    horizontal_distance = float(horizontal_distance)
    height_difference = float(height_difference)
    if not all(math.isfinite(value) for value in (horizontal_distance, height_difference)):
        raise ValueError("IK distances must be finite")
    if horizontal_distance < 0:
        raise ValueError("horizontal distance cannot be negative")

    target_distance = math.hypot(horizontal_distance, height_difference)
    safe_distance = max(MIN_REACH_M, min(target_distance, MAX_REACH_M))

    alpha_cosine = (
        UPPER_ARM_LENGTH_M**2 + safe_distance**2 - LOWER_CHAIN_LENGTH_M**2
    ) / (2.0 * UPPER_ARM_LENGTH_M * safe_distance)
    gamma_cosine = (
        UPPER_ARM_LENGTH_M**2 + LOWER_CHAIN_LENGTH_M**2 - safe_distance**2
    ) / (2.0 * UPPER_ARM_LENGTH_M * LOWER_CHAIN_LENGTH_M)
    alpha = math.degrees(math.acos(max(-1.0, min(1.0, alpha_cosine))))
    gamma = math.degrees(math.acos(max(-1.0, min(1.0, gamma_cosine))))
    target_slope = math.degrees(math.atan2(height_difference, horizontal_distance))

    shoulder_elevation = alpha - target_slope
    elbow_flexion = gamma - 180.0
    return shoulder_elevation, elbow_flexion
