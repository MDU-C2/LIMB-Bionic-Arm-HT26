"""Reachability and two-link inverse kinematics for the LIMB task scene."""

from __future__ import annotations

import math
from typing import Sequence

from sim.joint_limits import LEFT_ARM_LIMITS_DEG
from sim.robot_model import FOREARM_LENGTH_M, LEFT_GRIP_OFFSET_M, UPPER_ARM_LENGTH_M


SHOULDER_ORIGIN = (0.0, 0.0, 0.0)
HAND_GRIP_LENGTH_M = abs(LEFT_GRIP_OFFSET_M[0])
LOWER_CHAIN_LENGTH_M = FOREARM_LENGTH_M + HAND_GRIP_LENGTH_M

MAX_REACH_M = UPPER_ARM_LENGTH_M + LOWER_CHAIN_LENGTH_M
ELBOW_FLEXION_LIMIT = LEFT_ARM_LIMITS_DEG["elbow_x"]
SHOULDER_FLEXION_LIMIT = LEFT_ARM_LIMITS_DEG["shoulder_y"]
IK_POSITION_TOLERANCE_M = 0.04
MIN_ELBOW_ANGLE_RAD = math.radians(180.0 - ELBOW_FLEXION_LIMIT.upper)
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

    # The exact left-arm mirror points along world +Y at zero.  Negative
    # shoulder-Z moves it into the left (negative-X) half of the table.
    azimuth = 90.0 - math.degrees(math.atan2(y, x))
    azimuth = (azimuth + 180.0) % 360.0 - 180.0
    azimuth_limit = LEFT_ARM_LIMITS_DEG["shoulder_z"]
    if not azimuth_limit.lower <= azimuth <= azimuth_limit.upper:
        return False, f"AZIMUTH OUT OF LIMIT: {azimuth:.1f} deg"

    horizontal_distance = math.hypot(x, y)
    shoulder, elbow = calculate_ik_angles(horizontal_distance, z)
    if not SHOULDER_FLEXION_LIMIT.lower <= shoulder <= SHOULDER_FLEXION_LIMIT.upper:
        return False, f"SHOULDER OUT OF LIMIT: {shoulder:.1f} deg"
    if not ELBOW_FLEXION_LIMIT.lower <= elbow <= ELBOW_FLEXION_LIMIT.upper:
        return False, f"ELBOW OUT OF LIMIT: {elbow:.1f} deg"
    reached_horizontal, reached_height = _planar_position(shoulder, elbow)
    residual = math.hypot(
        reached_horizontal - horizontal_distance,
        reached_height - z,
    )
    if residual > IK_POSITION_TOLERANCE_M:
        return False, f"JOINT-LIMIT REACH ERROR: {residual:.2f} m"

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

    # Preserve the proven task-scene solution. The left URDF exposes physical
    # elbow flexion as a positive angle, so only the elbow sign differs from
    # the former right-arm command.
    shoulder_elevation = alpha - target_slope
    elbow_flexion = 180.0 - gamma
    return shoulder_elevation, elbow_flexion


def _planar_position(shoulder_deg: float, elbow_deg: float) -> tuple[float, float]:
    """Return horizontal reach and height for left-arm flexion angles."""
    shoulder = math.radians(shoulder_deg)
    lower = shoulder - math.radians(elbow_deg)
    return (
        UPPER_ARM_LENGTH_M * math.cos(shoulder)
        + LOWER_CHAIN_LENGTH_M * math.cos(lower),
        -UPPER_ARM_LENGTH_M * math.sin(shoulder)
        - LOWER_CHAIN_LENGTH_M * math.sin(lower),
    )
