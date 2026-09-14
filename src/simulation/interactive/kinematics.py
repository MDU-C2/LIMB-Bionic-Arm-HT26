"""Reachability and two-link inverse kinematics for the LIMB task scene."""

from __future__ import annotations

import math
from typing import Sequence


SHOULDER_ORIGIN = (0.0, 0.0, 0.0)
UPPER_ARM_LENGTH_M = 0.305
FOREARM_LENGTH_M = 0.310
HAND_GRIP_LENGTH_M = 0.120
LOWER_CHAIN_LENGTH_M = FOREARM_LENGTH_M + HAND_GRIP_LENGTH_M

MAX_REACH_M = UPPER_ARM_LENGTH_M + LOWER_CHAIN_LENGTH_M
# The inherited model limits elbow flexion to 90 degrees.
MIN_REACH_M = math.hypot(UPPER_ARM_LENGTH_M, LOWER_CHAIN_LENGTH_M)

AZIMUTH_LIMIT_DEG = (-95.0, 50.0)
ELEVATION_LIMIT_DEG = (-85.0, 150.0)


def _target_components(target: Sequence[float]) -> tuple[float, float, float]:
    """Validate and return a three-dimensional target position."""
    if len(target) != 3:
        raise ValueError("target must contain x, y, and z")
    x, y, z = (float(value) for value in target)
    if not all(math.isfinite(value) for value in (x, y, z)):
        raise ValueError("target coordinates must be finite")
    return x, y, z


def is_reachable(target: Sequence[float]) -> tuple[bool, str]:
    """Check the inherited distance, azimuth, and elevation limits."""
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
    if not AZIMUTH_LIMIT_DEG[0] <= azimuth <= AZIMUTH_LIMIT_DEG[1]:
        return False, f"AZIMUTH OUT OF LIMIT: {azimuth:.1f} deg"

    horizontal_distance = math.hypot(x, y)
    elevation = math.degrees(math.atan2(z, horizontal_distance))
    if not ELEVATION_LIMIT_DEG[0] <= elevation <= ELEVATION_LIMIT_DEG[1]:
        return False, f"ELEVATION OUT OF LIMIT: {elevation:.1f} deg"

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
    safe_distance = max(0.528, min(target_distance, 0.730))

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
