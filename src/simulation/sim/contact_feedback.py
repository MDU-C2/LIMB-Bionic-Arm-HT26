"""Contact values used by the simulated fingertip sensors."""

from __future__ import annotations

import math


def grasp_is_ready(
    curl: float,
    hand_target_distance_m: float,
    fingertip_contacts: int,
    *,
    min_curl: float,
    capture_distance_m: float,
    min_fingertip_contacts: int,
) -> bool:
    """Accept a close, curled hand even when mesh contact is intermittent.

    PyBullet contact points are useful feedback, but the imported finger meshes
    can miss a frame or touch the cup with a non-tip link. Proximity therefore
    acts as a deterministic fallback for the assisted grasp.
    """
    values = (curl, hand_target_distance_m, min_curl, capture_distance_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("grasp values must be finite")
    if min_fingertip_contacts < 0 or fingertip_contacts < 0:
        raise ValueError("contact counts cannot be negative")
    return curl >= min_curl and (
        fingertip_contacts >= min_fingertip_contacts
        or hand_target_distance_m <= capture_distance_m
    )


def estimate_contact_force_n(
    normal_force_n: float,
    contact_distance_m: float,
    stiffness_n_per_m: float,
) -> float:
    """Return a nonnegative contact signal from a PyBullet contact point."""
    values = (normal_force_n, contact_distance_m, stiffness_n_per_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("contact values must be finite")
    if stiffness_n_per_m < 0:
        raise ValueError("contact stiffness cannot be negative")
    penetration_force = max(0.0, -contact_distance_m * stiffness_n_per_m)
    return max(0.0, normal_force_n, penetration_force)
