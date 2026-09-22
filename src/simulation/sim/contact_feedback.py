"""Contact values used by the simulated fingertip sensors."""

from __future__ import annotations

import math


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
