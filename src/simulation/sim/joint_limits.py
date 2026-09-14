"""Oscar's executable four-joint software limits, in radians."""

from __future__ import annotations

import numpy as np


JOINT_LIMITS_RAD = np.array(
    [
        [0.0, 1.05],    # elbow flexion
        [0.0, 1.39],    # shoulder flexion
        [0.0, 0.69],    # shoulder abduction
        [-0.69, 0.69],  # shoulder internal/external rotation
    ]
)


def clamp_dmp_vector(q_rad: np.ndarray) -> np.ndarray:
    """Return a limited copy of a `(4,)` vector or `(..., 4)` trajectory."""
    q = np.asarray(q_rad, dtype=float)
    if q.ndim == 0 or q.shape[-1] != 4:
        raise ValueError(f"Expected final dimension 4, got {q.shape}")
    if not np.all(np.isfinite(q)):
        raise ValueError("Trajectory contains NaN or infinity")
    return np.clip(q, JOINT_LIMITS_RAD[:, 0], JOINT_LIMITS_RAD[:, 1])
