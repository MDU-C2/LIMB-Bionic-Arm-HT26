# Majd
"""Trajectory derivatives and the canonical DMP phase."""

from __future__ import annotations

import numpy as np
from scipy.signal import savgol_filter


def savgol_estimation(
    q: np.ndarray,
    *,
    dt: float,
    savgol_window_length: int = 11,
    savgol_polyorder: int = 3,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Estimate dq and ddq using a Savitzky-Golay smoothing + derivative pass.

    Falls back to gradient-based derivatives when there are too few points
    for a valid Savitzky-Golay setup.
    """
    y = np.asarray(q, dtype=float)
    T = y.size

    wl = min(int(savgol_window_length), T if T % 2 == 1 else T - 1)
    if wl < 3:
        dq = np.gradient(y, dt)
        ddq = np.gradient(dq, dt)
        return dq, ddq
    if wl % 2 == 0:
        wl -= 1

    po = int(savgol_polyorder)
    if po >= wl:
        po = wl - 1
    if po < 1:
        po = 1

    y_smooth = savgol_filter(y, window_length=wl, polyorder=po, mode="interp")
    dq = savgol_filter(y_smooth, window_length=wl, polyorder=po, deriv=1, delta=dt, mode="interp")
    ddq = savgol_filter(y_smooth, window_length=wl, polyorder=po, deriv=2, delta=dt, mode="interp")
    return dq, ddq


def estimate_derivatives(
    q: np.ndarray,
    *,
    dt: float,
    derivative_method: str = "savgol",
    savgol_window_length: int = 11,
    savgol_polyorder: int = 3,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Estimate dq and ddq from a 1D trajectory q(t).
    This is used to compute the forcing term for each joint in the transformation system.

    Args:
        q: np.ndarray: the trajectory
        dt: float: the time step
        derivative_method: str: the method to use for derivative estimation
        savgol_window_length: int: the window length for Savitzky-Golay filter
        savgol_polyorder: int: the polynomial order for Savitzky-Golay filter

    Returns:
        tuple[np.ndarray, np.ndarray]: the estimated dq and ddq
    """
    # 1. Convert the trajectory to a numpy array
    q = np.asarray(q, dtype=float)
    if q.ndim != 1:
        raise ValueError(f"Expected 1D q, got shape {q.shape}")
    if q.size < 3:  # Not enough samples to do a proper Savitzky-Golay estimate.
        dq = np.gradient(q, dt)
        ddq = np.gradient(dq, dt)
        return dq, ddq

    # 2. Get the method to use for derivative estimation
    method = derivative_method.strip().lower()

    # 3. Estimate the derivatives using gradient method if specified
    if method == "gradient":
        dq = np.gradient(q, dt)
        ddq = np.gradient(dq, dt)
        return dq, ddq

    if method != "savgol":
        raise ValueError(
            f"Unknown derivative_method '{derivative_method}'. Use 'gradient' or 'savgol'."
        )

    # 4. Estimate derivatives using Savitzky-Golay helper.
    return savgol_estimation(
        q,
        dt=dt,
        savgol_window_length=savgol_window_length,
        savgol_polyorder=savgol_polyorder,
    )


def canonical_phase(
    t: float | np.ndarray,
    *,
    tau: float,
    alpha_canonical: float,
) -> np.ndarray:
    """Canonical DMP phase variable x(t) = exp(-alpha * t / tau)."""
    t = np.asarray(t, dtype=float)
    return np.exp(-alpha_canonical * t / tau)
