# Majd
"""Trajectory derivatives and optional DMP curvature coupling."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from scipy.signal import savgol_filter

if TYPE_CHECKING:
    from aurora.simulation.dmp.dmp import DMPModel


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


def curvature_coupling(q, g, x, centers, widths, curvature_weights):
    """Curvature coupling function
    Args:
        q: current joint position, shape (n_joints,)
        g: goal joint position, shape (n_joints,)
        x: canonical phase, scalar (works as a phase gate)
        centers: centers, shape (n_basis,)
        widths: widths, shape (n_basis,)
        curvature_weights: shape (n_joints, n_basis)
    Returns:
        curvature_coupling: shape (n_joints,)
    """
    psi = np.exp(-widths * (x - centers) ** 2)
    psi_norm = psi / (psi.sum() + 1e-10)

    curvature_direction = curvature_weights @ psi_norm

    diff = g - q
    norm = np.linalg.norm(diff)

    if norm < 1e-5:
        return np.zeros_like(q)

    direction_to_goal = diff / norm
    joint_count = q.shape[0]
    perpendicular_projection = np.eye(joint_count) - np.outer(
        direction_to_goal,
        direction_to_goal,
    )

    return x * (perpendicular_projection @ curvature_direction)


def learn_curvature_weights_from_demo(
    demo: np.ndarray,
    model: DMPModel,
    dt: float,
    ridge_lambda: float,
) -> np.ndarray:
    """Learn curvature weights from a demo."""
    T, n_joints = demo.shape
    n_basis = model.centers.shape[0]

    q0 = demo[0]
    g = demo[-1]
    tau = model.tau

    dq = np.zeros_like(demo)
    ddq = np.zeros_like(demo)

    for joint in range(n_joints):
        dq[:, joint], ddq[:, joint] = estimate_derivatives(
            demo[:, joint],
            dt=dt,
            derivative_method="savgol",
            savgol_window_length=11,
            savgol_polyorder=3,
        )

    Phi_rows, Y_rows = [], []
    for k in range(T):
        t = k * dt
        x = float(canonical_phase(np.array([t]), tau=tau, alpha_canonical=model.alpha_canonical))

        q = demo[k]
        dq_k = dq[k]
        ddq_k = ddq[k]
        psi = np.exp(-model.widths * (x - model.centers) ** 2)
        psi_norm = psi / (psi.sum() + 1e-10)
        f = x * (model.weights @ psi_norm)

        baseline_numerator = (
            model.alpha_transformation * model.beta_transformation * (g - q)
            - model.alpha_transformation * dq_k
            + (g - q0) * f
        )
        C_target = tau**2 * ddq_k - baseline_numerator

        diff = g - q
        norm = np.linalg.norm(diff)
        if norm < 1e-5:
            continue
        e = diff / norm
        perpendicular_projection = np.eye(n_joints) - np.outer(e, e)
        row_blocks = [x * psi_norm[index] * perpendicular_projection for index in range(n_basis)]
        Phi_rows.append(np.concatenate(row_blocks, axis=1))
        Y_rows.append(C_target)

    if not Phi_rows:
        return np.zeros((n_joints, n_basis), dtype=float)

    Phi = np.vstack(Phi_rows)
    Y = np.concatenate(Y_rows)
    A = Phi.T @ Phi + ridge_lambda * np.eye(Phi.shape[1])
    b = Phi.T @ Y
    c_flat = np.linalg.solve(A, b)
    return c_flat.reshape(n_basis, n_joints).T


def canonical_phase(
    t: float | np.ndarray,
    *,
    tau: float,
    alpha_canonical: float,
) -> np.ndarray:
    """Canonical DMP phase variable x(t) = exp(-alpha * t / tau)."""
    t = np.asarray(t, dtype=float)
    return np.exp(-alpha_canonical * t / tau)
