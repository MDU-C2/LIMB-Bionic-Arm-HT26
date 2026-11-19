# Majd
"""Fit and replay movement trajectories using dynamic movement primitives."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aurora.simulation.dmp.dmp_signal import canonical_phase as canonical_phase
from aurora.simulation.dmp.dmp_signal import estimate_derivatives as estimate_derivatives
from aurora.simulation.dmp.dmp_signal import savgol_estimation as savgol_estimation

SPATIAL_SCALE_EPS_RAD = 0.02


def _spatial_scale(displacement, epsilon=SPATIAL_SCALE_EPS_RAD):
    # Near-equal endpoints retain the demonstrated motion's absolute amplitude.
    return np.where(np.abs(displacement) < epsilon, 1.0, displacement)


@dataclass
class DMPModel:
    """Parameters required for DMP rollouts and forcing reconstruction."""

    weights: np.ndarray  # shape: (n_joints, n_basis_functions)
    centers: np.ndarray  # shape: (n_basis_functions,) in phase
    widths: np.ndarray  # shape: (n_basis_functions,)
    alpha_canonical: float
    alpha_transformation: float
    beta_transformation: float
    tau: float
    n_joints: int


def _rbf_normalized(x: np.ndarray, centers: np.ndarray, widths: np.ndarray) -> np.ndarray:
    """Compute Gaussian RBF activations normalized row-wise."""
    psi = np.exp(-widths * (x[:, None] - centers[None, :]) ** 2)
    return psi / (psi.sum(axis=1, keepdims=True) + 1e-10)


def _validate_and_get_demo_shape(demos: list[np.ndarray]) -> tuple[int, int]:
    """Validate demos list and return (T_demo, n_joints)."""
    if not demos:
        raise ValueError("demos must be a non-empty list")
    if demos[0].ndim != 2:
        raise ValueError(f"demos[0] must be 2D (T, n_joints), got shape {demos[0].shape}")

    T_demo = int(demos[0].shape[0])
    n_joints = int(demos[0].shape[1])
    if T_demo < 2 or n_joints < 1:
        raise ValueError("demos must contain at least two samples and one joint")
    for i, q in enumerate(demos):
        if q.ndim != 2:
            raise ValueError(f"demos[{i}] must be 2D (T, n_joints), got shape {q.shape}")
        if q.shape[1] != n_joints:
            raise ValueError(f"demos[{i}] has n_joints={q.shape[1]}, expected {n_joints}")
        if q.shape[0] != T_demo:
            raise ValueError(
                "All demos must have the same length before calling fit "
                f"(got demos[0].shape[0]={T_demo} but demos[{i}].shape[0]={q.shape[0]})."
            )
        if not np.all(np.isfinite(q)):
            raise ValueError(f"demos[{i}] contains NaN or infinity")
    return T_demo, n_joints


def _centers_and_widths(
    alpha_canonical: float,
    n_basis_functions: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Build phase-space RBF centers and widths used by fit."""
    centers = np.exp(-np.linspace(0, 1, n_basis_functions) * alpha_canonical)  # 1 -> ~0
    if n_basis_functions <= 1:
        widths = np.array([1.0], dtype=np.float64)
    else:
        d = np.diff(centers)
        d = np.hstack((d, [d[-1]]))
        widths = 1.0 / (d**2 + 1e-12)
    return centers, widths


def _compute_f_target(
    q_joint: np.ndarray,
    *,
    tau: float,
    dt: float,
    alpha_transformation: float,
    beta_transformation: float,
    diff_g_q0_eps: float,
    savgol_window_length: int,
    savgol_polyorder: int,
) -> np.ndarray:
    """
    Compute the DMP forcing target for one joint.
    """
    dq, ddq = estimate_derivatives(
        q_joint,
        dt=dt,
        derivative_method="savgol",
        savgol_window_length=savgol_window_length,
        savgol_polyorder=savgol_polyorder,
    )

    q0_joint = float(q_joint[0])
    g_joint = float(q_joint[-1])
    g_minus_q0 = g_joint - q0_joint
    scale = _spatial_scale(g_minus_q0, diff_g_q0_eps)

    f_target = (
        tau**2 * ddq
        - alpha_transformation * beta_transformation * (g_joint - q_joint)
        + alpha_transformation * tau * dq
    ) / scale
    return f_target


def _solve_lwr_like_weights(
    *,
    demos: list[np.ndarray],
    phi: np.ndarray,
    n_joints: int,
    n_basis_functions: int,
    tau: float,
    dt: float,
    alpha_transformation: float,
    beta_transformation: float,
    ridge_lambda: float,
    diff_g_q0_eps: float,
    savgol_window_length: int,
    savgol_polyorder: int,
) -> np.ndarray:
    """
    Solve DMP weights with an LWR-style normal-equation solve.

    For each joint, it solves:
        (Phi^T Phi + lambda I) w = Phi^T f_target
    aggregated across all demos.

    return: weights, shape (n_joints, n_basis_functions)
    """
    n_demos = len(demos)
    # Phi is shared across demos, so A is the same per demo and can be scaled once.
    A = n_demos * (phi.T @ phi)
    A_reg = A + ridge_lambda * np.eye(n_basis_functions, dtype=np.float64)

    weights = np.zeros((n_joints, n_basis_functions), dtype=np.float64)

    for joint in range(n_joints):
        # b = sum_d Phi^T f_target_d
        b = np.zeros((n_basis_functions,), dtype=np.float64)
        for demo in demos:
            q_joint = demo[:, joint]
            f_target = _compute_f_target(
                q_joint,
                tau=tau,
                dt=dt,
                alpha_transformation=alpha_transformation,
                beta_transformation=beta_transformation,
                diff_g_q0_eps=diff_g_q0_eps,
                savgol_window_length=savgol_window_length,
                savgol_polyorder=savgol_polyorder,
            )

            b += phi.T @ f_target

        weights[joint, :] = np.linalg.solve(A_reg, b)

    return weights


def fit(
    demos: list[np.ndarray],
    tau: float,
    dt: float,
    n_basis_functions: int,
    alpha_canonical: float,
    alpha_transformation: float,
    beta_transformation: float,
) -> DMPModel:
    """
    Fit a DMP from a list of joint trajectories.
    This is the main function to fit a DMP model to a list of joint trajectories.

    Args:
        demos: List of trajectories, each with shape `(T, n_joints)`, in radians.

    Returns:
        DMPModel: the fitted DMP model
    """
    demos = [np.asarray(demo, dtype=float) for demo in demos]
    if not np.isfinite(tau) or tau <= 0:
        raise ValueError("tau must be finite and positive")
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    if n_basis_functions < 1:
        raise ValueError("n_basis_functions must be at least 1")
    transformation_parameters = (
        alpha_canonical,
        alpha_transformation,
        beta_transformation,
    )
    if not all(np.isfinite(value) and value > 0 for value in transformation_parameters):
        raise ValueError("DMP alpha and beta parameters must be finite and positive")

    # Every demonstration must share one sample grid for the normal-equation solve.
    T_demo, n_joints = _validate_and_get_demo_shape(demos)

    t_demo = np.arange(T_demo, dtype=np.float64) * dt
    phase = canonical_phase(
        t_demo,
        tau=tau,
        alpha_canonical=alpha_canonical,
    )

    centers, widths = _centers_and_widths(alpha_canonical, n_basis_functions)
    phi = _rbf_normalized(phase, centers, widths) * phase[:, None]

    ridge_lambda = 1e-6
    diff_g_q0_eps = SPATIAL_SCALE_EPS_RAD
    savgol_window_length = 11
    savgol_polyorder = 3

    weights = _solve_lwr_like_weights(
        demos=demos,
        phi=phi,
        n_joints=n_joints,
        n_basis_functions=n_basis_functions,
        tau=tau,
        dt=dt,
        alpha_transformation=alpha_transformation,
        beta_transformation=beta_transformation,
        ridge_lambda=ridge_lambda,
        diff_g_q0_eps=diff_g_q0_eps,
        savgol_window_length=savgol_window_length,
        savgol_polyorder=savgol_polyorder,
    )

    model = DMPModel(
        weights=weights,
        centers=centers,
        widths=widths,
        alpha_canonical=alpha_canonical,
        alpha_transformation=alpha_transformation,
        beta_transformation=beta_transformation,
        tau=tau,
        n_joints=n_joints,
    )
    return model


def _validate_rollout_inputs(
    model: DMPModel,
    q0: np.ndarray,
    goal: np.ndarray,
    tau: float,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Validate shared rollout inputs and return normalized arrays and length."""
    initial = np.asarray(q0, dtype=float)
    target = np.asarray(goal, dtype=float)
    expected_shape = (model.n_joints,)
    if initial.shape != expected_shape or target.shape != expected_shape:
        raise ValueError(f"q0 and g must each have shape {expected_shape}")
    if not np.all(np.isfinite(initial)) or not np.all(np.isfinite(target)):
        raise ValueError("q0 and g must contain only finite values")
    if not np.isfinite(tau) or tau <= 0 or not np.isfinite(dt) or dt <= 0:
        raise ValueError("tau and dt must be finite and positive")
    return initial, target, int(round(tau / dt)) + 1


def rollout_simple(
    model: DMPModel,
    q0: np.ndarray,
    g: np.ndarray,
    tau: float,
    dt: float,
) -> np.ndarray:
    """Generate a trajectory with Euler integration."""
    q0, goal, n_steps = _validate_rollout_inputs(model, q0, g, tau, dt)
    generated = np.zeros((n_steps, model.n_joints))
    position = q0.copy()
    velocity = np.zeros_like(position)
    generated[0] = position
    spatial_scale = _spatial_scale(goal - q0)

    time_value = 0.0
    for step_index in range(1, n_steps):
        phase = canonical_phase(
            time_value,
            tau=tau,
            alpha_canonical=model.alpha_canonical,
        )
        activations = np.exp(-model.widths * (phase - model.centers) ** 2)
        normalized_activations = activations / (activations.sum() + 1e-10)

        # The basis activations are shared by every joint, so calculate them once.
        forcing = phase * (model.weights @ normalized_activations)
        acceleration = (
            model.alpha_transformation * model.beta_transformation * (goal - position)
            - model.alpha_transformation * tau * velocity
            + spatial_scale * forcing
        ) / tau**2
        velocity += acceleration * dt
        position += velocity * dt
        generated[step_index] = position
        time_value += dt

    return generated
