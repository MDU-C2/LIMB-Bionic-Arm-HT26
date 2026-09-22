"""Dynamic Movement Primitive fitting and rollout algorithms.

The trajectory simulator uses the simple Euler rollout. Curvature coupling and
RK4 remain available for later motion experiments, but their additional model
data is only fitted when explicitly requested.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import savgol_filter


# --- 1. Model and basis functions ------------------------------------------


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
    curvature_weights: np.ndarray | None = None


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
        widths = 1.0 / (d ** 2 + 1e-12)
    return centers, widths

# --- 2. Trajectory derivatives ---------------------------------------------


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
    if wl < 3:
        dq = np.gradient(y, dt)
        ddq = np.gradient(dq, dt)
        return dq, ddq

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
    if q.size < 3: # Not enough samples to do a proper Savitzky-Golay estimate.
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
            f"Unknown derivative_method '{derivative_method}'. "
            "Use 'gradient' or 'savgol'."
        )

    # 4. Estimate derivatives using Savitzky-Golay helper.
    return savgol_estimation(
        q,
        dt=dt,
        savgol_window_length=savgol_window_length,
        savgol_polyorder=savgol_polyorder,
    )

# --- 3. Optional curvature coupling ----------------------------------------


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
        x = float(
            canonical_phase(
                np.array([t]), tau=tau, alpha_canonical=model.alpha_canonical
            )
        )

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
        row_blocks = [
            x * psi_norm[index] * perpendicular_projection for index in range(n_basis)
        ]
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

# --- 4. Model fitting -------------------------------------------------------


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
    scale = 1.0 if abs(g_minus_q0) < diff_g_q0_eps else g_minus_q0

    f_target = (
        tau**2 * ddq
        - alpha_transformation * beta_transformation * (g_joint - q_joint)
        + alpha_transformation * dq
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


def canonical_phase(
    t: float | np.ndarray,
    *,
    tau: float,
    alpha_canonical: float,
) -> np.ndarray:
    """Canonical DMP phase variable x(t) = exp(-alpha * t / tau)."""
    t = np.asarray(t, dtype=float)
    return np.exp(-alpha_canonical * t / tau)


def fit(
    demos: list[np.ndarray],
    tau: float,
    dt: float,
    n_basis_functions: int,
    alpha_canonical: float,
    alpha_transformation: float,
    beta_transformation: float,
    *,
    learn_curvature: bool = False,
) -> DMPModel:
    """
    Fit a DMP from a list of joint trajectories.
    This is the main function to fit a DMP model to a list of joint trajectories.

    Args:
        demos: List of trajectories, each with shape `(T, n_joints)`, in radians.
        learn_curvature: Fit optional cross-joint curvature coupling weights.

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

    # These values reproduce the migrated LIMB25 fitting behavior.
    ridge_lambda = 1e-6
    diff_g_q0_eps = 0.02  # About 1.15 degrees when angles are in radians.
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
    if learn_curvature:
        fitted_curvature = [
            learn_curvature_weights_from_demo(
                demo=demo,
                model=model,
                dt=dt,
                ridge_lambda=ridge_lambda,
            )
            for demo in demos
        ]
        model.curvature_weights = np.mean(fitted_curvature, axis=0)
    return model

# --- 5. Trajectory rollouts -------------------------------------------------


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
            model.alpha_transformation
            * model.beta_transformation
            * (goal - position)
            - model.alpha_transformation * velocity
            + (goal - q0) * forcing
        ) / tau**2
        velocity += acceleration * dt
        position += velocity * dt
        generated[step_index] = position
        time_value += dt

    return generated


def rollout_simple_with_coupling(
    model: DMPModel,
    q0: np.ndarray,
    g: np.ndarray,
    tau: float,
    dt: float,
) -> np.ndarray:
    """Generate an Euler rollout with optional learned curvature coupling."""
    q0, g, n_steps = _validate_rollout_inputs(model, q0, g, tau, dt)
    q_gen = np.zeros((n_steps, model.n_joints))
    q = q0.copy().astype(float)
    dq = np.zeros_like(q)
    q_gen[0] = q
    t = 0.0
    for k in range(1, n_steps):

        x = canonical_phase(t, tau=tau, alpha_canonical=model.alpha_canonical)

        psi = np.exp(-model.widths * (x - model.centers) ** 2)
        psi_norm = psi / (psi.sum() + 1e-10)

        f = x * (model.weights @ psi_norm)

        curvature = np.zeros_like(q)
        if model.curvature_weights is not None:
            curvature = curvature_coupling(
                q,
                g,
                x,
                model.centers,
                model.widths,
                model.curvature_weights,
            )

        ddq = (
            model.alpha_transformation * model.beta_transformation * (g - q)
            - model.alpha_transformation * dq
            + (g - q0) * f
            + curvature
        ) / tau**2
        dq += ddq * dt
        q += dq * dt
        q_gen[k] = q
        t += dt

    return q_gen


def rollout_rk4(
    model: DMPModel,
    q0: np.ndarray,
    g: np.ndarray,
    tau: float,
    dt: float,
) -> np.ndarray:
    """Generate a trajectory with fourth-order Runge-Kutta integration."""
    q0, goal, step_count = _validate_rollout_inputs(model, q0, g, tau, dt)
    joint_count = model.n_joints

    position = q0.copy()
    velocity = np.zeros_like(position)
    phase = 1.0
    state = np.concatenate([position, velocity, np.array([phase])])

    generated = np.zeros((step_count, joint_count), dtype=float)
    generated[0] = position

    # Local references avoid repeated attribute lookups inside the integrator.
    alpha_c = model.alpha_canonical
    alpha_z = model.alpha_transformation
    beta_z = model.beta_transformation
    centers = model.centers
    widths = model.widths
    weights = model.weights

    def forcing(phase_value: float) -> np.ndarray:
        """Compute the forcing term for every joint at one phase value."""
        activations = np.exp(-widths * (phase_value - centers) ** 2)
        normalized = activations / (activations.sum() + 1e-10)
        return phase_value * (weights @ normalized)

    def state_derivative(_time_value: float, current_state: np.ndarray) -> np.ndarray:
        """Return position, velocity, and canonical-phase derivatives."""
        current_position = current_state[:joint_count]
        current_velocity = current_state[joint_count : 2 * joint_count]
        current_phase = current_state[2 * joint_count]
        phase_derivative = -alpha_c * current_phase / tau
        acceleration = (
            alpha_z * beta_z * (goal - current_position)
            - alpha_z * current_velocity
            + (goal - q0) * forcing(current_phase)
        ) / tau**2
        return np.concatenate(
            [current_velocity, acceleration, np.array([phase_derivative])]
        )

    time_value = 0.0
    for step_index in range(1, step_count):
        k1 = state_derivative(time_value, state)
        k2 = state_derivative(time_value + 0.5 * dt, state + 0.5 * dt * k1)
        k3 = state_derivative(time_value + 0.5 * dt, state + 0.5 * dt * k2)
        k4 = state_derivative(time_value + dt, state + dt * k3)
        state += (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        # Numerical error near the end can otherwise make phase slightly negative.
        state[2 * joint_count] = max(0.0, state[2 * joint_count])
        generated[step_index] = state[:joint_count]
        time_value += dt

    return generated
