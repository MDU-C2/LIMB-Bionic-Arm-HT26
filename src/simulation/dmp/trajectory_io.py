"""Load Oscar Ågren's four-joint DMP trajectories or fit one from angles."""

from __future__ import annotations

from pathlib import Path
from typing import Literal
import warnings

import numpy as np

from .dmp import fit, rollout_simple


# --- LIMB25 filename compatibility -----------------------------------------


def _prefix(trial_dir: Path) -> str:
    """Return Oscar's optional subject/motion/trial filename prefix."""
    try:
        subject = int(trial_dir.parent.parent.name.removeprefix("subject_"))
        trial = int(trial_dir.name.removeprefix("trial_"))
    except ValueError:
        return ""
    motion = trial_dir.parent.name[:1].upper() or "X"
    return f"{subject:02d}{motion}{trial:03d}_"


def _first_existing(trial_dir: Path, names: list[str]) -> Path | None:
    """Return the first prefixed or plain candidate file that exists."""
    prefix = _prefix(trial_dir)
    for name in names:
        for candidate in (trial_dir / f"{prefix}{name}", trial_dir / name):
            if candidate.exists():
                return candidate
    return None


def _validate(q_rad: np.ndarray, dt: float) -> np.ndarray:
    """Validate a four-joint trajectory and its sample interval."""
    q = np.asarray(q_rad, dtype=float)
    if q.ndim != 2 or q.shape[0] < 2 or q.shape[1] != 4:
        raise ValueError(f"Expected trajectory shape (T>=2, 4), got {q.shape}")
    if not np.all(np.isfinite(q)):
        raise ValueError("Trajectory contains NaN or infinity")
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError(f"Expected a finite positive timestep, got {dt}")
    return q


# --- Public trajectory loaders ---------------------------------------------


def load_angles_demo(
    trial_dir: Path, source: Literal["auto", "raw", "clean"] = "auto"
) -> np.ndarray:
    """Load elbow plus three shoulder angles as a `(T, 4)` radian array."""
    names = {
        "auto": ["angles.npz", "angles_raw.npz", "angles_clean.npz"],
        "raw": ["angles_raw.npz", "angles.npz"],
        "clean": ["angles_clean.npz", "angles.npz"],
    }
    if source not in names:
        raise ValueError(f"Unknown angles source: {source}")
    path = _first_existing(trial_dir, names[source])
    if path is None:
        raise FileNotFoundError(f"No angles NPZ found in {trial_dir}")

    with np.load(path, allow_pickle=False) as data:
        if "elbow_rad" in data and "shoulder_rad" in data:
            elbow, shoulder = data["elbow_rad"], data["shoulder_rad"]
        elif "elbow_deg" in data and "shoulder_deg" in data:
            elbow = np.deg2rad(data["elbow_deg"])
            shoulder = np.deg2rad(data["shoulder_deg"])
        else:
            raise KeyError(f"Unexpected angle keys in {path}: {list(data.keys())}")

    q_demo = np.column_stack([elbow, shoulder])
    if q_demo.ndim != 2 or q_demo.shape[1] != 4:
        raise ValueError(f"Expected four angle columns, got {q_demo.shape}")
    # Sensor dropouts in the inherited recordings appear as non-finite rows.
    valid = np.all(np.isfinite(q_demo), axis=1)
    if not np.all(valid):
        warnings.warn(
            f"Removed {np.count_nonzero(~valid)} invalid angle rows from {path}",
            stacklevel=2,
        )
    q_demo = q_demo[valid]
    if len(q_demo) < 10:
        raise ValueError(f"Not enough valid angle samples: {q_demo.shape}")
    return q_demo


def resolve_saved_dmp_rollout_path(
    trial_dir: Path, rollout_source: Literal["clean", "raw"] = "clean"
) -> Path | None:
    """Resolve the preferred saved rollout, falling back to the other variant."""
    other = "raw" if rollout_source == "clean" else "clean"
    return _first_existing(
        trial_dir,
        [f"dmp_rollout_{rollout_source}.npz", f"dmp_rollout_{other}.npz"],
    )


def load_dmp_trajectory(
    trial_dir: Path,
    *,
    prefer_saved_rollout: bool = True,
    rollout_source: Literal["clean", "raw"] = "clean",
) -> tuple[np.ndarray, float]:
    """Load a saved rollout, or reproduce Oscar's one-second DMP fallback."""
    path = resolve_saved_dmp_rollout_path(trial_dir, rollout_source)
    if prefer_saved_rollout and path is not None:
        with np.load(path, allow_pickle=False) as data:
            if "q_gen_rad" not in data or "dt" not in data:
                raise KeyError(f"Unexpected rollout keys in {path}: {list(data.keys())}")
            q_gen = np.asarray(data["q_gen_rad"], dtype=float)
            dt_values = np.asarray(data["dt"], dtype=float)
        if dt_values.size != 1:
            raise ValueError(f"Expected one timestep in {path}, got {dt_values.shape}")
        dt = float(dt_values.reshape(-1)[0])
        return _validate(q_gen, dt), dt

    q_demo = load_angles_demo(trial_dir, source=rollout_source)
    dt = 1.0 / (len(q_demo) - 1)
    # These parameters reproduce Oscar Ågren's one-second LIMB25 rollout.
    model = fit(
        [q_demo],
        tau=1.0,
        dt=dt,
        n_basis_functions=15,
        alpha_canonical=4.0,
        alpha_transformation=25.0,
        beta_transformation=6.25,
    )
    q_gen = rollout_simple(model, q_demo[0], q_demo[-1], tau=1.0, dt=dt)
    return _validate(q_gen, dt), dt
