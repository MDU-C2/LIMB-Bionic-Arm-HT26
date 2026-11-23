# Christian och Majd
"""Convert Human Mimic movement progress into phase-selected DMP targets."""

from __future__ import annotations

import math
from numbers import Integral

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .dmp import DMPModel, rollout_simple
from .mimic import (
    MIMIC_JOINT_COUNT,
    MimicState,
    MimicTarget,
    ProgressState,
    validate_mimic_joint_count,
    validate_mimic_trajectory,
)


def _joint_vector(values: ArrayLike, *, name: str) -> NDArray[np.float64]:
    try:
        result = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain four joint angles in radians") from error
    if result.shape != (MIMIC_JOINT_COUNT,):
        raise ValueError(f"{name} must have shape ({MIMIC_JOINT_COUNT},)")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite radians")
    result = result.copy()
    result.setflags(write=False)
    return result


def _progress_value(value: float) -> float:
    if isinstance(value, bool):
        raise ValueError("progress must be between 0 and 1")
    try:
        progress = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("progress must be between 0 and 1") from error
    if not math.isfinite(progress) or not 0 <= progress <= 1:
        raise ValueError("progress must be between 0 and 1")
    return progress


class PhaseDrivenDMP:
    """Select four-joint DMP targets using human movement progress.

    The time-driven DMP is rolled out once on a normalized sample grid. Each
    later ``ProgressState`` selects and linearly interpolates a point on that
    rollout. Consequently, equal movement progress produces equal joint targets
    even when the human performs the movement at a different speed.

    This class returns human-space DMP angles. Robot mapping, safety limits and
    PyBullet control intentionally belong to later pipeline stages.
    """

    _PROGRESS_EPSILON = 1e-12

    def __init__(
        self,
        model: DMPModel,
        start_rad: ArrayLike,
        goal_rad: ArrayLike,
        *,
        sample_count: int = 501,
    ) -> None:
        if not isinstance(model, DMPModel):
            raise TypeError("model must be a DMPModel")
        validate_mimic_joint_count(model.n_joints)
        if isinstance(sample_count, bool) or not isinstance(sample_count, Integral):
            raise ValueError("sample_count must be an integer of at least 2")
        self.sample_count = int(sample_count)
        if self.sample_count < 2:
            raise ValueError("sample_count must be an integer of at least 2")

        self._start_rad = _joint_vector(start_rad, name="start_rad")
        self._goal_rad = _joint_vector(goal_rad, name="goal_rad")
        dt = model.tau / (self.sample_count - 1)
        rollout = rollout_simple(
            model,
            self._start_rad,
            self._goal_rad,
            tau=model.tau,
            dt=dt,
        )
        if rollout.shape[0] != self.sample_count:
            raise ValueError(
                "DMP rollout sample count does not match the requested phase grid"
            )
        self._trajectory_rad = validate_mimic_trajectory(
            rollout,
            name="DMP rollout",
        )
        self.reset()

    @property
    def trajectory_rad(self) -> NDArray[np.float64]:
        """The protected DMP rollout sampled from zero to full progress."""
        return self._trajectory_rad

    @property
    def last_target(self) -> MimicTarget:
        return self._last_target

    def reset(self) -> MimicTarget:
        """Clear the held target before starting a new Human Mimic movement."""
        self._last_angles: NDArray[np.float64] | None = None
        self._last_command_progress: float | None = None
        self._last_target = MimicTarget(
            0.0,
            None,
            MimicState.WAITING,
            "waiting for movement progress",
        )
        return self._last_target

    def angles_at(self, progress: float) -> NDArray[np.float64]:
        """Interpolate the DMP rollout at normalized progress in ``[0, 1]``."""
        progress = _progress_value(progress)
        position = progress * (self.sample_count - 1)
        left = int(math.floor(position))
        right = min(left + 1, self.sample_count - 1)
        fraction = position - left
        angles = (
            (1.0 - fraction) * self._trajectory_rad[left]
            + fraction * self._trajectory_rad[right]
        )
        angles.setflags(write=False)
        return angles

    def update(self, progress_state: ProgressState) -> MimicTarget:
        """Create the DMP target corresponding to one tracker output."""
        if not isinstance(progress_state, ProgressState):
            raise TypeError("progress_state must be a ProgressState")

        state = progress_state.state
        progress = progress_state.progress
        if state in {MimicState.WAITING, MimicState.LOST}:
            self._last_target = MimicTarget(
                progress,
                None,
                state,
                f"no DMP target: {progress_state.status or state.value}",
            )
            return self._last_target

        if state is MimicState.HOLDING and self._last_angles is not None:
            if not math.isclose(
                progress,
                float(self._last_command_progress),
                rel_tol=0.0,
                abs_tol=self._PROGRESS_EPSILON,
            ):
                raise ValueError("holding progress must equal the previous DMP target progress")
            self._last_target = MimicTarget(
                float(self._last_command_progress),
                self._last_angles,
                MimicState.HOLDING,
                "holding previous DMP target",
            )
            return self._last_target

        if (
            self._last_command_progress is not None
            and progress + self._PROGRESS_EPSILON < self._last_command_progress
        ):
            raise ValueError("DMP target progress must not move backwards; reset first")

        angles = self.angles_at(progress)
        self._last_angles = angles
        self._last_command_progress = progress
        if state is MimicState.COMPLETE:
            status = "final DMP target"
        elif state is MimicState.HOLDING:
            status = "initial held DMP target"
        else:
            status = "DMP target selected from movement progress"
        self._last_target = MimicTarget(progress, angles, state, status)
        return self._last_target
