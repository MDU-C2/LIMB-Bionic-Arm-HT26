# Christian och Majd
"""Shared data contracts for phase-driven human mimic control."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from numbers import Integral
from typing import Final

import numpy as np
from numpy.typing import ArrayLike, NDArray

MIMIC_JOINTS: Final = (
    "elbow_flexion",
    "shoulder_flexion",
    "shoulder_abduction",
    "shoulder_rotation_proxy",
)
MIMIC_JOINT_COUNT: Final = len(MIMIC_JOINTS)


class MimicState(str, Enum):
    """Lifecycle states shared by progress tracking and DMP targets."""

    WAITING = "waiting"
    RUNNING = "running"
    HOLDING = "holding"
    COMPLETE = "complete"
    LOST = "lost"


def _finite_nonnegative(value: float, *, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a finite nonnegative number") from error
    if not math.isfinite(result) or result < 0:
        raise ValueError(f"{name} must be a finite nonnegative number")
    return result


def _unit_interval(value: float, *, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be between 0 and 1") from error
    if not math.isfinite(result) or not 0 <= result <= 1:
        raise ValueError(f"{name} must be between 0 and 1")
    return result


def _text(value: object, *, name: str, allow_empty: bool) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{name} must be text")
    result = value.strip()
    if not allow_empty and not result:
        raise ValueError(f"{name} must not be empty")
    return result


def _state(value: MimicState | str) -> MimicState:
    try:
        return MimicState(value)
    except (TypeError, ValueError) as error:
        choices = ", ".join(state.value for state in MimicState)
        raise ValueError(f"state must be one of: {choices}") from error


def _angles(
    value: ArrayLike | None,
    *,
    name: str,
    allow_none: bool,
) -> NDArray[np.float64] | None:
    if value is None:
        if allow_none:
            return None
        raise ValueError(f"{name} must contain {MIMIC_JOINT_COUNT} joint angles in radians")
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"{name} must contain {MIMIC_JOINT_COUNT} joint angles in radians"
        ) from error
    if result.shape != (MIMIC_JOINT_COUNT,):
        raise ValueError(
            f"{name} must have shape ({MIMIC_JOINT_COUNT},) in the order {MIMIC_JOINTS}"
        )
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite radians")
    result = result.copy()
    result.setflags(write=False)
    return result


def validate_mimic_joint_count(n_joints: int, *, name: str = "DMP model") -> int:
    """Require the four anatomical channels used by the mimic pipeline."""
    if isinstance(n_joints, bool) or not isinstance(n_joints, Integral):
        raise ValueError(f"{name} joint count must be an integer")
    result = int(n_joints)
    if result != MIMIC_JOINT_COUNT:
        raise ValueError(
            f"{name} must use {MIMIC_JOINT_COUNT} joints in the order {MIMIC_JOINTS}; "
            f"got {result}"
        )
    return result


def validate_mimic_trajectory(
    values: ArrayLike,
    *,
    name: str = "reference trajectory",
) -> NDArray[np.float64]:
    """Return a protected `(samples, four joints)` trajectory in radians."""
    try:
        result = np.asarray(values, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"{name} must be a two-dimensional trajectory in radians"
        ) from error
    expected = f"(samples, {MIMIC_JOINT_COUNT}) in the order {MIMIC_JOINTS}"
    if result.ndim != 2 or result.shape[0] < 2 or result.shape[1] != MIMIC_JOINT_COUNT:
        raise ValueError(f"{name} must have shape {expected} with at least two samples")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite radians")
    result = result.copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class HumanMotionSample:
    """One human joint-space observation with explicit validity and provenance."""

    time_s: float
    angles_rad: ArrayLike | None
    valid: bool = True
    source: str = "unknown"
    status: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")
        object.__setattr__(self, "time_s", _finite_nonnegative(self.time_s, name="time_s"))
        object.__setattr__(
            self,
            "angles_rad",
            _angles(
                self.angles_rad,
                name="angles_rad",
                allow_none=not self.valid,
            ),
        )
        object.__setattr__(self, "source", _text(self.source, name="source", allow_empty=False))
        object.__setattr__(self, "status", _text(self.status, name="status", allow_empty=True))


@dataclass(frozen=True)
class ProgressState:
    """Validated output of the future movement-progress tracker."""

    progress: float
    active_segment: int
    confidence: float
    state: MimicState | str
    status: str = ""

    def __post_init__(self) -> None:
        progress = _unit_interval(self.progress, name="progress")
        confidence = _unit_interval(self.confidence, name="confidence")
        state = _state(self.state)
        if isinstance(self.active_segment, bool) or not isinstance(self.active_segment, Integral):
            raise ValueError("active_segment must be a nonnegative integer")
        active_segment = int(self.active_segment)
        if active_segment < 0:
            raise ValueError("active_segment must be a nonnegative integer")
        if state is MimicState.COMPLETE and progress != 1:
            raise ValueError("complete progress state must have progress 1")
        object.__setattr__(self, "progress", progress)
        object.__setattr__(self, "active_segment", active_segment)
        object.__setattr__(self, "confidence", confidence)
        object.__setattr__(self, "state", state)
        object.__setattr__(self, "status", _text(self.status, name="status", allow_empty=True))


@dataclass(frozen=True)
class MimicTarget:
    """A phase-selected DMP target before mapping to the simulated robot."""

    progress: float
    angles_rad: ArrayLike | None
    state: MimicState | str
    status: str = ""

    def __post_init__(self) -> None:
        progress = _unit_interval(self.progress, name="progress")
        state = _state(self.state)
        allow_none = state in {MimicState.WAITING, MimicState.LOST}
        angles = _angles(self.angles_rad, name="angles_rad", allow_none=True)
        if angles is None and not allow_none:
            raise ValueError(f"{state.value} mimic target requires joint angles")
        if angles is not None and allow_none:
            raise ValueError(f"{state.value} mimic target must not contain joint angles")
        if state is MimicState.COMPLETE and progress != 1:
            raise ValueError("complete mimic target must have progress 1")
        object.__setattr__(self, "progress", progress)
        object.__setattr__(self, "angles_rad", angles)
        object.__setattr__(self, "state", state)
        object.__setattr__(self, "status", _text(self.status, name="status", allow_empty=True))
