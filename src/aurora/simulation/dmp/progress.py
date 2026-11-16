# Christian och Majd
"""Online movement progress estimation for Human Mimic."""

from __future__ import annotations

import math
from numbers import Integral

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .mimic import HumanMotionSample, MimicState, ProgressState, validate_mimic_trajectory


def _finite_float(value: float, *, name: str, minimum: float, inclusive: bool) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    valid_minimum = result >= minimum if inclusive else result > minimum
    if not math.isfinite(result) or not valid_minimum:
        comparison = "at least" if inclusive else "greater than"
        raise ValueError(f"{name} must be finite and {comparison} {minimum}")
    return result


class ProgressTracker:
    """Estimate monotonic progress through a four-joint reference movement.

    Each human observation is projected onto a small forward window of the
    piecewise-linear reference trajectory. Progress is the projected segment
    position divided by the total number of segments. The window prevents a
    self-intersection later in the movement from causing a large phase jump.

    Timeout and confidence defaults are development values for synthetic data.
    They must be tuned when the real camera/IMU data is available.
    """

    _SEGMENT_EPSILON = 1e-12

    def __init__(
        self,
        reference_rad: ArrayLike,
        *,
        look_ahead_segments: int = 12,
        stationary_timeout_s: float = 0.30,
        lost_timeout_s: float = 0.75,
        confidence_scale_rad: float = 0.35,
        minimum_confidence: float = 0.10,
        progress_epsilon: float = 1e-4,
    ) -> None:
        reference = validate_mimic_trajectory(reference_rad)
        self._reference_rad = self._remove_consecutive_duplicates(reference)
        self._segment_count = self._reference_rad.shape[0] - 1

        if isinstance(look_ahead_segments, bool) or not isinstance(
            look_ahead_segments, Integral
        ):
            raise ValueError("look_ahead_segments must be a positive integer")
        self.look_ahead_segments = int(look_ahead_segments)
        if self.look_ahead_segments < 1:
            raise ValueError("look_ahead_segments must be a positive integer")

        self.stationary_timeout_s = _finite_float(
            stationary_timeout_s,
            name="stationary_timeout_s",
            minimum=0,
            inclusive=True,
        )
        self.lost_timeout_s = _finite_float(
            lost_timeout_s,
            name="lost_timeout_s",
            minimum=0,
            inclusive=False,
        )
        self.confidence_scale_rad = _finite_float(
            confidence_scale_rad,
            name="confidence_scale_rad",
            minimum=0,
            inclusive=False,
        )
        self.minimum_confidence = _finite_float(
            minimum_confidence,
            name="minimum_confidence",
            minimum=0,
            inclusive=False,
        )
        if self.minimum_confidence > 1:
            raise ValueError("minimum_confidence must not exceed 1")
        self.progress_epsilon = _finite_float(
            progress_epsilon,
            name="progress_epsilon",
            minimum=0,
            inclusive=False,
        )
        if self.progress_epsilon >= 1:
            raise ValueError("progress_epsilon must be less than 1")

        self.reset()

    @staticmethod
    def _remove_consecutive_duplicates(
        reference: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        kept = [reference[0]]
        for waypoint in reference[1:]:
            delta = waypoint - kept[-1]
            if float(delta @ delta) > ProgressTracker._SEGMENT_EPSILON:
                kept.append(waypoint)
        if len(kept) < 2:
            raise ValueError("reference trajectory must contain movement")
        result = np.asarray(kept, dtype=np.float64)
        result.setflags(write=False)
        return result

    @property
    def reference_rad(self) -> NDArray[np.float64]:
        """The protected reference waypoints used by the tracker."""
        return self._reference_rad

    @property
    def progress(self) -> float:
        return self._progress

    def reset(self) -> ProgressState:
        """Clear runtime state so a new movement can start from zero."""
        self._progress = 0.0
        self._active_segment = 0
        self._last_time_s: float | None = None
        self._last_progress_time_s: float | None = None
        self._missing_since_s: float | None = None
        self._has_match = False
        self._complete = False
        self._last_confidence = 0.0
        self._state = MimicState.WAITING
        self._status = "waiting for human movement"
        return self.snapshot()

    def snapshot(self) -> ProgressState:
        """Return the current state without consuming a new observation."""
        return ProgressState(
            self._progress,
            self._active_segment,
            self._last_confidence,
            self._state,
            self._status,
        )

    def update(self, sample: HumanMotionSample) -> ProgressState:
        """Consume one timestamped human observation and return progress."""
        if not isinstance(sample, HumanMotionSample):
            raise TypeError("sample must be a HumanMotionSample")
        if self._last_time_s is not None and sample.time_s < self._last_time_s:
            raise ValueError("sample timestamps must not move backwards")
        self._last_time_s = sample.time_s

        if self._complete:
            return self.snapshot()
        if not sample.valid or sample.angles_rad is None:
            return self._handle_missing(sample.time_s, 0.0, "invalid human sample")

        candidate, segment, local_progress, distance = self._match(sample.angles_rad)
        confidence = math.exp(-((distance / self.confidence_scale_rad) ** 2))
        if confidence < self.minimum_confidence:
            return self._handle_missing(
                sample.time_s,
                confidence,
                "human pose is outside the reference path",
            )

        self._missing_since_s = None
        previous_progress = self._progress
        self._progress = max(self._progress, candidate)
        self._last_confidence = confidence

        if candidate + self.progress_epsilon >= previous_progress:
            self._active_segment = max(self._active_segment, segment)
            if local_progress >= 1 - self.progress_epsilon:
                self._active_segment = min(
                    self._active_segment + 1,
                    self._segment_count - 1,
                )

        if not self._has_match:
            self._has_match = True
            self._last_progress_time_s = sample.time_s
        elif self._progress - previous_progress > self.progress_epsilon:
            self._last_progress_time_s = sample.time_s

        if self._progress >= 1 - self.progress_epsilon:
            self._progress = 1.0
            self._active_segment = self._segment_count - 1
            self._complete = True
            return self._result(MimicState.COMPLETE, "movement complete")

        stationary_s = sample.time_s - float(self._last_progress_time_s)
        if stationary_s >= self.stationary_timeout_s:
            state = MimicState.HOLDING
            status = "human movement paused"
        else:
            state = MimicState.RUNNING
            status = "reference movement matched"
        return self._result(state, status)

    def _match(
        self,
        angles_rad: NDArray[np.float64],
    ) -> tuple[float, int, float, float]:
        end_segment = min(
            self._active_segment + self.look_ahead_segments,
            self._segment_count - 1,
        )
        best: tuple[float, float, int, float] | None = None

        for segment in range(self._active_segment, end_segment + 1):
            start = self._reference_rad[segment]
            direction = self._reference_rad[segment + 1] - start
            length_squared = float(direction @ direction)
            local_progress = float(
                np.clip(((angles_rad - start) @ direction) / length_squared, 0.0, 1.0)
            )
            projection = start + local_progress * direction
            distance = float(np.linalg.norm(angles_rad - projection))
            candidate = (segment + local_progress) / self._segment_count

            # Distance is primary. Earlier progress wins an exact tie, which
            # avoids jumping at a self-intersection in the reference path.
            match = (distance, candidate, segment, local_progress)
            if best is None or match[:2] < best[:2]:
                best = match

        if best is None:  # pragma: no cover - construction guarantees segments
            raise RuntimeError("reference trajectory has no searchable segment")
        distance, candidate, segment, local_progress = best
        return candidate, segment, local_progress, distance

    def _handle_missing(
        self,
        time_s: float,
        confidence: float,
        reason: str,
    ) -> ProgressState:
        if self._missing_since_s is None:
            self._missing_since_s = time_s
        self._last_confidence = confidence
        missing_s = time_s - self._missing_since_s

        if missing_s >= self.lost_timeout_s:
            state = MimicState.LOST
            status = f"tracking lost: {reason}"
        elif self._has_match:
            state = MimicState.HOLDING
            status = f"holding last progress: {reason}"
        else:
            state = MimicState.WAITING
            status = f"waiting for valid input: {reason}"
        return self._result(state, status)

    def _result(self, state: MimicState, status: str) -> ProgressState:
        self._state = state
        self._status = status
        return ProgressState(
            self._progress,
            self._active_segment,
            self._last_confidence,
            state,
            status,
        )
