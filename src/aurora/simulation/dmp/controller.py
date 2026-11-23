# Christian och Majd
"""Orchestrate the complete sensor-to-DMP Human Mimic pipeline."""

from __future__ import annotations

import logging

from .mimic import HumanMotionSample, MimicTarget, ProgressState
from .online import PhaseDrivenDMP
from .progress import ProgressTracker

LOGGER = logging.getLogger(__name__)


class HumanMimicController:
    """Join progress estimation and phase-driven DMP target selection.

    ``update`` is the single runtime entry point: it consumes one validated
    human observation and returns one human-space DMP target. Robot mapping,
    safety limits, GUI updates and PyBullet control remain later-stage concerns.
    """

    def __init__(
        self,
        progress_tracker: ProgressTracker,
        phase_dmp: PhaseDrivenDMP,
    ) -> None:
        if not isinstance(progress_tracker, ProgressTracker):
            raise TypeError("progress_tracker must be a ProgressTracker")
        if not isinstance(phase_dmp, PhaseDrivenDMP):
            raise TypeError("phase_dmp must be a PhaseDrivenDMP")
        self.progress_tracker = progress_tracker
        self.phase_dmp = phase_dmp
        self.reset()

    @property
    def last_progress(self) -> ProgressState:
        """The latest tracker result, including confidence and active segment."""
        return self._last_progress

    @property
    def last_target(self) -> MimicTarget:
        """The latest target returned to the next pipeline stage."""
        return self._last_target

    def reset(self) -> MimicTarget:
        """Start a new movement and reset both stateful components together."""
        self._last_progress = self.progress_tracker.reset()
        self._last_target = self.phase_dmp.reset()
        LOGGER.debug(
            "human_mimic reset state=%s progress=%.6f",
            self._last_target.state.value,
            self._last_target.progress,
        )
        return self._last_target

    def update(self, sample: HumanMotionSample) -> MimicTarget:
        """Process one human observation and return its phase-selected DMP target."""
        if not isinstance(sample, HumanMotionSample):
            raise TypeError("sample must be a HumanMotionSample")

        progress = self.progress_tracker.update(sample)
        target = self.phase_dmp.update(progress)
        self._last_progress = progress
        self._last_target = target

        if LOGGER.isEnabledFor(logging.DEBUG):
            target_angles = (
                None
                if target.angles_rad is None
                else tuple(round(float(value), 6) for value in target.angles_rad)
            )
            LOGGER.debug(
                "human_mimic time_s=%.6f source=%s state=%s progress=%.6f "
                "confidence=%.6f active_segment=%d target_rad=%s",
                sample.time_s,
                sample.source,
                target.state.value,
                target.progress,
                progress.confidence,
                progress.active_segment,
                target_angles,
            )
        return target
