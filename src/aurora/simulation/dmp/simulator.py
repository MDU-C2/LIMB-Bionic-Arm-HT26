# Christian och Majd
"""Map Human Mimic DMP targets onto the maintained left-arm simulator."""

from __future__ import annotations

import math

from aurora.sensors.fusion import interactive_control_targets
from aurora.simulation.interactive.arm_control import (
    AuroraArm,
    set_elbow,
    set_shoulder,
    step_toward,
)

from .mimic import MIMIC_JOINTS, MimicTarget


def mimic_robot_targets(target: MimicTarget) -> dict[str, float] | None:
    """Convert a human-space DMP target in radians to limited robot degrees."""
    if not isinstance(target, MimicTarget):
        raise TypeError("target must be a MimicTarget")
    if target.angles_rad is None:
        return None
    human_degrees = {
        name: math.degrees(float(angle))
        for name, angle in zip(MIMIC_JOINTS, target.angles_rad, strict=True)
    }
    return interactive_control_targets(human_degrees)


class MimicRobotAdapter:
    """Apply Human Mimic targets using existing robot limits and conventions."""

    def __init__(self) -> None:
        self._last_robot_targets: dict[str, float] | None = None

    @property
    def last_robot_targets(self) -> dict[str, float] | None:
        return None if self._last_robot_targets is None else dict(self._last_robot_targets)

    def reset(self) -> None:
        self._last_robot_targets = None

    def apply(
        self,
        arm: AuroraArm,
        target: MimicTarget,
        *,
        elapsed_s: float,
        speed_limited: bool = True,
    ) -> dict[str, float] | None:
        """Update the arm command model, or hold it when no target is available."""
        if not isinstance(arm, AuroraArm):
            raise TypeError("arm must be an AuroraArm")
        if not isinstance(speed_limited, bool):
            raise TypeError("speed_limited must be a boolean")
        if not math.isfinite(elapsed_s) or elapsed_s <= 0:
            raise ValueError("elapsed_s must be finite and positive")

        robot_targets = mimic_robot_targets(target)
        if robot_targets is None:
            return None

        def command(current: float, name: str) -> float:
            requested = robot_targets[name]
            if not speed_limited:
                return requested
            return step_toward(current, requested, name, elapsed_s=elapsed_s)

        set_shoulder(
            arm,
            x=command(arm.shoulder.angle_x, "shoulder_x"),
            y=command(arm.shoulder.angle_y, "shoulder_y"),
            z=command(arm.shoulder.angle_z, "shoulder_z"),
            mode="abs",
        )
        set_elbow(
            arm,
            x=command(arm.elbow.angle_x, "elbow_x"),
            mode="abs",
        )
        self._last_robot_targets = dict(robot_targets)
        return dict(robot_targets)
