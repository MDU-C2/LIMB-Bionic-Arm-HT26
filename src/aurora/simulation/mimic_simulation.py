# Christian och Majd
"""Run the complete Human Mimic pipeline on synthetic data in PyBullet."""

from __future__ import annotations

import argparse
import contextlib
import math
import time
from dataclasses import dataclass

import numpy as np
import pybullet as p

from aurora.recording.common import enable_ctrl_break_as_interrupt
from aurora.simulation.dmp.controller import HumanMimicController
from aurora.simulation.dmp.dmp import fit
from aurora.simulation.dmp.mimic import HumanMotionSample, validate_mimic_trajectory
from aurora.simulation.dmp.online import PhaseDrivenDMP
from aurora.simulation.dmp.progress import ProgressTracker
from aurora.simulation.dmp.simulator import MimicRobotAdapter
from aurora.simulation.interactive.arm_control import (
    AuroraArm,
    build_joint_index_map,
    get_angles_deg,
    sync_to_pybullet,
)
from aurora.simulation.interactive.settings import SIMULATION_FREQUENCY_HZ
from aurora.simulation.model.robot_model import load_left_arm


@dataclass(frozen=True)
class MimicSimulationResult:
    """Summary returned by a completed synthetic PyBullet validation."""

    steps: int
    progress: float
    state: str
    commanded_angles_deg: dict[str, float]
    robot_targets_deg: dict[str, float]


def synthetic_mimic_reference(sample_count: int = 121) -> np.ndarray:
    """Create a smooth, explicit non-participant four-joint exercise."""
    if isinstance(sample_count, bool) or not isinstance(sample_count, int) or sample_count < 11:
        raise ValueError("sample_count must be an integer of at least 11")
    phase = np.linspace(0.0, 1.0, sample_count)
    return np.column_stack(
        (
            0.25 + 0.62 * phase + 0.05 * np.sin(np.pi * phase),
            0.35 + 0.48 * phase,
            0.10 + 0.32 * np.sin(0.5 * np.pi * phase),
            0.18 * np.sin(np.pi * phase),
        )
    )


def build_synthetic_mimic_controller(
    reference_rad: np.ndarray | None = None,
) -> HumanMimicController:
    """Fit and connect the Step 1-4 components for simulator rehearsal."""
    reference = validate_mimic_trajectory(
        synthetic_mimic_reference() if reference_rad is None else reference_rad,
        name="synthetic Human Mimic reference",
    )
    tau = 1.0
    model = fit(
        [reference],
        tau=tau,
        dt=tau / (len(reference) - 1),
        n_basis_functions=20,
        alpha_canonical=4.0,
        alpha_transformation=25.0,
        beta_transformation=6.25,
    )
    return HumanMimicController(
        ProgressTracker(reference),
        PhaseDrivenDMP(model, reference[0], reference[-1], sample_count=501),
    )


def _angles_at(reference: np.ndarray, progress: float) -> np.ndarray:
    position = progress * (len(reference) - 1)
    left = int(math.floor(position))
    right = min(left + 1, len(reference) - 1)
    fraction = position - left
    return (1.0 - fraction) * reference[left] + fraction * reference[right]


def run_synthetic_mimic(
    *,
    duration_s: float = 4.0,
    mode: str = "kinematic",
    headless: bool = False,
) -> MimicSimulationResult:
    """Drive the left-arm PyBullet model from the complete synthetic pipeline."""
    if not math.isfinite(duration_s) or not 0.5 <= duration_s <= 30.0:
        raise ValueError("duration_s must be between 0.5 and 30 seconds")
    if mode not in {"kinematic", "dynamic"}:
        raise ValueError("mode must be kinematic or dynamic")
    if not isinstance(headless, bool):
        raise TypeError("headless must be a boolean")

    connection = p.connect(p.DIRECT if headless else p.GUI)
    if connection < 0:
        raise RuntimeError("Could not connect to PyBullet")

    reference = synthetic_mimic_reference()
    controller = build_synthetic_mimic_controller(reference)
    adapter = MimicRobotAdapter()
    arm = AuroraArm()
    step_s = 1.0 / SIMULATION_FREQUENCY_HZ
    total_steps = int(math.ceil(duration_s * SIMULATION_FREQUENCY_HZ))
    target = None
    try:
        if not headless:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.resetDebugVisualizerCamera(
                cameraDistance=1.25,
                cameraYaw=35,
                cameraPitch=-20,
                cameraTargetPosition=(0.0, 0.0, 0.65),
            )
        p.setGravity(0, 0, -9.81 if mode == "dynamic" else 0.0)
        p.setTimeStep(step_s)
        robot = load_left_arm(p, base_position=(0.0, 0.0, 0.0))
        joint_indices = build_joint_index_map(robot, p)

        initial_sample = HumanMotionSample(
            0.0,
            reference[0],
            source="synthetic-mimic-simulator",
        )
        target = controller.update(initial_sample)
        adapter.apply(arm, target, elapsed_s=step_s, speed_limited=False)
        sync_to_pybullet(arm, robot, joint_indices, client=p, use_motors=False)

        for step in range(1, total_steps + 1):
            if not p.isConnected():
                raise RuntimeError("PyBullet closed before the synthetic movement completed")
            elapsed_s = min(step * step_s, duration_s)
            progress = elapsed_s / duration_s
            sample = HumanMotionSample(
                elapsed_s,
                _angles_at(reference, progress),
                source="synthetic-mimic-simulator",
            )
            target = controller.update(sample)
            adapter.apply(arm, target, elapsed_s=step_s, speed_limited=True)
            sync_to_pybullet(
                arm,
                robot,
                joint_indices,
                client=p,
                use_motors=mode == "dynamic",
            )
            p.stepSimulation()
            if not headless:
                time.sleep(step_s)

        if target is None:  # pragma: no cover - loop always creates a target
            raise RuntimeError("Synthetic Human Mimic produced no target")
        robot_targets = adapter.last_robot_targets
        if robot_targets is None:
            raise RuntimeError("Synthetic Human Mimic produced no robot target")
        result = MimicSimulationResult(
            steps=total_steps,
            progress=target.progress,
            state=target.state.value,
            commanded_angles_deg=get_angles_deg(arm),
            robot_targets_deg=robot_targets,
        )
        print(
            f"Human Mimic simulation: {result.state}, "
            f"progress={result.progress * 100:.1f}%, steps={result.steps}"
        )
        if not headless:
            print("Movement complete. Close the PyBullet window to stop.")
            while p.isConnected():
                p.stepSimulation()
                time.sleep(step_s)
        return result
    finally:
        if p.isConnected():
            p.disconnect()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration",
        type=float,
        default=4.0,
        help="Synthetic human movement duration in seconds",
    )
    parser.add_argument(
        "--mode",
        choices=("kinematic", "dynamic"),
        default="kinematic",
        help="Direct pose preview or motor physics",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Validate once without opening the PyBullet window",
    )
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 0.5 <= args.duration <= 30.0:
        parser.error("--duration must be between 0.5 and 30 seconds")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    enable_ctrl_break_as_interrupt(stdin_stop=True)
    try:
        run_synthetic_mimic(
            duration_s=args.duration,
            mode=args.mode,
            headless=args.headless,
        )
        return 0
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        print(f"Human Mimic simulation failed: {error}")
        return 1


if __name__ == "__main__":
    with contextlib.suppress(KeyboardInterrupt):
        raise SystemExit(main())
