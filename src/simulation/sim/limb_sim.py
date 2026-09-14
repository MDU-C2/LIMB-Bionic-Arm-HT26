"""Play a four-joint DMP trajectory on Oscar Ågren's LIMB PyBullet model."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import pybullet as p


SIM_DIR = Path(__file__).resolve().parent
SIMULATION_ROOT = SIM_DIR.parent
REPOSITORY_ROOT = SIM_DIR.parents[2]
if str(SIMULATION_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATION_ROOT))

from dmp.trajectory_io import load_dmp_trajectory, resolve_saved_dmp_rollout_path
from sim.joint_limits import DMP_JOINT_NAMES, HARDWARE_JOINT_LIMITS_DEG, clamp_dmp_vector


EXAMPLE_PATH = REPOSITORY_ROOT / "examples" / "simulation" / "demo"
JOINT_MAPPING = (
    ("elbow_flexion", "jLeftElbow_roty", 1.0),
    ("shoulder_flexion", "jLeftShoulder_roty", 1.0),
    ("shoulder_abduction", "jLeftShoulder_rotz", -1.0),
    ("shoulder_internal_rotation", "jLeftShoulder_rotx", 1.0),
)


def joint_index(body_uid: int, name: str) -> int:
    """Return the PyBullet index for a named URDF joint."""
    for index in range(p.getNumJoints(body_uid)):
        if p.getJointInfo(body_uid, index)[1].decode("utf-8") == name:
            return index
    raise KeyError(f"Joint not found in URDF: {name}")


def speed_limited_step_times(
    trajectory: np.ndarray,
    requested_dt: float,
) -> np.ndarray:
    """Return a safe interval for each trajectory sample."""
    step_times = np.full(len(trajectory), requested_dt, dtype=float)
    if len(trajectory) < 2:
        return step_times

    differences = np.diff(trajectory, axis=0)
    for column, name in enumerate(DMP_JOINT_NAMES):
        limit = HARDWARE_JOINT_LIMITS_DEG[name]
        positive_speed = math.radians(limit.speed_positive)
        negative_speed = math.radians(limit.speed_negative)
        speeds = np.where(differences[:, column] >= 0, positive_speed, negative_speed)
        step_times[1:] = np.maximum(
            step_times[1:],
            np.abs(differences[:, column]) / speeds,
        )
    return step_times


def main() -> None:
    """Parse options, load a trajectory, and play it in PyBullet."""
    # Keep the CLI usable both from the GUI and for repeatable terminal runs.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path", type=Path, default=EXAMPLE_PATH)
    parser.add_argument(
        "--source",
        choices=["clean", "raw"],
        default="clean",
        help="Prefer clean or raw input, falling back to compatible available data",
    )
    parser.add_argument(
        "--refit",
        action="store_true",
        help="Fit a DMP from angles.npz instead of loading the saved rollout",
    )
    parser.add_argument("--loop", action="store_true")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run once without opening a window or waiting in real time",
    )
    args = parser.parse_args()
    if args.loop and args.headless:
        parser.error("--loop and --headless cannot be combined")
    if not args.path.is_dir():
        raise FileNotFoundError(f"Trial directory not found: {args.path}")

    # Resolve and validate all data before opening a PyBullet window.
    saved = resolve_saved_dmp_rollout_path(args.path, args.source)
    if saved is not None and not args.refit:
        expected_suffix = f"dmp_rollout_{args.source}.npz"
        if not saved.name.endswith(expected_suffix):
            print(f"Requested {args.source} rollout not found; using {saved.name}")
        print(f"Loading saved DMP rollout: {saved}")
    else:
        print(
            f"Fitting a DMP from {args.source}-preferred angles "
            "(15 basis functions, one-second rollout)"
        )
    requested, dt = load_dmp_trajectory(
        args.path,
        rollout_source=args.source,
        prefer_saved_rollout=not args.refit,
    )
    trajectory = clamp_dmp_vector(requested)
    step_times = speed_limited_step_times(trajectory, dt)
    clipped = int(np.count_nonzero(requested != trajectory))
    print(f"Trajectory: {len(trajectory)} samples, dt={dt:.6f} s; {clipped} values clipped")
    slowed_steps = int(np.count_nonzero(step_times > dt * 1.000001))
    if slowed_steps:
        print(
            f"Playback duration increased from {len(trajectory) * dt:.2f} s to "
            f"{float(np.sum(step_times)):.2f} s for motor speeds "
            f"({slowed_steps} slowed steps)"
        )
    if clipped:
        clipped_by_joint = np.count_nonzero(requested != trajectory, axis=0)
        details = ", ".join(
            f"{JOINT_MAPPING[index][0]}={int(count)}"
            for index, count in enumerate(clipped_by_joint)
            if count
        )
        print(f"Clipped by joint: {details}")
    if args.loop:
        restart_jump_deg = np.rad2deg(np.abs(trajectory[0] - trajectory[-1]))
        discontinuities = ", ".join(
            f"{JOINT_MAPPING[index][0]}={jump:.1f} deg"
            for index, jump in enumerate(restart_jump_deg)
            if jump > 5.0
        )
        if discontinuities:
            print(f"Warning: loop restart has a visible jump ({discontinuities})")

    connection = p.connect(p.DIRECT if args.headless else p.GUI)
    if connection < 0:
        raise RuntimeError("Could not connect to PyBullet")
    try:
        p.setGravity(0, 0, 0)
        # PyBullet on Windows cannot open the non-ASCII parent path directly.
        # Loading from this directory keeps all URDF and mesh paths ASCII-only.
        previous_directory = Path.cwd()
        os.chdir(SIM_DIR)
        try:
            robot = p.loadURDF(
                "arm/left_arm.urdf",
                baseOrientation=p.getQuaternionFromEuler((math.pi / 2, 0, math.pi / 2)),
                useFixedBase=True,
            )
        finally:
            os.chdir(previous_directory)
        joint_ids = [joint_index(robot, urdf_name) for _, urdf_name, _ in JOINT_MAPPING]
        for index in range(p.getNumJoints(robot)):
            p.setJointMotorControl2(robot, index, p.POSITION_CONTROL, force=0.0)

        print("Playing:")
        for logical, urdf_name, sign in JOINT_MAPPING:
            print(f"  {logical} -> {urdf_name} (sign {sign:+.0f})")
        # Resetting joint state gives deterministic playback of measured data.
        while True:
            for sample, step_time in zip(trajectory, step_times):
                if not p.isConnected():
                    return
                p.setTimeStep(float(step_time))
                for column, joint in enumerate(joint_ids):
                    command = float(sample[column] * JOINT_MAPPING[column][2])
                    p.resetJointState(robot, joint, command)
                p.stepSimulation()
                if not args.headless:
                    time.sleep(float(step_time))
            if not args.loop:
                break
        print("Playback finished")
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
