"""Open the LIMB PyBullet model with arm and wrist joint sliders.

Adapted from ``sim/sandbox_limb_sim.py`` in Oscar Agren's DMP-arm project at
commit 88b0d2ea5df63764ebb56aad8926f92084926fc7.  This version uses the limits
and joint mapping selected for the AURORA simulation baseline.
"""

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
if str(SIMULATION_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATION_ROOT))

from sim.joint_limits import HARDWARE_JOINT_LIMITS_DEG, JOINT_LIMITS_RAD
from sim.limb_sim import JOINT_MAPPING, joint_index


DEFAULT_ANGLES_DEG = (36.0, 37.47, 13.26, -18.95)
SLIDER_ORDER = (1, 2, 3, 0)  # Shoulder controls first, matching Oscar's UI.


def main() -> None:
    """Open the left-arm model and update it from PyBullet sliders."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--step-time",
        type=float,
        default=1.0 / 120.0,
        help="Seconds between visual updates (default: 1/120)",
    )
    args = parser.parse_args()
    if not math.isfinite(args.step_time) or args.step_time <= 0:
        parser.error("--step-time must be finite and positive")

    connection = p.connect(p.GUI)
    if connection < 0:
        raise RuntimeError("Could not connect to PyBullet")

    try:
        for preview in (
            p.COV_ENABLE_RGB_BUFFER_PREVIEW,
            p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
            p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
        ):
            p.configureDebugVisualizer(preview, 0)
        p.resetDebugVisualizerCamera(
            cameraDistance=0.95,
            cameraYaw=42,
            cameraPitch=-24,
            cameraTargetPosition=(0.0, 0.0, -0.30),
        )

        # PyBullet on Windows cannot open the non-ASCII parent path directly.
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
        wrist_joint = joint_index(robot, "jLeftElbow_rotz")
        limits_deg = np.rad2deg(JOINT_LIMITS_RAD)
        sliders: dict[int, int] = {}
        for column in SLIDER_ORDER:
            logical_name, urdf_name, _ = JOINT_MAPPING[column]
            lower, upper = limits_deg[column]
            default = float(np.clip(DEFAULT_ANGLES_DEG[column], lower, upper))
            label = f"{logical_name} ({urdf_name}) [deg]"
            sliders[column] = p.addUserDebugParameter(
                label, float(lower), float(upper), default
            )
        wrist_limit = HARDWARE_JOINT_LIMITS_DEG["lower_arm_rotation"]
        wrist_slider = p.addUserDebugParameter(
            "wrist_rotation (jLeftElbow_rotz) [deg]",
            wrist_limit.lower,
            wrist_limit.upper,
            wrist_limit.home,
        )

        print("Manual joint control ready. Close the PyBullet window to stop.")
        for logical_name, urdf_name, sign in JOINT_MAPPING:
            print(f"  {logical_name} -> {urdf_name} (sign {sign:+.0f})")

        while p.isConnected():
            try:
                angles_deg = np.array(
                    [
                        p.readUserDebugParameter(sliders[column])
                        for column in range(len(JOINT_MAPPING))
                    ],
                    dtype=float,
                )
                for column, joint in enumerate(joint_ids):
                    command = math.radians(float(angles_deg[column])) * JOINT_MAPPING[column][2]
                    p.resetJointState(robot, joint, command)
                wrist_angle = float(p.readUserDebugParameter(wrist_slider))
                p.resetJointState(robot, wrist_joint, math.radians(wrist_angle))
                p.performCollisionDetection()
                time.sleep(args.step_time)
            except p.error:
                break

    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
