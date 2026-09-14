"""Open the LIMB PyBullet model with sliders for the four mapped joints.

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

from sim.joint_limits import DMP_JOINT_NAMES, HARDWARE_JOINT_LIMITS_DEG, JOINT_LIMITS_RAD
from sim.limb_sim import JOINT_MAPPING, joint_index


DEFAULT_ANGLES_DEG = (36.0, 37.47, 13.26, -18.95)
SLIDER_ORDER = (1, 2, 3, 0)  # Shoulder controls first, matching Oscar's UI.


def main() -> None:
    """Open the left-arm model and update it from PyBullet sliders."""
    # This entry point is launched by the GUI and can also be tuned directly.
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
        p.setGravity(0, 0, 0)
        p.setTimeStep(args.step_time)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.25,
            cameraYaw=42,
            cameraPitch=-24,
            cameraTargetPosition=(0.0, 0.0, -0.15),
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
        for index in range(p.getNumJoints(robot)):
            p.setJointMotorControl2(robot, index, p.POSITION_CONTROL, force=0.0)

        # Generate controls from the same limits used by trajectory playback.
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

        instruction_id = p.addUserDebugText(
            "Use the Params sliders to set joint angles (degrees).",
            (0.10, 0.05, 0.20),
            textColorRGB=(1.0, 1.0, 1.0),
            textSize=1.25,
        )
        angle_text_id = -1

        print("Manual joint control ready. Close the PyBullet window to stop.")
        for logical_name, urdf_name, sign in JOINT_MAPPING:
            print(f"  {logical_name} -> {urdf_name} (sign {sign:+.0f})")

        # PyBullet owns the sliders; this loop only mirrors them into the model.
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
                    current_command = p.getJointState(robot, joint)[0]
                    physical_difference = (
                        command - current_command
                    ) * JOINT_MAPPING[column][2]
                    limit = HARDWARE_JOINT_LIMITS_DEG[DMP_JOINT_NAMES[column]]
                    p.setJointMotorControl2(
                        robot,
                        joint,
                        p.POSITION_CONTROL,
                        targetPosition=command,
                        force=200.0,
                        maxVelocity=math.radians(limit.speed(physical_difference)),
                    )

                angle_lines = ["Current angles (deg)"]
                angle_lines.extend(
                    f"{JOINT_MAPPING[i][0]}: {angles_deg[i]:7.2f}"
                    for i in range(len(JOINT_MAPPING))
                )
                angle_text_id = p.addUserDebugText(
                    "\n".join(angle_lines),
                    (0.10, 0.05, 0.12),
                    textColorRGB=(1.0, 1.0, 1.0),
                    textSize=1.1,
                    replaceItemUniqueId=angle_text_id,
                )
                p.stepSimulation()
                time.sleep(args.step_time)
            except p.error:
                break

        # Keep the reference alive until after the loop for older PyBullet builds.
        _ = instruction_id
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
