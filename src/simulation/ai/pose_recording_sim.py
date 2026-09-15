"""Play a camera pose recording on the limited LIMB arm model."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import pybullet as p


AI_DIR = Path(__file__).resolve().parent
SIMULATION_ROOT = AI_DIR.parent
SIM_DIR = SIMULATION_ROOT / "sim"
if str(SIMULATION_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATION_ROOT))
if str(AI_DIR) not in sys.path:
    sys.path.insert(0, str(AI_DIR))

from pose_mapping import (
    CameraArmMapper,
    FingerMotionLimiter,
    MotionSmoother,
    estimate_finger_curls,
)
from sim.limb_sim import JOINT_MAPPING, joint_index


FINGER_JOINTS = {
    "thumb": ("thumb_joint_1", "thumb_joint_2", "thumb_joint_3"),
    "index": ("index_joint_1", "index_joint_2"),
    "middle": ("middle_joint_1", "middle_joint_2"),
    "ring": ("ring_joint_1", "ring_joint_2", "ring_joint_3"),
    "pinky": ("pinky_joint_1", "pinky_joint_2", "pinky_joint_3"),
}
FINGER_MODEL_RANGE_RAD = 1.5


def load_frames(path: Path) -> list[dict[str, object]]:
    """Read frames from the pose-recording JSON format."""
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"Could not read {path}: {error}") from error
    frames = value.get("data") if isinstance(value, dict) else None
    if not isinstance(frames, list) or not frames:
        raise ValueError("Recording must contain a non-empty data list")
    return frames


def wrist_from_hand(hand: object) -> list[object] | None:
    """Return hand landmark zero as the wrist point."""
    if not isinstance(hand, list):
        return None
    for point in hand:
        if not isinstance(point, dict):
            continue
        try:
            is_wrist = int(point.get("id", -1)) == 0
        except (TypeError, ValueError):
            is_wrist = False
        if is_wrist:
            return [point.get("x"), point.get("y"), point.get("depth_m")]
    return None


def load_robot(headless: bool) -> tuple[int, list[int], dict[str, tuple[int, ...]]]:
    """Open PyBullet and return named arm and finger joints."""
    connection = p.connect(p.DIRECT if headless else p.GUI)
    if connection < 0:
        raise RuntimeError("Could not connect to PyBullet")
    p.setGravity(0, 0, 0)
    if not headless:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(1.7, 145, -12, [0.0, 0.25, 0.25])

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

    for index in range(p.getNumJoints(robot)):
        p.setJointMotorControl2(robot, index, p.POSITION_CONTROL, force=0.0)
    arm_joints = [joint_index(robot, urdf_name) for _, urdf_name, _ in JOINT_MAPPING]
    finger_joints = {
        name: tuple(joint_index(robot, joint_name) for joint_name in names)
        for name, names in FINGER_JOINTS.items()
    }
    return robot, arm_joints, finger_joints


def main() -> None:
    """Parse playback options and animate one pose recording."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps must be positive")
    if args.loop and args.headless:
        parser.error("--loop and --headless cannot be combined")

    frames = load_frames(args.recording)
    dt = 1.0 / args.fps
    mapper = CameraArmMapper()
    smoother = MotionSmoother()
    finger_limiter = FingerMotionLimiter()
    robot, arm_joints, finger_joints = load_robot(args.headless)
    print(f"Loaded {len(frames)} pose frames from {args.recording}")
    skipped = 0

    try:
        while True:
            for frame in frames:
                if not p.isConnected():
                    return
                if not isinstance(frame, dict):
                    skipped += 1
                    p.stepSimulation()
                    continue
                wrist = wrist_from_hand(frame.get("hand"))
                angles = mapper.calculate(frame.get("shoulder"), frame.get("elbow"), wrist)
                if angles is None:
                    skipped += 1
                else:
                    command = smoother.step(angles, dt)
                    for column, joint in enumerate(arm_joints):
                        p.resetJointState(
                            robot,
                            joint,
                            float(command[column] * JOINT_MAPPING[column][2]),
                        )

                finger_targets = estimate_finger_curls(frame.get("hand"))
                finger_commands = finger_limiter.step(finger_targets, dt)
                for name, joints in finger_joints.items():
                    for joint in joints:
                        p.resetJointState(
                            robot,
                            joint,
                            finger_commands[name] * FINGER_MODEL_RANGE_RAD,
                        )
                p.stepSimulation()
                if not args.headless:
                    time.sleep(dt)
            if not args.loop:
                break

        print(f"Playback finished; skipped {skipped} invalid frame(s)")
        if not args.headless:
            print("Close the simulation window or stop the program from the GUI")
            while p.isConnected():
                time.sleep(0.05)
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        raise SystemExit(f"Error: {error}") from error
