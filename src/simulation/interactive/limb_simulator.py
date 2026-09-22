"""Interactive LIMB arm task simulation migrated from LIMB-HT25.

The simulator opens a PyBullet task scene, a Pygame control HUD, and a Tkinter
sensor dashboard. Paths are resolved from this file so it works from any clone
location, including Windows repositories whose parent path contains non-ASCII
characters.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

SIMULATION_ROOT = Path(__file__).resolve().parents[1]
if str(SIMULATION_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATION_ROOT))

import pybullet as p
import pybullet_data
import pygame
import tkinter as tk
from kinematics import calculate_ik_angles, is_reachable
from sim.controller_params import (
    ARM_ACTUATOR_INFO, ARM_MOTOR_FORCE_NM, ARM_POSITION_GAIN, ARM_VELOCITY_GAIN,
    FINGER_MOTOR_FORCE_NM, FINGER_POSITION_GAIN, FINGER_VELOCITY_GAIN,
    FINGER_PREVIEW_SPEED_RAD_S,
)
from sim.contact_feedback import estimate_contact_force_n, grasp_is_ready
from sim.dynamics import ArmDynamics
from sim.joint_limits import (
    RIGHT_ARM_HARDWARE_SIGN,
    RIGHT_ARM_LIMITS_DEG,
    finger_joint_angles_rad,
    max_velocity_rad_s,
)
from sim.robot_model import RIGHT_GRIP_OFFSET_M
from torque_graph import TorqueGraph


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--mode", choices=("kinematic", "dynamic"), default="kinematic",
                    help="Direct joint preview or gravity-and-motor physics mode")
parser.add_argument("--telemetry-out", type=Path,
                    help="Write dynamic-mode state and torque snapshots as JSON lines")
parser.add_argument("--torque-out", type=Path,
                    help="Write a readable dynamic-mode arm torque CSV")
args = parser.parse_args()
DYNAMIC_MODE = args.mode == "dynamic"
if (args.telemetry_out is not None or args.torque_out is not None) and not DYNAMIC_MODE:
    parser.error("--telemetry-out and --torque-out require --mode dynamic")


SIMULATION_FREQUENCY_HZ = 60
PHYSICS_SUBSTEPS = 4
TARGET_RADIUS_M = 0.04
TARGET_HEIGHT_M = 0.10
SAFE_REACH_DISTANCE_M = 0.10
# The HUD rounds to two decimals. A little margin avoids showing 0.20 m while
# rejecting the manual F grip because the unrounded distance is just above it.
GRASP_CAPTURE_DISTANCE_M = 0.25
TARGET_NORMAL_MASS_KG = 0.1
HAND_GRIP_OFFSET_M = RIGHT_GRIP_OFFSET_M
MIN_GRASP_CURL = 0.45
MAX_GRASP_CURL = 1.5
MIN_GRASP_FINGERTIPS = 2
FINGERTIP_CONTACT_FORCE_N = 0.15
CONTACT_STIFFNESS_N_PER_M = 500.0
AUTO_GRASP_CURL_PER_SECOND = 0.6
AUTO_GRASP_WRIST_DEG = 20.0

FINGER_CURL_PER_SECOND = 3.0
FINGER_JOINT_TOKENS = ("thumb", "index", "middle", "ring", "pinky")

READY_POSE_DEG = {
    "shoulder_x": 0.0,
    "shoulder_y": 15.0,
    "shoulder_z": -18.0,
    "elbow_x": -20.0,
    "wrist_rotation": 70.0,
}
CAMERA_MODES = ("overview", "shoulder", "hand")


# Arm commands


def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp a numeric value to an inclusive range."""
    return max(min_value, min(max_value, value))


def deg_to_rad(deg: float) -> float:
    """Convert degrees to radians."""
    return deg * math.pi / 180.0


def rad_to_deg(rad: float) -> float:
    """Convert radians to degrees."""
    return rad * 180.0 / math.pi


def hardware_value(logical_name: str, value: float) -> float:
    """Convert a right-arm URDF value to the physical actuator sign."""
    return value * RIGHT_ARM_HARDWARE_SIGN[logical_name]


def step_toward(current: float, target: float, joint_name: str, scale: float = 1.0) -> float:
    """Move a command toward a target without exceeding its actuator speed."""
    limit = RIGHT_ARM_LIMITS_DEG[joint_name]
    target = limit.clamp(target)
    difference = target - current
    if difference == 0:
        return current
    max_step = limit.speed(difference) * scale / SIMULATION_FREQUENCY_HZ
    return current + clamp(difference, -max_step, max_step)


class ManualMotion:
    """Turn held keys into smooth, firmware-limited joint movement."""

    def __init__(self) -> None:
        self.velocity_deg_s = {name: 0.0 for name in RIGHT_ARM_LIMITS_DEG}

    def reset(self) -> None:
        for name in self.velocity_deg_s:
            self.velocity_deg_s[name] = 0.0

    def step(
        self,
        keys,
        positive_key: int,
        negative_key: int,
        joint_name: str,
        current_angle: float,
        speed_scale: float,
    ) -> float:
        direction = int(bool(keys[positive_key])) - int(bool(keys[negative_key]))
        limit = RIGHT_ARM_LIMITS_DEG[joint_name]
        target_velocity = direction * limit.speed(direction) * speed_scale
        current_velocity = self.velocity_deg_s[joint_name]

        if limit.acceleration is None:
            current_velocity = target_velocity
        else:
            acceleration_step = limit.acceleration / SIMULATION_FREQUENCY_HZ
            current_velocity += clamp(
                target_velocity - current_velocity,
                -acceleration_step,
                acceleration_step,
            )

        next_angle = limit.clamp(
            current_angle + current_velocity / SIMULATION_FREQUENCY_HZ
        )
        if next_angle == current_angle and current_velocity != 0.0:
            current_velocity = 0.0

        self.velocity_deg_s[joint_name] = current_velocity
        return next_angle - current_angle


class Shoulder:
    """Commands for the three shoulder actuators."""

    def __init__(self) -> None:
        """Initialize the shoulder in its neutral pose."""
        x_limit = RIGHT_ARM_LIMITS_DEG["shoulder_x"]
        y_limit = RIGHT_ARM_LIMITS_DEG["shoulder_y"]
        z_limit = RIGHT_ARM_LIMITS_DEG["shoulder_z"]
        self.angle_x = x_limit.home
        self.angle_y = y_limit.home
        self.angle_z = z_limit.home
        self.min_angle_x, self.max_angle_x = x_limit.lower, x_limit.upper
        self.min_angle_y, self.max_angle_y = y_limit.lower, y_limit.upper
        self.min_angle_z, self.max_angle_z = z_limit.lower, z_limit.upper


class Elbow:
    """Command for elbow flexion."""

    def __init__(self) -> None:
        """Initialize elbow flexion."""
        x_limit = RIGHT_ARM_LIMITS_DEG["elbow_x"]
        self.angle_x = x_limit.home
        self.min_angle_x, self.max_angle_x = x_limit.lower, x_limit.upper


class Wrist:
    """Command for the driven wrist rotation."""

    def __init__(self) -> None:
        rotation_limit = RIGHT_ARM_LIMITS_DEG["wrist_rotation"]
        self.rotation = rotation_limit.home
        self.min_rotation = rotation_limit.lower
        self.max_rotation = rotation_limit.upper


class Hand:
    """Shared curl command for the five finger servos."""

    def __init__(self) -> None:
        """Initialize the hand in an open pose."""
        self.curl = 0.0  # 0.0 is open; 1.0 is a closed fist.
        self.min_curl = 0.0
        self.max_curl = 1.5  # Original LIMB-HT25 hand travel, in radians.


class LimbArm:
    """Operator-requested angles for the complete simulated arm."""

    def __init__(self) -> None:
        """Initialize every joint group in its neutral pose."""
        self.shoulder = Shoulder()
        self.elbow = Elbow()
        self.wrist = Wrist()
        self.hand = Hand()


def set_shoulder(
    arm: LimbArm,
    x: float | None = None,
    y: float | None = None,
    z: float | None = None,
    mode: str = "abs",
) -> None:
    """Set or adjust shoulder angles while applying software limits."""
    if x is not None:
        target = (arm.shoulder.angle_x + x) if mode == "rel" else x
        arm.shoulder.angle_x = clamp(target, arm.shoulder.min_angle_x, arm.shoulder.max_angle_x)
    if y is not None:
        target = (arm.shoulder.angle_y + y) if mode == "rel" else y
        arm.shoulder.angle_y = clamp(target, arm.shoulder.min_angle_y, arm.shoulder.max_angle_y)
    if z is not None:
        target = (arm.shoulder.angle_z + z) if mode == "rel" else z
        arm.shoulder.angle_z = clamp(target, arm.shoulder.min_angle_z, arm.shoulder.max_angle_z)


def set_elbow(
    arm: LimbArm,
    x: float | None = None,
    mode: str = "abs",
) -> None:
    """Set or adjust elbow flexion while applying software limits."""
    if x is not None:
        target = (arm.elbow.angle_x + x) if mode == "rel" else x
        arm.elbow.angle_x = clamp(target, arm.elbow.min_angle_x, arm.elbow.max_angle_x)


def set_wrist(arm: LimbArm, rotation: float, mode: str = "abs") -> None:
    """Set or adjust wrist rotation while applying its physical limits."""
    target = (arm.wrist.rotation + rotation) if mode == "rel" else rotation
    arm.wrist.rotation = clamp(
        target,
        arm.wrist.min_rotation,
        arm.wrist.max_rotation,
    )


def set_ready_pose(arm: LimbArm) -> None:
    """Move the command model to the simulator's clear starting pose."""
    set_shoulder(
        arm,
        x=READY_POSE_DEG["shoulder_x"],
        y=READY_POSE_DEG["shoulder_y"],
        z=READY_POSE_DEG["shoulder_z"],
    )
    set_elbow(arm, x=READY_POSE_DEG["elbow_x"])
    set_wrist(arm, READY_POSE_DEG["wrist_rotation"])
    arm.hand.curl = 0.0


def get_angles_deg(arm: LimbArm) -> dict[str, float]:
    """Return the five requested arm-joint angles in degrees."""
    return {
        "shoulder_x": arm.shoulder.angle_x,
        "shoulder_y": arm.shoulder.angle_y,
        "shoulder_z": arm.shoulder.angle_z,
        "elbow_x": arm.elbow.angle_x,
        "wrist_rotation": arm.wrist.rotation,
    }


def get_angles_rad(arm: LimbArm) -> dict[str, float]:
    """Return requested arm and finger joint angles in radians."""
    degs = get_angles_deg(arm)
    angles_rad = {k: deg_to_rad(v) for k, v in degs.items()}
    angles_rad.update(finger_joint_angles_rad(arm.hand.curl))
    return angles_rad


# PyBullet mapping


def sync_to_pybullet(
    arm: LimbArm,
    body_id: int,
    joint_name_to_index: dict[str, int],
    client=None,
    use_motors: bool = True,
) -> None:
    """Apply the requested arm pose to mapped PyBullet joints."""
    cli = client if client is not None else p
    if not cli.getConnectionInfo().get("isConnected", 0):
        return

    requested_angles = get_angles_rad(arm)

    for name, angle in requested_angles.items():
        if name not in joint_name_to_index:
            continue
        joint_index = joint_name_to_index[name]
        is_finger = any(token in name for token in FINGER_JOINT_TOKENS)
        force = FINGER_MOTOR_FORCE_NM if is_finger else ARM_MOTOR_FORCE_NM[name]
        p_gain = FINGER_POSITION_GAIN if is_finger else ARM_POSITION_GAIN
        v_gain = FINGER_VELOCITY_GAIN if is_finger else ARM_VELOCITY_GAIN
        max_velocity = FINGER_PREVIEW_SPEED_RAD_S if is_finger else max_velocity_rad_s(name)

        if use_motors:
            urdf_joint = cli.getJointInfo(body_id, joint_index)
            if urdf_joint[10] > 0:
                force = min(force, urdf_joint[10])
            if urdf_joint[11] > 0:
                max_velocity = min(max_velocity, urdf_joint[11])
            cli.setJointMotorControl2(
                bodyIndex=body_id,
                jointIndex=joint_index,
                controlMode=cli.POSITION_CONTROL,
                targetPosition=angle,
                positionGain=p_gain,
                velocityGain=v_gain,
                force=force,
                maxVelocity=max_velocity,
            )
        else:
            cli.resetJointState(
                bodyUniqueId=body_id,
                jointIndex=joint_index,
                targetValue=angle,
                targetVelocity=0.0,
            )

URDF_TO_LOGICAL_JOINT = {
    "jRightShoulder_rotx": "shoulder_x",
    "jRightShoulder_roty": "shoulder_y",
    "jRightShoulder_rotz": "shoulder_z",
    "jRightElbow_roty": "elbow_x",
    "jRightWrist_rotation": "wrist_rotation",
    "thumb_joint_1": "thumb_1",
    "thumb_joint_2": "thumb_2",
    "thumb_joint_3": "thumb_3",
    "index_joint_1": "index_1",
    "index_joint_2": "index_2",
    "middle_joint_1": "middle_1",
    "middle_joint_2": "middle_2",
    "ring_joint_1": "ring_1",
    "ring_joint_2": "ring_2",
    "ring_joint_3": "ring_3",
    "pinky_joint_1": "pinky_1",
    "pinky_joint_2": "pinky_2",
    "pinky_joint_3": "pinky_3",
}
ARM_JOINT_NAMES = frozenset(
    {"shoulder_x", "shoulder_y", "shoulder_z", "elbow_x", "wrist_rotation"}
)
LOGICAL_TO_URDF_JOINT = {
    logical: urdf
    for urdf, logical in URDF_TO_LOGICAL_JOINT.items()
    if logical in ARM_JOINT_NAMES
}


def build_joint_index_map(robot_id: int, client) -> dict[str, int]:
    """Map logical controller names to their indices in the right-arm URDF."""
    joint_map: dict[str, int] = {}
    for joint_index in range(client.getNumJoints(robot_id)):
        joint_info = client.getJointInfo(robot_id, joint_index)
        urdf_name = joint_info[1].decode("utf-8")
        logical_name = URDF_TO_LOGICAL_JOINT.get(urdf_name)
        if logical_name and joint_info[2] == client.JOINT_REVOLUTE:
            joint_map[logical_name] = joint_index

    print("--- Joint map (controller -> PyBullet) ---")
    logical_to_urdf = {logical: urdf for urdf, logical in URDF_TO_LOGICAL_JOINT.items()}
    for logical_name, joint_index in joint_map.items():
        print(f"  {logical_name} -> joint {joint_index} ({logical_to_urdf[logical_name]})")

    arm_joint_count = len(ARM_JOINT_NAMES.intersection(joint_map))
    if arm_joint_count != len(ARM_JOINT_NAMES):
        print(
            f"Warning: {arm_joint_count}/{len(ARM_JOINT_NAMES)} arm joints found. "
            "Check the URDF names."
        )
    return joint_map


def build_link_name_index_map(robot_id: int, client) -> dict[str, int]:
    """Map every URDF link name to its PyBullet link index."""
    base_name = client.getBodyInfo(robot_id)[0].decode("utf-8")
    link_map = {base_name: -1}
    for joint_index in range(client.getNumJoints(robot_id)):
        joint_info = client.getJointInfo(robot_id, joint_index)
        child_link_name = joint_info[12].decode("utf-8")
        link_map[child_link_name] = joint_index
    return link_map


def resolve_link_index(
    link_indices_by_name: dict[str, int],
    preferred_names: tuple[str, ...],
    name_fragment: str,
) -> tuple[str, int]:
    """Resolve a display link by preferred name, then by a name fragment."""
    for name in preferred_names:
        if name in link_indices_by_name:
            return name, link_indices_by_name[name]
    for name, link_index in link_indices_by_name.items():
        if name_fragment in name.lower():
            return name, link_index
    raise KeyError(f"No link found for '{name_fragment}'")


# PyBullet scene
simulator_dir = os.path.abspath(os.path.dirname(__file__))
model_dir = os.path.abspath(os.path.join(simulator_dir, "..", "sim"))
urdf_path = os.path.join(model_dir, "arm", "right_arm.urdf")

print(f"Simulator mode: {args.mode}")
print(f"Simulator directory: {simulator_dir}")
print(f"URDF path: {urdf_path}")

try:
    physics_client = p.connect(p.GUI)
except p.error as error:
    raise SystemExit(f"Could not start PyBullet: {error}") from error

if physics_client < 0:
    raise SystemExit("Could not connect to the PyBullet GUI.")

preview_visible = False
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
for preview in (
    p.COV_ENABLE_RGB_BUFFER_PREVIEW,
    p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
    p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
):
    p.configureDebugVisualizer(preview, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.setPhysicsEngineParameter(
    fixedTimeStep=1.0 / SIMULATION_FREQUENCY_HZ,
    numSolverIterations=100,
    numSubSteps=PHYSICS_SUBSTEPS,
)
p.loadURDF("plane.urdf")
p.loadURDF("table/table.urdf", [0, 0.8, -0.2], useFixedBase=True)

target_start_position = [0.2, 0.6, 0.5]
target_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
cup_collision_shape = p.createCollisionShape(
    shapeType=p.GEOM_CYLINDER,
    radius=TARGET_RADIUS_M,
    height=TARGET_HEIGHT_M,
)
cup_visual_shape = p.createVisualShape(
    shapeType=p.GEOM_CYLINDER,
    radius=TARGET_RADIUS_M,
    length=TARGET_HEIGHT_M,
    rgbaColor=[0.9, 0.12, 0.08, 1.0],
)
target_body = p.createMultiBody(
    baseMass=TARGET_NORMAL_MASS_KG,
    baseCollisionShapeIndex=cup_collision_shape,
    baseVisualShapeIndex=cup_visual_shape,
    basePosition=target_start_position,
    baseOrientation=target_start_orientation,
)
print(f"Cylindrical cup (ID: {target_body}) placed at {target_start_position}")


def anchor_target_to_world() -> int:
    """Hold the target at its start pose until grasping begins."""
    constraint = p.createConstraint(
        parentBodyUniqueId=target_body,
        parentLinkIndex=-1,
        childBodyUniqueId=-1,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        parentFrameOrientation=[0, 0, 0, 1],
        childFramePosition=target_start_position,
        childFrameOrientation=target_start_orientation,
    )
    p.changeConstraint(constraint, maxForce=10_000, erp=1.0)
    return constraint


target_anchor_constraint = anchor_target_to_world()
p.resetDebugVisualizerCamera(
    cameraDistance=1.25,
    cameraYaw=45,
    cameraPitch=-24,
    cameraTargetPosition=[0, 0.35, 0.45]
)



try:
    print("Attempting to load robot...")
    base_orientation = p.getQuaternionFromEuler([0, 0, 0])

    # PyBullet on Windows cannot open a non-ASCII absolute path reliably. Load
    # the repository-relative path while this directory is the working folder.
    previous_directory = os.getcwd()
    os.chdir(model_dir)
    try:
        p.setAdditionalSearchPath(".")
        robot_body = p.loadURDF(
            "arm/right_arm.urdf",
            [0, 0, 0.7],
            base_orientation,
            useFixedBase=True,
        )
    finally:
        os.chdir(previous_directory)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

    print("\nRobot loaded successfully!")

    p.changeDynamics(
        target_body,
        -1,
        mass=TARGET_NORMAL_MASS_KG,
        lateralFriction=2.5,
        spinningFriction=0.1,
        rollingFriction=0.1,
    )
    for joint_index in range(p.getNumJoints(robot_body)):
        p.changeDynamics(
            robot_body,
            joint_index,
            lateralFriction=2.5,
            spinningFriction=0.1,
            rollingFriction=0.001,
        )
        if p.getJointInfo(robot_body, joint_index)[2] != p.JOINT_FIXED:
            p.setJointMotorControl2(
                robot_body, joint_index, p.VELOCITY_CONTROL, force=0.0
            )

except p.error as error:
    print("\n--- ERROR LOADING ROBOT ---")
    print("Check that right_arm.urdf and its STL files are in src/simulation/sim/arm.")
    print(f"PyBullet error: {error}")
    p.disconnect()
    raise SystemExit(1) from error

# Controls and dashboard
print("Initializing 'brain'...")

arm = LimbArm()
set_ready_pose(arm)
manual_motion = ManualMotion()
joint_indices = build_joint_index_map(robot_body, p)
sync_to_pybullet(arm, robot_body, joint_indices, client=p, use_motors=False)
dynamics = ArmDynamics(robot_body, p)
link_indices_by_name = build_link_name_index_map(robot_body, p)

# Keep the project name attached to the orange upper-arm shell. Debug text is
# used instead of modifying the STL so the source mesh remains reusable.
upper_arm_link_index = link_indices_by_name.get("right_upper_arm", -1)
if upper_arm_link_index >= 0:
    p.addUserDebugText(
        "AURORA",
        textPosition=[0.13, -0.045, 0.035],
        textColorRGB=[0.03, 0.03, 0.03],
        textSize=1.25,
        lifeTime=0,
        parentObjectUniqueId=robot_body,
        parentLinkIndex=upper_arm_link_index,
    )

try:
    hand_link_name, hand_link_index = resolve_link_index(
        link_indices_by_name,
        ("right_hand", "RightHand"),
        "hand",
    )
    print(f"Link 'Hand' found: {hand_link_name} (Index: {hand_link_index})")

    wrist_link_name, wrist_link_index = resolve_link_index(
        link_indices_by_name,
        ("right_wrist_rotation",),
        "wrist",
    )
    print(f"Link 'Wrist' found: {wrist_link_name} (Index: {wrist_link_index})")

    elbow_link_name, elbow_link_index = resolve_link_index(
        link_indices_by_name,
        ("right_forearm", "RightForeArm"),
        "forearm",
    )
    print(f"Link 'Elbow' found: {elbow_link_name} (Index: {elbow_link_index})")
except KeyError as error:
    print(f"ERROR: {error}. Link positions will be unavailable.")
    print("Links found:", list(link_indices_by_name.keys()))
    hand_link_index = wrist_link_index = elbow_link_index = -1

finger_contact_link_indices = {
    index
    for name, index in link_indices_by_name.items()
    if any(finger in name.lower() for finger in FINGER_JOINT_TOKENS)
}
fingertip_link_indices = {
    finger: link_indices_by_name.get(f"{finger}_link_3", -1)
    for finger in FINGER_JOINT_TOKENS
}
hand_contact_link_indices = set(finger_contact_link_indices)
if hand_link_index >= 0:
    hand_contact_link_indices.add(hand_link_index)

print("Initializing sensor window (Tkinter)...")
sensor_window = tk.Tk()
sensor_window.title("LIMB Sensor Dashboard")
sensor_window.geometry("500x805+35+35")
sensor_window.protocol("WM_DELETE_WINDOW", sensor_window.withdraw)

sensor_vars = {}
tk_font = ("Consolas", 11)
tk_font_bold = ("Consolas", 12, "bold")

joint_frame = tk.Frame(sensor_window, padx=10, pady=10)
joint_frame.pack(fill='x')

tk.Label(joint_frame, text="--- JOINT POSE (Simulated) ---", font=tk_font_bold).pack(anchor='w')

joint_names_map = {
    "Upper rot": "shoulder_x",
    "Shoulder UD": "shoulder_y",
    "Shoulder LR": "shoulder_z",
    "Elbow": "elbow_x",
    "Wrist rot": "wrist_rotation",
}

row = tk.Frame(joint_frame)
tk.Label(row, text="", width=13, font=("Consolas", 11, "bold")).pack(side=tk.LEFT)
tk.Label(
    row,
    text="Angle",
    width=12,
    anchor="w",
    font=("Consolas", 11, "underline"),
).pack(side=tk.LEFT)
tk.Label(
    row,
    text="Torque",
    width=12,
    anchor="w",
    font=("Consolas", 11, "underline"),
).pack(side=tk.LEFT)
row.pack(anchor='w')

for name, key in joint_names_map.items():
    sensor_vars[f"{key}_angle"] = tk.StringVar(value="--.- deg")
    sensor_vars[f"{key}_torque"] = tk.StringVar(value="N/A")

    row = tk.Frame(joint_frame)
    tk.Label(row, text=f"{name}:", width=13, font=tk_font_bold).pack(side=tk.LEFT)
    tk.Label(
        row,
        textvariable=sensor_vars[f"{key}_angle"],
        width=12,
        anchor="w",
        font=tk_font,
    ).pack(side=tk.LEFT)
    tk.Label(
        row,
        textvariable=sensor_vars[f"{key}_torque"],
        width=10,
        anchor="w",
        font=tk_font,
    ).pack(side=tk.LEFT)
    row.pack(anchor='w')

torque_graph = TorqueGraph(
    sensor_window,
    tk,
    {key: name for name, key in joint_names_map.items()},
    torque_limit_nm=max(ARM_MOTOR_FORCE_NM.values()),
    sample_rate_hz=SIMULATION_FREQUENCY_HZ,
)
torque_graph.redraw(DYNAMIC_MODE)

imu_frame = tk.Frame(sensor_window, padx=10, pady=10)
imu_frame.pack(fill='x')

tk.Label(imu_frame, text="--- IMU SENSOR (Hand) ---", font=tk_font_bold).pack(anchor='w')
sensor_vars["imu_roll"] = tk.StringVar(value="Roll:  --.-")
sensor_vars["imu_pitch"] = tk.StringVar(value="Pitch: --.-")
sensor_vars["imu_yaw"] = tk.StringVar(value="Yaw:   --.-")

tk.Label(imu_frame, textvariable=sensor_vars["imu_roll"], font=tk_font).pack(anchor='w')
tk.Label(imu_frame, textvariable=sensor_vars["imu_pitch"], font=tk_font).pack(anchor='w')
tk.Label(imu_frame, textvariable=sensor_vars["imu_yaw"], font=tk_font).pack(anchor='w')

bio_frame = tk.Frame(sensor_window, padx=10, pady=10)
bio_frame.pack(fill='x')

tk.Label(bio_frame, text="--- BIO-SENSORS (Simulated) ---", font=tk_font_bold).pack(anchor='w')
sensor_vars["emg_shoulder"] = tk.StringVar(value="EMG: open BLE live preview")
sensor_vars["pressure_hand"] = tk.StringVar(value="Fingertip contact: 0/5")

tk.Label(bio_frame, textvariable=sensor_vars["emg_shoulder"], font=tk_font).pack(anchor='w')
tk.Label(bio_frame, textvariable=sensor_vars["pressure_hand"], font=tk_font).pack(anchor='w')
for finger in FINGER_JOINT_TOKENS:
    sensor_vars[f"pressure_{finger}"] = tk.StringVar(
        value=f"{finger.title():6}: 0.00 N  [----------]"
    )
    tk.Label(
        bio_frame,
        textvariable=sensor_vars[f"pressure_{finger}"],
        font=tk_font,
    ).pack(anchor='w')

physics_frame = tk.Frame(sensor_window, padx=10, pady=6)
physics_frame.pack(fill='x')
tk.Label(physics_frame, text="--- PHYSICS (URDF ESTIMATE) ---", font=tk_font_bold).pack(anchor='w')
sensor_vars["gravity_effort"] = tk.StringVar(value="Gravity hold: --")
sensor_vars["grip_speed"] = tk.StringVar(value="Grip speed: --")
tk.Label(physics_frame, textvariable=sensor_vars["gravity_effort"], font=tk_font).pack(anchor='w')
tk.Label(physics_frame, textvariable=sensor_vars["grip_speed"], font=tk_font).pack(anchor='w')


print("Initializing Pygame interface...")
os.environ.setdefault("SDL_VIDEO_WINDOW_POS", "35,500")
pygame.init()
screen = pygame.display.set_mode((900, 440))
pygame.display.set_caption("LIMB Arm Controller - click here to drive")
font_title = pygame.font.SysFont("Segoe UI", 27, bold=True)
font_heading = pygame.font.SysFont("Segoe UI", 18, bold=True)
font = pygame.font.SysFont("Segoe UI", 16)
font_small = pygame.font.SysFont("Segoe UI", 14)
clock = pygame.time.Clock()
print("\n--- Keyboard Controls ---")
print("Shoulder: W/S and A/D | Upper arm: Q/E")
print("Elbow: Up/Down | Wrist: Left/Right | Fingers: F/G")
print("Space interact | H auto reach | R reset | C camera | Esc quit")


def draw_text(text: str, position, color, text_font=font) -> None:
    screen.blit(text_font.render(text, True, color), position)


def draw_panel(rect, heading: str) -> None:
    pygame.draw.rect(screen, (29, 36, 49), rect, border_radius=10)
    pygame.draw.rect(screen, (55, 68, 88), rect, width=1, border_radius=10)
    draw_text(heading, (rect.x + 18, rect.y + 13), (242, 246, 252), font_heading)


def draw_control_row(y: int, keys_text: str, label: str, value: str, x: int = 32) -> None:
    key_rect = pygame.Rect(x, y, 92, 27)
    pygame.draw.rect(screen, (50, 104, 216), key_rect, border_radius=6)
    key_surface = font_small.render(keys_text, True, (255, 255, 255))
    screen.blit(key_surface, key_surface.get_rect(center=key_rect.center))
    draw_text(label, (x + 106, y + 3), (207, 216, 230), font_small)
    draw_text(value, (x + 285, y + 3), (121, 214, 168), font_small)

# Main loop
running = True
hud_visible = True
grasp_transform = None
grasp_constraint = None
auto_grasp_pending = False
auto_grasp_phase = "reach"
h_contact_stop = False
latched_fingertip_force_n = {finger: 0.0 for finger in FINGER_JOINT_TOKENS}
reach_line_id = -1
target_text_id = -1

camera_mode = CAMERA_MODES[0]
default_cam_yaw = 45
default_cam_pitch = -24
default_cam_target = [0, 0.35, 0.45]
notice_text = "Ready - click this window to control the arm"
notice_until_ms = 0

default_position = (0.0, 0.0, 0.0)
default_orientation = (0.0, 0.0, 0.0)
hand_position = default_position
wrist_position = default_position
elbow_position = default_position
imu_euler_rad = default_orientation
measured_angles_deg = {name: 0.0 for name in joint_indices}
measured_motor_torque = {name: 0.0 for name in joint_indices}
physics_snapshot = None
frame_number = 0
telemetry_file = None
if args.telemetry_out is not None:
    args.telemetry_out.parent.mkdir(parents=True, exist_ok=True)
    telemetry_file = args.telemetry_out.open("w", encoding="utf-8")
    print(f"Physics telemetry: {args.telemetry_out}")

torque_csv_file = None
torque_csv_writer = None
if args.torque_out is not None:
    args.torque_out.parent.mkdir(parents=True, exist_ok=True)
    torque_csv_file = args.torque_out.open("w", newline="", encoding="utf-8")
    torque_csv_writer = csv.DictWriter(
        torque_csv_file,
        fieldnames=(
            "time_s",
            "logical_joint",
            "movement",
            "urdf_joint",
            "motor_model",
            "mapping_source",
            "published_motor_rating",
            "angle_deg",
            "velocity_deg_s",
            "applied_motor_torque_Nm",
            "gravity_torque_Nm",
            "inverse_dynamics_torque_Nm",
            "simulation_effort_limit_Nm",
            "payload_kg",
        ),
    )
    torque_csv_writer.writeheader()
    print(f"Torque CSV: {args.torque_out}")


def set_target_robot_collision(enabled: bool) -> None:
    """A constrained payload should not collide with the hand carrying it."""
    for link_index in range(-1, p.getNumJoints(robot_body)):
        p.setCollisionFilterPair(
            robot_body, target_body, link_index, -1, int(enabled)
        )


def target_contact_feedback() -> tuple[set[int], dict[str, float]]:
    """Return cup contacts and virtual force values at the five fingertips."""
    force_by_finger = {finger: 0.0 for finger in FINGER_JOINT_TOKENS}
    try:
        points = p.getContactPoints(bodyA=robot_body, bodyB=target_body)
    except p.error:
        return set(), force_by_finger

    contact_links = {point[3] for point in points if point[3] >= 0}
    finger_by_link = {
        link_index: finger
        for finger, link_index in fingertip_link_indices.items()
        if link_index >= 0
    }
    for point in points:
        finger = finger_by_link.get(point[3])
        if finger is None:
            continue
        contact_force = estimate_contact_force_n(
            normal_force_n=float(point[9]),
            contact_distance_m=float(point[8]),
            stiffness_n_per_m=CONTACT_STIFFNESS_N_PER_M,
        )
        force_by_finger[finger] = min(
            20.0,
            force_by_finger[finger] + contact_force,
        )
    return contact_links, force_by_finger


while running and p.isConnected():

    try:
        target_position, _ = p.getBasePositionAndOrientation(target_body)
        robot_position, _ = p.getBasePositionAndOrientation(robot_body)
    except p.error:
        running = False
        break

    target_x_relative = target_position[0] - robot_position[0]
    target_y_relative = target_position[1] - robot_position[1]
    target_z_relative = target_position[2] - robot_position[2]
    target_relative_position = (target_x_relative, target_y_relative, target_z_relative)
    hand_target_distance = math.dist(hand_position, target_position)

    reset_requested = False
    interact_requested = False
    camera_changed = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_t:
                hud_visible = not hud_visible
                notice_text = "Target guide shown" if hud_visible else "Target guide hidden"
                notice_until_ms = pygame.time.get_ticks() + 1800

            if event.key == pygame.K_SPACE:
                interact_requested = True

            if event.key == pygame.K_r:
                reset_requested = True

            if event.key in (pygame.K_c, pygame.K_TAB):
                camera_index = (CAMERA_MODES.index(camera_mode) + 1) % len(CAMERA_MODES)
                camera_mode = CAMERA_MODES[camera_index]
                camera_changed = True
                notice_text = f"Camera: {camera_mode}"
                notice_until_ms = pygame.time.get_ticks() + 1800

            if event.key in (pygame.K_1, pygame.K_2, pygame.K_3):
                camera_mode = CAMERA_MODES[event.key - pygame.K_1]
                camera_changed = True
                notice_text = f"Camera: {camera_mode}"
                notice_until_ms = pygame.time.get_ticks() + 1800

            if event.key == pygame.K_p:
                preview_visible = not preview_visible
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, int(preview_visible))
                for preview in (
                    p.COV_ENABLE_RGB_BUFFER_PREVIEW,
                    p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                    p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
                ):
                    p.configureDebugVisualizer(preview, int(preview_visible))
                notice_text = (
                    "Camera previews shown"
                    if preview_visible
                    else "Camera previews hidden"
                )
                notice_until_ms = pygame.time.get_ticks() + 1800

            if event.key == pygame.K_i:
                if sensor_window.state() == "withdrawn":
                    sensor_window.deiconify()
                    sensor_window.lift()
                    notice_text = "Sensor dashboard shown"
                else:
                    sensor_window.withdraw()
                    notice_text = "Sensor dashboard hidden"
                notice_until_ms = pygame.time.get_ticks() + 1800

    # The scene can receive focus instead of the separate Pygame controller.
    # Accept the essential task keys from either window.
    bullet_events = p.getKeyboardEvents()

    def bullet_key_down(letter: str) -> bool:
        return any(
            bullet_events.get(ord(key), 0) & p.KEY_IS_DOWN
            for key in (letter.lower(), letter.upper())
        )

    def bullet_key_triggered(letter: str) -> bool:
        return any(
            bullet_events.get(ord(key), 0) & p.KEY_WAS_TRIGGERED
            for key in (letter.lower(), letter.upper())
        )

    if bullet_events.get(ord(" "), 0) & p.KEY_WAS_TRIGGERED:
        interact_requested = True
    if bullet_key_triggered("r"):
        reset_requested = True

    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        running = False

    if reset_requested:
        auto_grasp_pending = False
        auto_grasp_phase = "reach"
        h_contact_stop = False
        grasp_transform = None
        latched_fingertip_force_n = {
            finger: 0.0 for finger in FINGER_JOINT_TOKENS
        }
        if grasp_constraint is not None:
            p.removeConstraint(grasp_constraint)
            grasp_constraint = None
            set_target_robot_collision(True)
        if target_anchor_constraint is not None:
            p.removeConstraint(target_anchor_constraint)
            target_anchor_constraint = None

        p.changeDynamics(target_body, -1, mass=TARGET_NORMAL_MASS_KG)
        p.setCollisionFilterGroupMask(
            target_body,
            -1,
            collisionFilterGroup=1,
            collisionFilterMask=1,
        )
        p.resetBasePositionAndOrientation(
            target_body,
            target_start_position,
            target_start_orientation,
        )
        p.resetBaseVelocity(target_body, [0, 0, 0], [0, 0, 0])
        target_anchor_constraint = anchor_target_to_world()

        set_ready_pose(arm)
        manual_motion.reset()
        if p.isConnected():
            sync_to_pybullet(arm, robot_body, joint_indices, client=p, use_motors=False)
            p.performCollisionDetection()
        notice_text = "Arm and target reset"
        notice_until_ms = pygame.time.get_ticks() + 2200

    current_contact_links, fingertip_force_n = target_contact_feedback()
    hand_target_contact = bool(current_contact_links & hand_contact_link_indices)
    active_fingertips = {
        finger
        for finger, force_n in fingertip_force_n.items()
        if force_n >= FINGERTIP_CONTACT_FORCE_N
    }

    h_reach_requested = keys[pygame.K_h] or bullet_key_down("h")
    if not h_reach_requested:
        h_contact_stop = False
    elif (
        (hand_target_contact or hand_target_distance <= SAFE_REACH_DISTANCE_M)
        and not h_contact_stop
    ):
        h_contact_stop = True
        notice_text = "Safe reach distance reached - H movement stopped"
        notice_until_ms = pygame.time.get_ticks() + 1800

    if auto_grasp_pending and auto_grasp_phase == "open":
        if arm.hand.curl <= 0.01:
            arm.hand.curl = 0.0
            auto_grasp_phase = "reach"
    elif (
        auto_grasp_pending
        and auto_grasp_phase == "reach"
        and (hand_target_contact or hand_target_distance <= SAFE_REACH_DISTANCE_M)
    ):
        auto_grasp_phase = "close"
        notice_text = "Cup reached - closing fingers"
        notice_until_ms = pygame.time.get_ticks() + 1800

    auto_reach_active = auto_grasp_pending and auto_grasp_phase == "reach"
    if (h_reach_requested and not h_contact_stop) or auto_reach_active:
        try:
            relative_x, relative_y, relative_z = target_relative_position
            horizontal_distance = math.hypot(relative_x, relative_y)
            target_shoulder_y, target_elbow_x = calculate_ik_angles(
                horizontal_distance,
                relative_z,
            )
            world_azimuth = math.atan2(relative_y, relative_x)
            target_shoulder_z = math.degrees(world_azimuth - math.pi / 2.0)
            while target_shoulder_z > 180:
                target_shoulder_z -= 360
            while target_shoulder_z <= -180:
                target_shoulder_z += 360

            set_shoulder(
                arm,
                x=step_toward(arm.shoulder.angle_x, 0.0, "shoulder_x"),
                y=step_toward(
                    arm.shoulder.angle_y,
                    target_shoulder_y,
                    "shoulder_y",
                ),
                z=step_toward(
                    arm.shoulder.angle_z,
                    target_shoulder_z,
                    "shoulder_z",
                ),
                mode="abs",
            )
            set_elbow(
                arm,
                x=step_toward(arm.elbow.angle_x, target_elbow_x, "elbow_x"),
                mode="abs",
            )
            if auto_reach_active:
                set_wrist(
                    arm,
                    step_toward(
                        arm.wrist.rotation,
                        AUTO_GRASP_WRIST_DEG,
                        "wrist_rotation",
                    ),
                    mode="abs",
                )
        except ValueError as error:
            notice_text = f"Target cannot be reached: {error}"
            notice_until_ms = pygame.time.get_ticks() + 1800

    if auto_grasp_pending and auto_grasp_phase in {"open", "close"}:
        auto_hand_step = AUTO_GRASP_CURL_PER_SECOND / SIMULATION_FREQUENCY_HZ
        if auto_grasp_phase == "open":
            arm.hand.curl = max(0.0, arm.hand.curl - auto_hand_step)
        else:
            arm.hand.curl = min(MAX_GRASP_CURL, arm.hand.curl + auto_hand_step)
    else:
        hand_step = FINGER_CURL_PER_SECOND / SIMULATION_FREQUENCY_HZ
        delta_hand = (hand_step if keys[pygame.K_f] or bullet_key_down("f") else 0) + (
            -hand_step if keys[pygame.K_g] or bullet_key_down("g") else 0
        )
        if delta_hand:
            new_value = arm.hand.curl + delta_hand
            arm.hand.curl = clamp(new_value, arm.hand.min_curl, arm.hand.max_curl)

    # Match LIMB-HT25 manual handling: holding F inside 20 cm closes the hand
    # and immediately secures the object. Space keeps the assisted workflow.
    manual_grasp_requested = (
        (keys[pygame.K_f] or bullet_key_down("f"))
        and hand_target_distance <= GRASP_CAPTURE_DISTANCE_M
    )

    if (
        interact_requested or auto_grasp_pending or manual_grasp_requested
    ) and grasp_transform is None and hand_link_index != -1:
        if interact_requested and auto_grasp_pending:
            auto_grasp_pending = False
            auto_grasp_phase = "reach"
            notice_text = "Assisted grasp canceled"
            notice_until_ms = pygame.time.get_ticks() + 1800
            print(notice_text)
        elif manual_grasp_requested or (
            auto_grasp_pending
            and auto_grasp_phase == "close"
            and grasp_is_ready(
                arm.hand.curl,
                hand_target_distance,
                len(active_fingertips),
                min_curl=MIN_GRASP_CURL,
                capture_distance_m=GRASP_CAPTURE_DISTANCE_M,
                min_fingertip_contacts=MIN_GRASP_FINGERTIPS,
            )
        ):
            auto_grasp_pending = False
            auto_grasp_phase = "reach"
            latched_fingertip_force_n = dict(fingertip_force_n)
            target_position_world, target_orientation_world = (
                p.getBasePositionAndOrientation(target_body)
            )
            if target_anchor_constraint is not None:
                p.removeConstraint(target_anchor_constraint)
                target_anchor_constraint = None
            hand_state = p.getLinkState(robot_body, hand_link_index)
            hand_pos_world = hand_state[4]
            hand_orn_world = hand_state[5]
            inverse_hand_position, inverse_hand_orientation = p.invertTransform(
                hand_pos_world,
                hand_orn_world,
            )
            target_position_in_hand, target_orientation_in_hand = p.multiplyTransforms(
                inverse_hand_position,
                inverse_hand_orientation,
                target_position_world,
                target_orientation_world,
            )

            if DYNAMIC_MODE:
                set_target_robot_collision(False)
                inverse_com_position, inverse_com_orientation = p.invertTransform(
                    hand_state[0], hand_state[1]
                )
                target_position_in_com, target_orientation_in_com = p.multiplyTransforms(
                    inverse_com_position,
                    inverse_com_orientation,
                    target_position_world,
                    target_orientation_world,
                )
                grasp_constraint = p.createConstraint(
                    robot_body, hand_link_index, target_body, -1,
                    p.JOINT_FIXED, (0, 0, 0),
                    target_position_in_com, (0, 0, 0),
                    target_orientation_in_com, (0, 0, 0, 1),
                )
                contact_force_n = sum(latched_fingertip_force_n.values())
                p.changeConstraint(
                    grasp_constraint,
                    maxForce=max(10.0, min(40.0, 10.0 * contact_force_n)),
                    erp=0.5,
                )
            else:
                p.changeDynamics(target_body, -1, mass=0.0)
                p.setCollisionFilterGroupMask(
                    target_body,
                    -1,
                    collisionFilterGroup=0,
                    collisionFilterMask=0,
                )
            grasp_transform = (target_position_in_hand, target_orientation_in_hand)
            notice_text = (
                "Manual F grip secured"
                if manual_grasp_requested
                else f"Grip secured ({len(active_fingertips)} fingertip contacts)"
            )
            notice_until_ms = pygame.time.get_ticks() + 1800
            print(notice_text)
        elif (
            auto_grasp_pending
            and auto_grasp_phase == "close"
            and arm.hand.curl >= MAX_GRASP_CURL
        ):
            auto_grasp_pending = False
            auto_grasp_phase = "reach"
            notice_text = (
                f"Grip failed: {len(active_fingertips)}/{MIN_GRASP_FINGERTIPS} "
                "fingertip contacts"
            )
            notice_until_ms = pygame.time.get_ticks() + 2600
            print(notice_text)
        elif interact_requested:
            target_reachable, target_reason = is_reachable(target_relative_position)
            if target_reachable:
                auto_grasp_pending = True
                if arm.hand.curl > 0.01:
                    auto_grasp_phase = "open"
                else:
                    auto_grasp_phase = "reach"
                notice_text = f"Safe reach started ({hand_target_distance:.2f} m away)"
            else:
                notice_text = target_reason.title()
            notice_until_ms = pygame.time.get_ticks() + 2200
            print(notice_text)

    elif (interact_requested or keys[pygame.K_g] or bullet_key_down("g")) and grasp_transform is not None:
        grasp_transform = None
        latched_fingertip_force_n = {
            finger: 0.0 for finger in FINGER_JOINT_TOKENS
        }
        if grasp_constraint is not None:
            p.removeConstraint(grasp_constraint)
            grasp_constraint = None
            set_target_robot_collision(True)

        p.changeDynamics(target_body, -1, mass=TARGET_NORMAL_MASS_KG)
        p.setCollisionFilterGroupMask(
            target_body,
            -1,
            collisionFilterGroup=1,
            collisionFilterMask=1,
        )
        p.resetBaseVelocity(target_body, linearVelocity=[0, 0, -0.2])
        notice_text = "Target released"
        notice_until_ms = pygame.time.get_ticks() + 1800
        print(notice_text)

    speed_mult = 1.0
    if keys[pygame.K_LCTRL] or keys[pygame.K_RCTRL]:
        speed_mult = 0.25

    if h_reach_requested or auto_grasp_pending:
        manual_motion.reset()
    else:
        delta_should_y = manual_motion.step(
            keys,
            pygame.K_s,
            pygame.K_w,
            "shoulder_y",
            arm.shoulder.angle_y,
            speed_mult,
        )
        delta_should_z = manual_motion.step(
            keys,
            pygame.K_d,
            pygame.K_a,
            "shoulder_z",
            arm.shoulder.angle_z,
            speed_mult,
        )
        delta_should_x = manual_motion.step(
            keys,
            pygame.K_e,
            pygame.K_q,
            "shoulder_x",
            arm.shoulder.angle_x,
            speed_mult,
        )
        set_shoulder(
            arm,
            x=delta_should_x,
            y=delta_should_y,
            z=delta_should_z,
            mode="rel",
        )

        delta_elbow_x = manual_motion.step(
            keys,
            pygame.K_DOWN,
            pygame.K_UP,
            "elbow_x",
            arm.elbow.angle_x,
            speed_mult,
        )
        delta_wrist = manual_motion.step(
            keys,
            pygame.K_RIGHT,
            pygame.K_LEFT,
            "wrist_rotation",
            arm.wrist.rotation,
            speed_mult,
        )
        set_elbow(arm, x=delta_elbow_x, mode="rel")
        set_wrist(arm, delta_wrist, mode="rel")

    if not p.isConnected():
        running = False
        continue

    if DYNAMIC_MODE:
        sync_to_pybullet(
            arm=arm, body_id=robot_body, joint_name_to_index=joint_indices,
            client=p, use_motors=True,
        )
        p.stepSimulation()
    else:
        # Direct preview keeps the light finger links visually stable.
        p.stepSimulation()
        sync_to_pybullet(
            arm=arm, body_id=robot_body, joint_name_to_index=joint_indices,
            client=p, use_motors=False,
        )
        p.performCollisionDetection()

    measured_angles_deg = {}

    try:
        for name, index in joint_indices.items():
            state = p.getJointState(robot_body, index)
            measured_angles_deg[name] = rad_to_deg(state[0])
            measured_motor_torque[name] = state[3] if DYNAMIC_MODE else 0.0
    except p.error:
        for name in joint_indices:
            measured_angles_deg[name] = 0.0
            measured_motor_torque[name] = 0.0

    frame_number += 1
    if DYNAMIC_MODE:
        torque_graph.append(measured_motor_torque)
    if frame_number % 6 == 0:
        torque_graph.redraw(DYNAMIC_MODE)
    if DYNAMIC_MODE and frame_number % 12 == 0:
        commanded_q, _, _ = dynamics.joint_state()
        column_for_joint = {joint: column for column, joint in enumerate(dynamics.joints)}
        for name, angle in get_angles_rad(arm).items():
            joint = joint_indices.get(name)
            if joint in column_for_joint:
                commanded_q[column_for_joint[joint]] = angle
        physics_snapshot = dynamics.snapshot(
            commanded_positions=commanded_q,
            payload_kg=TARGET_NORMAL_MASS_KG if grasp_constraint is not None else 0.0,
            time_s=frame_number / SIMULATION_FREQUENCY_HZ,
        )
        if telemetry_file is not None:
            telemetry_file.write(json.dumps(physics_snapshot) + "\n")
            telemetry_file.flush()
        if torque_csv_writer is not None:
            snapshot_columns = {
                name: column
                for column, name in enumerate(physics_snapshot["joint_names"])
            }
            for logical_name in joint_names_map.values():
                urdf_name = LOGICAL_TO_URDF_JOINT[logical_name]
                column = snapshot_columns[urdf_name]
                actuator = ARM_ACTUATOR_INFO[logical_name]
                torque_csv_writer.writerow(
                    {
                        "time_s": f"{physics_snapshot['time_s']:.6f}",
                        "logical_joint": logical_name,
                        "movement": actuator["movement"],
                        "urdf_joint": urdf_name,
                        "motor_model": actuator["model"],
                        "mapping_source": actuator["mapping"],
                        "published_motor_rating": actuator["published_rating"],
                        "angle_deg": f"{math.degrees(physics_snapshot['q_rad'][column]):.6f}",
                        "velocity_deg_s": (
                            f"{math.degrees(physics_snapshot['qd_rad_s'][column]):.6f}"
                        ),
                        "applied_motor_torque_Nm": (
                            f"{physics_snapshot['motor_torque_Nm'][column]:.6f}"
                        ),
                        "gravity_torque_Nm": (
                            f"{physics_snapshot['gravity_torque_Nm'][column]:.6f}"
                        ),
                        "inverse_dynamics_torque_Nm": (
                            f"{physics_snapshot['inverse_dynamics_torque_Nm'][column]:.6f}"
                        ),
                        "simulation_effort_limit_Nm": f"{ARM_MOTOR_FORCE_NM[logical_name]:.6f}",
                        "payload_kg": f"{physics_snapshot['payload_kg']:.6f}",
                    }
                )
            torque_csv_file.flush()

    try:
        if hand_link_index != -1:
            hand_state = p.getLinkState(robot_body, hand_link_index)
            hand_position, _ = p.multiplyTransforms(
                hand_state[4],
                hand_state[5],
                HAND_GRIP_OFFSET_M,
                (0.0, 0.0, 0.0, 1.0),
            )
            imu_quaternion = hand_state[5]
            imu_euler_rad = p.getEulerFromQuaternion(imu_quaternion)
        else:
            hand_position = default_position
            imu_euler_rad = default_orientation

        wrist_position = (
            p.getLinkState(robot_body, wrist_link_index)[4]
            if wrist_link_index != -1
            else default_position
        )
        elbow_position = (
            p.getLinkState(robot_body, elbow_link_index)[4]
            if elbow_link_index != -1
            else default_position
        )
    except p.error:
        hand_position = wrist_position = elbow_position = default_position
        imu_euler_rad = default_orientation

    if not DYNAMIC_MODE and grasp_transform is not None and hand_link_index != -1:
        hand_state = p.getLinkState(robot_body, hand_link_index)
        target_position, target_orientation = p.multiplyTransforms(
            hand_state[4], hand_state[5], *grasp_transform
        )
        p.resetBasePositionAndOrientation(target_body, target_position, target_orientation)

    if camera_mode == "overview" and camera_changed:
        p.resetDebugVisualizerCamera(
            cameraDistance=1.25,
            cameraYaw=default_cam_yaw,
            cameraPitch=default_cam_pitch,
            cameraTargetPosition=default_cam_target,
        )
    elif camera_mode == "shoulder":
        p.resetDebugVisualizerCamera(
            cameraDistance=0.65,
            cameraYaw=40,
            cameraPitch=-18,
            cameraTargetPosition=[
                robot_position[0],
                robot_position[1] + 0.18,
                robot_position[2] - 0.03,
            ],
        )
    elif camera_mode == "hand":
        p.resetDebugVisualizerCamera(
            cameraDistance=0.48,
            cameraYaw=45,
            cameraPitch=-22,
            cameraTargetPosition=hand_position,
        )

    hardware_angles_deg = {
        name: hardware_value(name, measured_angles_deg.get(name, 0.0))
        for name in ARM_JOINT_NAMES
    }

    dx = target_position[0] - hand_position[0]
    dy = target_position[1] - hand_position[1]
    dz = target_position[2] - hand_position[2]
    hand_target_distance = math.sqrt(dx**2 + dy**2 + dz**2)
    target_is_reachable, reachability_message = is_reachable(target_relative_position)
    if target_is_reachable:
        status_color = (121, 214, 168)
        status_text = f"Target reachable - hand is {hand_target_distance:.2f} m away"
    else:
        status_color = (255, 119, 119)
        status_text = reachability_message.title()

    screen.fill((15, 21, 31))
    draw_text("LIMB Arm Controller", (20, 14), (245, 248, 252), font_title)
    draw_text(
        f"{args.mode.title()} mode | controls use the mapped joint limits and speeds",
        (21, 50),
        (151, 164, 183),
        font_small,
    )
    camera_label = f"CAMERA  {camera_mode.upper()}"
    camera_badge = pygame.Rect(690, 20, 188, 34)
    pygame.draw.rect(screen, (38, 55, 79), camera_badge, border_radius=17)
    camera_surface = font_small.render(camera_label, True, (177, 207, 255))
    screen.blit(camera_surface, camera_surface.get_rect(center=camera_badge.center))

    draw_panel(pygame.Rect(20, 78, 420, 262), "Move the arm")
    draw_control_row(
        122,
        "W / S",
        "Shoulder up / down",
        f"{hardware_angles_deg['shoulder_y']:.1f} deg",
    )
    draw_control_row(
        162,
        "A / D",
        "Shoulder left / right",
        f"{hardware_angles_deg['shoulder_z']:.1f} deg",
    )
    draw_control_row(
        202,
        "Q / E",
        "Upper-arm rotation",
        f"{hardware_angles_deg['shoulder_x']:.1f} deg",
    )
    draw_control_row(
        242,
        "UP / DOWN",
        "Elbow bend / extend",
        f"{hardware_angles_deg['elbow_x']:.1f} deg",
    )
    draw_control_row(
        282,
        "LEFT / RIGHT",
        "Wrist rotation",
        f"{hardware_angles_deg['wrist_rotation']:.1f} deg",
    )

    draw_panel(pygame.Rect(460, 78, 420, 262), "Actions and view")
    action_x = 472
    draw_control_row(
        122,
        "F / G",
        "Close / open fingers",
        f"{arm.hand.curl * 100:.0f}%",
        action_x,
    )
    draw_control_row(
        162,
        "SPACE",
        "Grab / release target",
        "holding" if grasp_transform is not None else auto_grasp_phase if auto_grasp_pending else "ready",
        action_x,
    )
    draw_control_row(
        202,
        "H",
        "Hold for safe approach",
        "stopped" if h_contact_stop else "reach",
        action_x,
    )
    draw_control_row(242, "C or 1-3", "Change camera", camera_mode, action_x)
    draw_control_row(282, "R", "Reset arm and target", "", action_x)

    pygame.draw.rect(screen, (24, 31, 43), pygame.Rect(20, 354, 860, 68), border_radius=10)
    draw_text(status_text, (36, 366), status_color, font)
    current_notice = notice_text if pygame.time.get_ticks() < notice_until_ms else ""
    if not pygame.key.get_focused():
        current_notice = "Click controller for arm keys; Space, F/G, H, R also work in the scene"
    elif not current_notice:
        current_notice = (
            "Ctrl: precise  |  T: target guide  |  P: camera previews  |  "
            "I: sensors  |  Esc: quit"
        )
    draw_text(current_notice, (36, 394), (164, 178, 198), font_small)

    if hud_visible and hand_link_index != -1:
        reach_line_id = p.addUserDebugLine(
            hand_position,
            target_position,
            lineColorRGB=[1, 0, 0],
            lineWidth=2,
            replaceItemUniqueId=reach_line_id,
        )
        target_text_id = p.addUserDebugText(
            f"TARGET: CUP_ALPHA [DIST: {hand_target_distance:.2f}m]",
            [target_position[0], target_position[1], target_position[2] + 0.15],
            textColorRGB=[1, 1, 0],
            textSize=1.2,
            replaceItemUniqueId=target_text_id,
        )
    elif reach_line_id >= 0:
        p.removeUserDebugItem(reach_line_id)
        p.removeUserDebugItem(target_text_id)
        reach_line_id = -1
        target_text_id = -1

    pygame.display.flip()

    try:
        for key in joint_names_map.values():
            sensor_vars[f"{key}_angle"].set(f"{hardware_angles_deg[key]:.1f} deg")
            sensor_vars[f"{key}_torque"].set(
                f"{measured_motor_torque[key]:.2f} Nm" if DYNAMIC_MODE else "N/A"
            )

        if physics_snapshot is not None:
            shoulder_column = dynamics.names.index("jRightShoulder_roty")
            gravity_hold = physics_snapshot["gravity_torque_Nm"][shoulder_column]
            velocity = physics_snapshot["end_effector"]["velocity_m_s"]
            grip_speed = math.sqrt(sum(component**2 for component in velocity))
            sensor_vars["gravity_effort"].set(f"Shoulder gravity hold: {gravity_hold:.2f} Nm")
            sensor_vars["grip_speed"].set(f"Grip speed: {grip_speed:.3f} m/s")

        sensor_vars["imu_roll"].set(f"Roll:  {rad_to_deg(imu_euler_rad[0]):.1f}")
        sensor_vars["imu_pitch"].set(f"Pitch: {rad_to_deg(imu_euler_rad[1]):.1f}")
        sensor_vars["imu_yaw"].set(f"Yaw:   {rad_to_deg(imu_euler_rad[2]):.1f}")

        displayed_fingertip_force_n = {
            finger: max(
                fingertip_force_n[finger],
                latched_fingertip_force_n[finger]
                if grasp_transform is not None
                else 0.0,
            )
            for finger in FINGER_JOINT_TOKENS
        }
        displayed_contacts = sum(
            force_n >= FINGERTIP_CONTACT_FORCE_N
            for force_n in displayed_fingertip_force_n.values()
        )
        total_contact_force_n = sum(displayed_fingertip_force_n.values())
        sensor_vars["pressure_hand"].set(
            f"Virtual fingertip contact: {displayed_contacts}/5  "
            f"total {total_contact_force_n:.2f} N"
        )
        for finger, force_n in displayed_fingertip_force_n.items():
            filled = round(min(force_n / 2.0, 1.0) * 10)
            bar = "#" * filled + "-" * (10 - filled)
            sensor_vars[f"pressure_{finger}"].set(
                f"{finger.title():6}: {force_n:5.2f} N  [{bar}]"
            )

        sensor_window.update()

    except tk.TclError:
        print("Sensor window closed.")
        running = False

    clock.tick(SIMULATION_FREQUENCY_HZ)


print("Simulation finished.")
try:
    sensor_window.destroy()
except tk.TclError:
    pass

pygame.quit()
if telemetry_file is not None:
    telemetry_file.close()
if torque_csv_file is not None:
    torque_csv_file.close()
if p.isConnected():
    p.disconnect()
