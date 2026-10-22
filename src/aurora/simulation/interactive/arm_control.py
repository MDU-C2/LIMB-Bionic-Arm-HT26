# Majd, Christian
"""Arm commands, movement limits, and PyBullet joint mapping."""

from __future__ import annotations

import math

import pybullet as p

from aurora.simulation.interactive.settings import (
    FINGER_JOINT_TOKENS,
    LIVE_ALIGNMENT_POSE_DEG,
    READY_POSE_DEG,
    SIMULATION_FREQUENCY_HZ,
)
from aurora.simulation.model.controller_params import (
    ARM_MOTOR_FORCE_NM,
    ARM_POSITION_GAIN,
    ARM_VELOCITY_GAIN,
    FINGER_MOTOR_FORCE_NM,
    FINGER_POSITION_GAIN,
    FINGER_PREVIEW_SPEED_RAD_S,
    FINGER_VELOCITY_GAIN,
)
from aurora.simulation.model.joint_limits import (
    LEFT_ARM_HARDWARE_SIGN,
    LEFT_ARM_LIMITS_DEG,
    finger_joint_angles_rad,
    max_velocity_rad_s,
)


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
    """Convert a left-arm URDF value to the physical actuator sign."""
    return value * LEFT_ARM_HARDWARE_SIGN[logical_name]


def step_toward(
    current: float,
    target: float,
    joint_name: str,
    scale: float = 1.0,
    elapsed_s: float | None = None,
) -> float:
    """Move toward a target at the actuator speed, independent of render FPS."""
    limit = LEFT_ARM_LIMITS_DEG[joint_name]
    target = limit.clamp(target)
    difference = target - current
    if difference == 0:
        return current
    if elapsed_s is None:
        elapsed_s = 1.0 / SIMULATION_FREQUENCY_HZ
    elapsed_s = min(0.1, max(0.001, float(elapsed_s)))
    max_step = limit.speed(difference) * scale * elapsed_s
    return current + clamp(difference, -max_step, max_step)


class ManualMotion:
    """Turn held keys into smooth, firmware-limited joint movement."""

    def __init__(self) -> None:
        self.velocity_deg_s = {name: 0.0 for name in LEFT_ARM_LIMITS_DEG}

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
        elapsed_s: float | None = None,
    ) -> float:
        direction = int(bool(keys[positive_key])) - int(bool(keys[negative_key]))
        dt = 1.0 / SIMULATION_FREQUENCY_HZ if elapsed_s is None else elapsed_s
        dt = min(0.1, max(0.001, float(dt)))
        limit = LEFT_ARM_LIMITS_DEG[joint_name]
        target_velocity = direction * limit.speed(direction) * speed_scale
        current_velocity = self.velocity_deg_s[joint_name]

        if limit.acceleration is None:
            current_velocity = target_velocity
        else:
            acceleration_step = limit.acceleration * dt
            current_velocity += clamp(
                target_velocity - current_velocity,
                -acceleration_step,
                acceleration_step,
            )

        next_angle = limit.clamp(current_angle + current_velocity * dt)
        if next_angle == current_angle and current_velocity != 0.0:
            current_velocity = 0.0

        self.velocity_deg_s[joint_name] = current_velocity
        return next_angle - current_angle


class Shoulder:
    """Commands for the three shoulder actuators."""

    def __init__(self) -> None:
        """Initialize the shoulder in its neutral pose."""
        x_limit = LEFT_ARM_LIMITS_DEG["shoulder_x"]
        y_limit = LEFT_ARM_LIMITS_DEG["shoulder_y"]
        z_limit = LEFT_ARM_LIMITS_DEG["shoulder_z"]
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
        x_limit = LEFT_ARM_LIMITS_DEG["elbow_x"]
        self.angle_x = x_limit.home
        self.min_angle_x, self.max_angle_x = x_limit.lower, x_limit.upper


class Wrist:
    """Command for the driven wrist rotation."""

    def __init__(self) -> None:
        rotation_limit = LEFT_ARM_LIMITS_DEG["wrist_rotation"]
        self.rotation = rotation_limit.home
        self.min_rotation = rotation_limit.lower
        self.max_rotation = rotation_limit.upper


class Hand:
    """Shared curl command for the five finger servos."""

    def __init__(self) -> None:
        """Initialize the hand in an open pose."""
        self.curl = 0.0  # 0.0 is open; 1.0 is a closed fist.
        self.min_curl = 0.0
        self.max_curl = 1.5  # Hand travel in radians.


class AuroraArm:
    """Operator-requested angles for the complete simulated arm."""

    def __init__(self) -> None:
        """Initialize every joint group in its neutral pose."""
        self.shoulder = Shoulder()
        self.elbow = Elbow()
        self.wrist = Wrist()
        self.hand = Hand()


def set_shoulder(
    arm: AuroraArm,
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
    arm: AuroraArm,
    x: float | None = None,
    mode: str = "abs",
) -> None:
    """Set or adjust elbow flexion while applying software limits."""
    if x is not None:
        target = (arm.elbow.angle_x + x) if mode == "rel" else x
        arm.elbow.angle_x = clamp(target, arm.elbow.min_angle_x, arm.elbow.max_angle_x)


def set_wrist(arm: AuroraArm, rotation: float, mode: str = "abs") -> None:
    """Set or adjust wrist rotation while applying its physical limits."""
    target = (arm.wrist.rotation + rotation) if mode == "rel" else rotation
    arm.wrist.rotation = clamp(
        target,
        arm.wrist.min_rotation,
        arm.wrist.max_rotation,
    )


def set_ready_pose(arm: AuroraArm) -> None:
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


def set_live_alignment_pose(arm: AuroraArm) -> None:
    """Show the neutral tabletop pose the operator copies before feedback starts."""
    set_shoulder(
        arm,
        x=LIVE_ALIGNMENT_POSE_DEG["shoulder_x"],
        y=LIVE_ALIGNMENT_POSE_DEG["shoulder_y"],
        z=LIVE_ALIGNMENT_POSE_DEG["shoulder_z"],
    )
    set_elbow(arm, x=LIVE_ALIGNMENT_POSE_DEG["elbow_x"])
    set_wrist(arm, LIVE_ALIGNMENT_POSE_DEG["wrist_rotation"])
    arm.hand.curl = 0.0


def get_angles_deg(arm: AuroraArm) -> dict[str, float]:
    """Return the five requested arm-joint angles in degrees."""
    return {
        "shoulder_x": arm.shoulder.angle_x,
        "shoulder_y": arm.shoulder.angle_y,
        "shoulder_z": arm.shoulder.angle_z,
        "elbow_x": arm.elbow.angle_x,
        "wrist_rotation": arm.wrist.rotation,
    }


def get_angles_rad(arm: AuroraArm) -> dict[str, float]:
    """Return requested arm and finger joint angles in radians."""
    degs = get_angles_deg(arm)
    angles_rad = {k: deg_to_rad(v) for k, v in degs.items()}
    angles_rad.update(finger_joint_angles_rad(arm.hand.curl))
    return angles_rad


# PyBullet mapping


def sync_to_pybullet(
    arm: AuroraArm,
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
        max_velocity = (
            FINGER_PREVIEW_SPEED_RAD_S
            if is_finger
            else max_velocity_rad_s(name, LEFT_ARM_LIMITS_DEG)
        )

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
    "jLeftShoulder_rotx": "shoulder_x",
    "jLeftShoulder_roty": "shoulder_y",
    "jLeftShoulder_rotz": "shoulder_z",
    "jLeftElbow_roty": "elbow_x",
    "jLeftWrist_rotation": "wrist_rotation",
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
ARM_JOINT_NAMES = frozenset({"shoulder_x", "shoulder_y", "shoulder_z", "elbow_x", "wrist_rotation"})
LOGICAL_TO_URDF_JOINT = {
    logical: urdf for urdf, logical in URDF_TO_LOGICAL_JOINT.items() if logical in ARM_JOINT_NAMES
}


def build_joint_index_map(robot_id: int, client) -> dict[str, int]:
    """Map logical controller names to their indices in the left-arm URDF."""
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
