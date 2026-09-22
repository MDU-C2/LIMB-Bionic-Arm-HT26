"""PyBullet URDF loading and joint/link inspection for the LIMB arm models."""

from __future__ import annotations

from contextlib import contextmanager
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pybullet as p


SIM_DIR = Path(__file__).resolve().parent
RIGHT_ARM_JOINTS = (
    "jRightShoulder_rotz",
    "jRightShoulder_roty",
    "jRightShoulder_rotx",
    "jRightElbow_roty",
    "jRightWrist_rotation",
)
RIGHT_HAND_LINK = "right_hand"
UPPER_ARM_LENGTH_M = 0.305
FOREARM_LENGTH_M = 0.310
RIGHT_GRIP_OFFSET_M = (0.105, 0.0, 0.0)


@contextmanager
def model_directory():
    """Use ASCII relative mesh paths when loading URDFs on Windows."""
    previous = Path.cwd()
    os.chdir(SIM_DIR)
    try:
        yield
    finally:
        os.chdir(previous)


def load_right_arm(
    client=p,
    base_position=(0.0, 0.0, 0.7),
    base_orientation=(0.0, 0.0, 0.0, 1.0),
) -> int:
    """Load the current five-actuator right arm at the task-scene base pose."""
    with model_directory():
        return client.loadURDF(
            "arm/right_arm.urdf", base_position, base_orientation,
            useFixedBase=True,
        )


def link_name_index(body_id: int, client=p) -> dict[str, int]:
    result = {client.getBodyInfo(body_id)[0].decode("utf-8"): -1}
    result.update({
        client.getJointInfo(body_id, index)[12].decode("utf-8"): index
        for index in range(client.getNumJoints(body_id))
    })
    return result


def moving_joint_indices(body_id: int, client=p) -> tuple[int, ...]:
    """Return all movable joints in generalized-coordinate order."""
    entries = [
        (client.getJointInfo(body_id, index)[3], index)
        for index in range(client.getNumJoints(body_id))
        if client.getJointInfo(body_id, index)[3] >= 0
    ]
    return tuple(index for _, index in sorted(entries))


def inspect_model(body_id: int, client=p) -> list[dict[str, object]]:
    """Return URDF-origin, motion-limit, mass and inertia data for every joint."""
    rows = []
    # PyBullet reports parent frame positions relative to the parent's inertial
    # frame, which are not the xyz values written in the URDF. Keep both.
    urdf_joints = {
        element.attrib["name"]: element
        for element in ET.parse(SIM_DIR / "arm" / "right_arm.urdf").getroot().findall("joint")
    }
    base_name = client.getBodyInfo(body_id)[0].decode("utf-8")
    for index in range(client.getNumJoints(body_id)):
        info = client.getJointInfo(body_id, index)
        dynamics = client.getDynamicsInfo(body_id, index)
        parent_index = info[16]
        parent_name = (
            base_name if parent_index < 0
            else client.getJointInfo(body_id, parent_index)[12].decode("utf-8")
        )
        urdf_joint = urdf_joints.get(info[1].decode("utf-8"))
        origin = None if urdf_joint is None else urdf_joint.find("origin")
        rows.append({
            "index": index,
            "q_index": info[3],
            "joint": info[1].decode("utf-8"),
            "parent_link": parent_name,
            "child_link": info[12].decode("utf-8"),
            "type": {
                client.JOINT_REVOLUTE: "revolute",
                client.JOINT_PRISMATIC: "prismatic",
                client.JOINT_FIXED: "fixed",
            }.get(info[2], str(info[2])),
            "axis": list(info[13]),
            "joint_origin_xyz": (
                [float(value) for value in origin.get("xyz", "0 0 0").split()]
                if origin is not None else [0.0, 0.0, 0.0]
            ),
            "joint_origin_rpy": (
                [float(value) for value in origin.get("rpy", "0 0 0").split()]
                if origin is not None else [0.0, 0.0, 0.0]
            ),
            "bullet_parent_frame_xyz": list(info[14]),
            "bullet_parent_frame_quaternion": list(info[15]),
            "limits_rad": [info[8], info[9]] if info[3] >= 0 else None,
            "max_effort_Nm": info[10],
            "max_speed_rad_s": info[11],
            "damping": info[6],
            "friction": info[7],
            "mass_kg": dynamics[0],
            "local_com_xyz_m": list(dynamics[3]),
            "local_inertia_xyz_kg_m2": list(dynamics[2]),
            "local_inertial_quaternion": list(dynamics[4]),
        })
    return rows
