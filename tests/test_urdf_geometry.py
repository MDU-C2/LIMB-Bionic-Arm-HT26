# Christian, Majd
"""Validate the migrated URDF against PyBullet's geometry and dynamics."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pybullet as p
from simulation_fixture import SimulationPhysicsCase

from aurora.sensors.fusion import interactive_control_targets
from aurora.simulation.interactive.settings import READY_POSE_DEG
from aurora.simulation.model.dynamics import ArmDynamics
from aurora.simulation.model.joint_limits import LEFT_ARM_LIMITS_DEG, RIGHT_ARM_LIMITS_DEG
from aurora.simulation.model.robot_model import (
    LEFT_ARM_JOINTS,
    LEFT_GRIP_OFFSET_M,
    LEFT_HAND_LINK,
    RIGHT_ARM_JOINTS,
    RIGHT_HAND_LINK,
    inspect_model,
    link_name_index,
    load_left_arm,
)


class UrdfGeometryTests(SimulationPhysicsCase):
    """Model geometry, mirrored coordinates, and rendered movement."""

    def test_model_and_joint_mapping(self) -> None:
        rows = inspect_model(self.robot, p)
        by_name = {row["joint"]: row for row in rows}
        self.assertEqual(tuple(self.dynamics.names[:5]), RIGHT_ARM_JOINTS)
        self.assertEqual(self.dynamics.dof, 18)
        self.assertAlmostEqual(by_name["jRightWrist_rotation"]["joint_origin_xyz"][0], 0.310)
        self.assertEqual(by_name["right_hand_mount"]["type"], "fixed")
        self.assertGreater(by_name["jRightElbow_roty"]["mass_kg"], 0)

    def test_right_urdf_limits_match_central_controller_limits(self) -> None:
        logical = ("shoulder_z", "shoulder_y", "shoulder_x", "elbow_x", "wrist_rotation")
        by_name = {row["joint"]: row for row in inspect_model(self.robot, p)}
        for urdf_name, logical_name in zip(RIGHT_ARM_JOINTS, logical, strict=False):
            lower, upper = by_name[urdf_name]["limits_rad"]
            configured = RIGHT_ARM_LIMITS_DEG[logical_name]
            self.assertAlmostEqual(math.degrees(lower), configured.lower, places=2)
            self.assertAlmostEqual(math.degrees(upper), configured.upper, places=2)

    def test_left_urdf_matches_interactive_joint_mapping_and_limits(self) -> None:
        robot = load_left_arm(p)
        dynamics = ArmDynamics(
            robot,
            p,
            arm_joint_names=LEFT_ARM_JOINTS,
            hand_link=LEFT_HAND_LINK,
            grip_offset=LEFT_GRIP_OFFSET_M,
        )
        self.assertEqual(tuple(dynamics.names[:5]), LEFT_ARM_JOINTS)
        logical = ("shoulder_z", "shoulder_y", "shoulder_x", "elbow_x", "wrist_rotation")
        by_name = {row["joint"]: row for row in inspect_model(robot, p, "left_arm.urdf")}
        for urdf_name, logical_name in zip(LEFT_ARM_JOINTS, logical, strict=False):
            lower, upper = by_name[urdf_name]["limits_rad"]
            configured = LEFT_ARM_LIMITS_DEG[logical_name]
            self.assertAlmostEqual(math.degrees(lower), configured.lower, places=2)
            self.assertAlmostEqual(math.degrees(upper), configured.upper, places=2)

    def test_left_model_is_an_exact_reflection_of_the_right_model(self) -> None:
        left = load_left_arm(p)
        left_dynamics = ArmDynamics(
            left,
            p,
            arm_joint_names=LEFT_ARM_JOINTS,
            hand_link=LEFT_HAND_LINK,
            grip_offset=LEFT_GRIP_OFFSET_M,
        )
        right_q = np.zeros(self.dynamics.dof)
        left_q = np.zeros(left_dynamics.dof)
        right_q[:5] = np.radians([-22.0, 31.0, 17.0, -42.0, 33.0])
        left_q[:5] = np.radians([-22.0, 31.0, 17.0, 42.0, 33.0])
        finger_pose = np.linspace(0.05, 0.65, self.dynamics.dof - 5)
        right_q[5:] = finger_pose
        left_q[5:] = finger_pose
        for joint, angle in zip(self.dynamics.joints, right_q, strict=False):
            p.resetJointState(self.robot, joint, float(angle))
        for joint, angle in zip(left_dynamics.joints, left_q, strict=False):
            p.resetJointState(left, joint, float(angle))

        right_links = link_name_index(self.robot, p)
        left_links = link_name_index(left, p)
        reflection = np.diag([-1.0, 1.0, 1.0])
        link_pairs = (
            (RIGHT_HAND_LINK, LEFT_HAND_LINK),
            ("thumb_link_3", "thumb_link_3"),
            ("index_link_3", "index_link_3"),
            ("middle_link_3", "middle_link_3"),
            ("ring_link_3", "ring_link_3"),
            ("pinky_link_3", "pinky_link_3"),
        )
        for right_name, left_name in link_pairs:
            right_state = p.getLinkState(
                self.robot, right_links[right_name], computeForwardKinematics=True
            )
            left_state = p.getLinkState(left, left_links[left_name], computeForwardKinematics=True)
            np.testing.assert_allclose(
                left_state[4], reflection @ np.asarray(right_state[4]), atol=2e-5
            )
            right_rotation = np.asarray(p.getMatrixFromQuaternion(right_state[5])).reshape(3, 3)
            left_rotation = np.asarray(p.getMatrixFromQuaternion(left_state[5])).reshape(3, 3)
            np.testing.assert_allclose(
                left_rotation,
                reflection @ right_rotation @ reflection,
                atol=2e-5,
            )

    def test_left_ready_pose_has_neutral_wrist_and_a_moves_left(self) -> None:
        robot = load_left_arm(p)
        dynamics = ArmDynamics(
            robot,
            p,
            arm_joint_names=LEFT_ARM_JOINTS,
            hand_link=LEFT_HAND_LINK,
            grip_offset=LEFT_GRIP_OFFSET_M,
        )
        q = np.zeros(dynamics.dof)
        q[:5] = np.radians(
            [
                READY_POSE_DEG[name]
                for name in (
                    "shoulder_z",
                    "shoulder_y",
                    "shoulder_x",
                    "elbow_x",
                    "wrist_rotation",
                )
            ]
        )
        for joint, angle in zip(dynamics.joints, q, strict=False):
            p.resetJointState(robot, joint, float(angle))
        ready = np.asarray(dynamics.end_effector_state()["position_m"])
        self.assertLess(ready[0], 0.0)
        self.assertLess(np.linalg.norm(ready - [-0.2, 0.6, 0.5]), 0.18)

        # HT25's neutral/reset command is wrist zero.  No invented 70-degree
        # mounting correction is baked into the left model.
        hand_index = next(
            index
            for index in range(p.getNumJoints(robot))
            if p.getJointInfo(robot, index)[12].decode("utf-8") == LEFT_HAND_LINK
        )
        hand_orientation = p.getLinkState(robot, hand_index, computeForwardKinematics=True)[5]
        hand_roll_deg = math.degrees(p.getEulerFromQuaternion(hand_orientation)[0])
        self.assertAlmostEqual(hand_roll_deg, 0.0, delta=0.5)

        # A decreases shoulder-Z and moves the mirrored arm farther left.
        p.resetJointState(robot, dynamics.joints[0], math.radians(READY_POSE_DEG["shoulder_z"] - 5))
        moved = np.asarray(dynamics.end_effector_state()["position_m"])
        self.assertLess(moved[0], ready[0])

    def test_live_anatomical_targets_move_the_rendered_left_arm_correctly(self) -> None:
        robot = load_left_arm(p)
        dynamics = ArmDynamics(
            robot,
            p,
            arm_joint_names=LEFT_ARM_JOINTS,
            hand_link=LEFT_HAND_LINK,
            grip_offset=LEFT_GRIP_OFFSET_M,
        )

        def apply(angles):
            targets = interactive_control_targets(angles)
            commanded = {
                "shoulder_z": targets.get("shoulder_z", -5.0),
                "shoulder_y": targets.get("shoulder_y", 0.0),
                "shoulder_x": targets.get("shoulder_x", 0.0),
                "elbow_x": targets.get("elbow_x", 0.0),
                "wrist_rotation": 0.0,
            }
            for joint, name in zip(
                dynamics.joints[:5],
                ("shoulder_z", "shoulder_y", "shoulder_x", "elbow_x", "wrist_rotation"),
                strict=False,
            ):
                value = LEFT_ARM_LIMITS_DEG[name].clamp(commanded[name])
                p.resetJointState(robot, joint, math.radians(value))
            return np.asarray(dynamics.end_effector_state()["position_m"])

        arm_down = apply({"shoulder_flexion": 0.0, "shoulder_abduction": 5.0})
        arm_forward = apply({"shoulder_flexion": 90.0, "shoulder_abduction": 5.0})
        arm_outward = apply({"shoulder_flexion": 90.0, "shoulder_abduction": 40.0})
        elbow_bent = apply(
            {
                "shoulder_flexion": 0.0,
                "shoulder_abduction": 5.0,
                "elbow_flexion": 60.0,
            }
        )
        axial_right = apply(
            {
                "shoulder_flexion": 90.0,
                "shoulder_abduction": 5.0,
                "shoulder_rotation_proxy": 60.0,
                "elbow_flexion": 60.0,
            }
        )
        axial_left = apply(
            {
                "shoulder_flexion": 90.0,
                "shoulder_abduction": 5.0,
                "shoulder_rotation_proxy": -60.0,
                "elbow_flexion": 60.0,
            }
        )

        self.assertGreater(arm_forward[1], arm_down[1] + 0.4)
        self.assertGreater(arm_forward[2], arm_down[2] + 0.4)
        self.assertLess(arm_outward[0], arm_forward[0] - 0.3)
        self.assertGreater(elbow_bent[1], arm_down[1] + 0.2)
        self.assertGreater(elbow_bent[2], arm_down[2] + 0.2)
        # A positive camera axial proxy is the tracked hand moving right.  The
        # left-arm URDF must follow it right rather than mirroring it left.
        self.assertGreater(axial_right[0], axial_left[0] + 0.5)

    def test_left_arm_uses_the_established_orange_and_dark_appearance(self) -> None:
        urdf = (
            Path(__file__).resolve().parents[1]
            / "src"
            / "aurora"
            / "simulation"
            / "model"
            / "arm"
            / "left_arm.urdf"
        )
        root = ET.parse(urdf).getroot()
        material_for_mesh = {
            Path(mesh.get("filename")).name: visual.find("material").get("name")
            for visual in root.findall(".//visual")
            if (mesh := visual.find("geometry/mesh")) is not None
            and visual.find("material") is not None
        }
        self.assertEqual(material_for_mesh["r_shoulder_base.stl"], "dark")
        self.assertEqual(material_for_mesh["r_shoulder.stl"], "orange")
        self.assertEqual(material_for_mesh["bicep.stl"], "orange")
        self.assertEqual(material_for_mesh["bicepcover.stl"], "dark")
        self.assertEqual(material_for_mesh["r_forearm.stl"], "orange")
        self.assertEqual(material_for_mesh["r_hand.stl"], "dark")

    def test_shoulder_base_and_shell_share_the_elevation_stage(self) -> None:
        arm_directory = (
            Path(__file__).resolve().parents[1] / "src" / "aurora" / "simulation" / "model" / "arm"
        )
        for filename, shoulder_link in (
            ("right_arm.urdf", "right_shoulder_roty"),
            ("left_arm.urdf", "left_shoulder_roty"),
        ):
            root = ET.parse(arm_directory / filename).getroot()
            link = root.find(f"./link[@name='{shoulder_link}']")
            self.assertIsNotNone(link)
            shoulder_meshes = {
                Path(mesh.get("filename")).name for mesh in link.findall("visual/geometry/mesh")
            }
            self.assertEqual(
                shoulder_meshes,
                {"r_shoulder_base.stl", "r_shoulder.stl"},
            )

            # The bicep stays on the next axial-rotation link.  Its established
            # 116 mm origin therefore remains attached to this assembly while
            # the shoulder elevates instead of crossing a fixed base mesh.
            upper_name = filename.removesuffix("_arm.urdf") + "_upper_arm"
            upper = root.find(f"./link[@name='{upper_name}']")
            self.assertIsNotNone(upper)
            upper_meshes = {
                Path(mesh.get("filename")).name for mesh in upper.findall("visual/geometry/mesh")
            }
            self.assertEqual(upper_meshes, {"bicep.stl", "bicepcover.stl"})
