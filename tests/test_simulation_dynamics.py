"""Validate the migrated URDF against PyBullet's geometry and dynamics."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation
import math


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "simulation"))

from sim.dynamics import ArmDynamics
from sim.controller_params import ARM_ACTUATOR_INFO, ARM_MOTOR_FORCE_NM
from sim.contact_feedback import estimate_contact_force_n, grasp_is_ready
from sim.robot_model import (
    LEFT_ARM_JOINTS,
    LEFT_GRIP_OFFSET_M,
    LEFT_HAND_LINK,
    RIGHT_ARM_JOINTS,
    RIGHT_GRIP_OFFSET_M,
    RIGHT_HAND_LINK,
    inspect_model,
    link_name_index,
    load_left_arm,
    load_right_arm,
)
from interactive.kinematics import calculate_ik_angles, is_reachable
from interactive.torque_graph import TorqueHistory
from ai.sensor_fusion import interactive_control_targets
from sim.joint_limits import LEFT_ARM_LIMITS_DEG, RIGHT_ARM_LIMITS_DEG, finger_joint_angles_rad


class SimulationDynamicsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.client_id = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)
        self.robot = load_right_arm(p)
        self.dynamics = ArmDynamics(self.robot, p)

    def tearDown(self) -> None:
        p.disconnect(self.client_id)

    def pose(self, shoulder_y=0.35, elbow=-0.4) -> np.ndarray:
        q = np.zeros(self.dynamics.dof)
        q[:5] = [-0.30, shoulder_y, 0.10, elbow, 0.50]
        return q

    def reset_pose(self, q) -> None:
        for joint, angle in zip(self.dynamics.joints, q):
            p.resetJointState(self.robot, joint, float(angle))

    def tip_state(self):
        state = p.getLinkState(self.robot, self.dynamics.hand_link,
                               computeForwardKinematics=1)
        return p.multiplyTransforms(
            state[4], state[5], RIGHT_GRIP_OFFSET_M, (0, 0, 0, 1)
        )

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
        for urdf_name, logical_name in zip(RIGHT_ARM_JOINTS, logical):
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
        by_name = {
            row["joint"]: row
            for row in inspect_model(robot, p, "left_arm.urdf")
        }
        for urdf_name, logical_name in zip(LEFT_ARM_JOINTS, logical):
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
        for joint, angle in zip(self.dynamics.joints, right_q):
            p.resetJointState(self.robot, joint, float(angle))
        for joint, angle in zip(left_dynamics.joints, left_q):
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
            left_state = p.getLinkState(
                left, left_links[left_name], computeForwardKinematics=True
            )
            np.testing.assert_allclose(
                left_state[4], reflection @ np.asarray(right_state[4]), atol=2e-5
            )
            right_rotation = np.asarray(
                p.getMatrixFromQuaternion(right_state[5])
            ).reshape(3, 3)
            left_rotation = np.asarray(
                p.getMatrixFromQuaternion(left_state[5])
            ).reshape(3, 3)
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
        q[:5] = np.radians([-18.0, 15.0, 0.0, 20.0, 0.0])
        for joint, angle in zip(dynamics.joints, q):
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
        hand_orientation = p.getLinkState(
            robot, hand_index, computeForwardKinematics=True
        )[5]
        hand_roll_deg = math.degrees(p.getEulerFromQuaternion(hand_orientation)[0])
        self.assertAlmostEqual(hand_roll_deg, 0.0, delta=0.5)

        # A decreases shoulder-Z and moves the mirrored arm farther left.
        p.resetJointState(robot, dynamics.joints[0], math.radians(-23.0))
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
            ):
                value = LEFT_ARM_LIMITS_DEG[name].clamp(commanded[name])
                p.resetJointState(robot, joint, math.radians(value))
            return np.asarray(dynamics.end_effector_state()["position_m"])

        arm_down = apply({"shoulder_flexion": 0.0, "shoulder_abduction": 5.0})
        arm_forward = apply({"shoulder_flexion": 90.0, "shoulder_abduction": 5.0})
        arm_outward = apply({"shoulder_flexion": 90.0, "shoulder_abduction": 40.0})
        elbow_bent = apply({
            "shoulder_flexion": 0.0,
            "shoulder_abduction": 5.0,
            "elbow_flexion": 60.0,
        })
        axial_right = apply({
            "shoulder_flexion": 90.0,
            "shoulder_abduction": 5.0,
            "shoulder_rotation_proxy": 60.0,
            "elbow_flexion": 60.0,
        })
        axial_left = apply({
            "shoulder_flexion": 90.0,
            "shoulder_abduction": 5.0,
            "shoulder_rotation_proxy": -60.0,
            "elbow_flexion": 60.0,
        })

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
            / "src" / "simulation" / "sim" / "arm" / "left_arm.urdf"
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
            Path(__file__).resolve().parents[1]
            / "src" / "simulation" / "sim" / "arm"
        )
        for filename, shoulder_link in (
            ("right_arm.urdf", "right_shoulder_roty"),
            ("left_arm.urdf", "left_shoulder_roty"),
        ):
            root = ET.parse(arm_directory / filename).getroot()
            link = root.find(f"./link[@name='{shoulder_link}']")
            self.assertIsNotNone(link)
            shoulder_meshes = {
                Path(mesh.get("filename")).name
                for mesh in link.findall("visual/geometry/mesh")
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
                Path(mesh.get("filename")).name
                for mesh in upper.findall("visual/geometry/mesh")
            }
            self.assertEqual(upper_meshes, {"bicep.stl", "bicepcover.stl"})

    def test_jacobian_matches_urdf_fk_position_and_orientation(self) -> None:
        q = self.pose()
        self.reset_pose(q)
        linear, angular = self.dynamics.jacobian(q)
        position, orientation = self.tip_state()
        # getLinkState/multiplyTransforms return float32 world coordinates;
        # 1e-3 rad keeps finite differences above their quantization noise.
        epsilon = 1e-3
        for column, joint in enumerate(self.dynamics.joints[:5]):
            p.resetJointState(self.robot, joint, float(q[column] + epsilon))
            next_position, next_orientation = self.tip_state()
            p.resetJointState(self.robot, joint, float(q[column]))
            linear_fd = (np.asarray(next_position) - position) / epsilon
            angular_fd = (
                Rotation.from_quat(next_orientation)
                * Rotation.from_quat(orientation).inv()
            ).as_rotvec() / epsilon
            np.testing.assert_allclose(linear[:, column], linear_fd, atol=8e-4)
            np.testing.assert_allclose(angular[:, column], angular_fd, atol=8e-4)

    def test_mass_matrix_inverse_dynamics_and_payload(self) -> None:
        for q in (self.pose(), self.pose(0.65, -0.7)):
            qd = np.zeros(self.dynamics.dof)
            qdd = np.zeros(self.dynamics.dof)
            qd[:5] = [0.1, -0.2, 0.1, 0.3, -0.1]
            qdd[:5] = [0.2, 0.1, -0.1, 0.2, 0.05]
            for payload in (0.0, 0.25, 0.5, 1.0):
                terms = self.dynamics.terms(q, qd, qdd, payload)
                matrix = terms["mass_matrix"]
                self.assertEqual(matrix.shape, (18, 18))
                np.testing.assert_allclose(matrix, matrix.T, atol=1e-9)
                self.assertGreater(float(np.min(np.linalg.eigvalsh(matrix))), 0)
                np.testing.assert_allclose(
                    terms["inverse_dynamics_torque"],
                    terms["reconstructed_torque"], atol=1e-4,
                )
                np.testing.assert_allclose(
                    self.dynamics.inverse_dynamics(q, np.zeros(18), np.zeros(18), payload),
                    terms["gravity_torque"], atol=1e-9,
                )
        neutral = self.pose(0.0, 0.0)
        gravity_no_payload = self.dynamics.terms(neutral, np.zeros(18), np.zeros(18))["gravity_torque"]
        gravity_with_payload = self.dynamics.terms(neutral, np.zeros(18), np.zeros(18), 1.0)["gravity_torque"]
        self.assertGreater(abs(gravity_with_payload[1]), abs(gravity_no_payload[1]))
        quarter_payload = self.dynamics.terms(neutral, np.zeros(18), np.zeros(18), 0.25)["gravity_torque"]
        np.testing.assert_allclose(
            quarter_payload - gravity_no_payload,
            0.25 * (gravity_with_payload - gravity_no_payload), atol=1e-8,
        )
        folded = neutral.copy()
        folded[3] = -0.9
        folded_gravity = self.dynamics.terms(folded, np.zeros(18), np.zeros(18))["gravity_torque"]
        self.assertGreater(abs(gravity_no_payload[1]), abs(folded_gravity[1]))

    def test_unpowered_arm_moves_under_gravity(self) -> None:
        q = self.pose(0.25, -0.4)
        self.reset_pose(q)
        for joint in self.dynamics.joints:
            p.setJointMotorControl2(self.robot, joint, p.VELOCITY_CONTROL, force=0)
        p.setTimeStep(1.0 / 240.0)
        for _ in range(60):
            p.stepSimulation()
        later, _, _ = self.dynamics.joint_state()
        self.assertGreater(float(np.linalg.norm(later[:5] - q[:5])), 1e-3)

    def test_position_motor_moves_without_joint_teleportation(self) -> None:
        q = self.pose(0.25, -0.4)
        self.reset_pose(q)
        p.setTimeStep(1.0 / 240.0)
        for column, joint in enumerate(self.dynamics.joints):
            target = float(q[column] + 0.2) if column == 1 else float(q[column])
            p.setJointMotorControl2(
                self.robot, joint, p.POSITION_CONTROL, targetPosition=target,
                positionGain=0.05, velocityGain=1.0, force=20.0,
                maxVelocity=0.35,
            )
        p.stepSimulation()
        first, _, _ = self.dynamics.joint_state()
        self.assertGreater(first[1], q[1])
        self.assertLess(first[1], q[1] + 0.02)
        for _ in range(119):
            p.stepSimulation()
        later, _, _ = self.dynamics.joint_state()
        self.assertGreater(later[1], first[1] + 0.05)
        self.assertLess(later[1], q[1] + 0.201)

    def test_dynamic_grasp_constraint_keeps_payload_at_hand(self) -> None:
        q = self.pose(0.25, -0.4)
        self.reset_pose(q)
        tip = self.dynamics.end_effector_state()["position_m"]
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.02, 0.02, 0.02))
        payload = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=collision,
                                    basePosition=tip)
        for link in range(-1, p.getNumJoints(self.robot)):
            p.setCollisionFilterPair(self.robot, payload, link, -1, 0)
        hand_state = p.getLinkState(self.robot, self.dynamics.hand_link,
                                    computeForwardKinematics=1)
        inverse_hand = p.invertTransform(hand_state[0], hand_state[1])
        parent_position, parent_orientation = p.multiplyTransforms(
            *inverse_hand, tip, (0, 0, 0, 1)
        )
        constraint = p.createConstraint(
            self.robot, self.dynamics.hand_link, payload, -1, p.JOINT_FIXED,
            (0, 0, 0), parent_position, (0, 0, 0),
            parent_orientation, (0, 0, 0, 1),
        )
        p.changeConstraint(constraint, maxForce=100, erp=0.8)
        for column, joint in enumerate(self.dynamics.joints):
            p.setJointMotorControl2(
                self.robot, joint, p.POSITION_CONTROL,
                targetPosition=float(q[column]), force=20.0,
            )
        p.setTimeStep(1.0 / 240.0)
        for _ in range(120):
            p.stepSimulation()
        payload_position, _ = p.getBasePositionAndOrientation(payload)
        hand_position = self.dynamics.end_effector_state()["position_m"]
        self.assertLess(np.linalg.norm(np.asarray(payload_position) - hand_position), 0.025)

    def test_planar_geometric_ik_uses_model_grip_length(self) -> None:
        robot = load_left_arm(p)
        dynamics = ArmDynamics(
            robot,
            p,
            arm_joint_names=LEFT_ARM_JOINTS,
            hand_link=LEFT_HAND_LINK,
            grip_offset=LEFT_GRIP_OFFSET_M,
        )
        q = np.zeros(dynamics.dof)
        q[1] = 0.35
        q[3] = 0.4
        for joint, angle in zip(dynamics.joints, q):
            p.resetJointState(robot, joint, float(angle))
        x, y, z = dynamics.end_effector_state()["position_m"]
        shoulder_deg, elbow_deg = calculate_ik_angles(math.hypot(x, y), z - 0.7)
        self.assertLess(abs(shoulder_deg - math.degrees(q[1])), 3.0)
        self.assertLess(abs(elbow_deg - math.degrees(q[3])), 3.0)

    def test_left_arm_keeps_the_existing_table_target_reachable(self) -> None:
        target_from_base = (-0.2, 0.6, -0.2)
        reachable, reason = is_reachable(target_from_base)
        self.assertTrue(reachable, reason)
        shoulder, elbow = calculate_ik_angles(math.hypot(-0.2, 0.6), -0.2)
        self.assertGreaterEqual(shoulder, LEFT_ARM_LIMITS_DEG["shoulder_y"].lower)
        self.assertLessEqual(shoulder, LEFT_ARM_LIMITS_DEG["shoulder_y"].upper)
        self.assertGreaterEqual(elbow, LEFT_ARM_LIMITS_DEG["elbow_x"].lower)
        self.assertLessEqual(elbow, LEFT_ARM_LIMITS_DEG["elbow_x"].upper)

    def test_finger_model_has_stable_open_and_closed_poses(self) -> None:
        open_hand = finger_joint_angles_rad(0.0)
        closed_hand = finger_joint_angles_rad(1.5)
        self.assertLess(open_hand["thumb_1"], 0.0)
        self.assertEqual(open_hand["thumb_2"], 0.0)
        self.assertLess(closed_hand["thumb_1"], open_hand["thumb_1"])
        self.assertGreater(closed_hand["thumb_2"], open_hand["thumb_2"])
        self.assertGreater(closed_hand["thumb_3"], open_hand["thumb_3"])
        for finger in ("index", "middle", "ring", "pinky"):
            self.assertEqual(open_hand[f"{finger}_1"], 0.0)
            self.assertGreater(closed_hand[f"{finger}_1"], 0.0)

    def test_fingertip_force_uses_contact_or_penetration_signal(self) -> None:
        self.assertEqual(estimate_contact_force_n(0.0, 0.001, 500.0), 0.0)
        self.assertEqual(estimate_contact_force_n(1.2, 0.0, 500.0), 1.2)
        self.assertEqual(estimate_contact_force_n(0.0, -0.002, 500.0), 1.0)
        with self.assertRaises(ValueError):
            estimate_contact_force_n(0.0, 0.0, -1.0)

    def test_assisted_grasp_accepts_contact_or_close_proximity(self) -> None:
        settings = {
            "min_curl": 0.45,
            "capture_distance_m": 0.14,
            "min_fingertip_contacts": 2,
        }
        self.assertTrue(grasp_is_ready(0.5, 0.20, 2, **settings))
        self.assertTrue(grasp_is_ready(0.5, 0.12, 0, **settings))
        self.assertFalse(grasp_is_ready(0.3, 0.12, 2, **settings))
        self.assertFalse(grasp_is_ready(0.5, 0.20, 1, **settings))

    def test_finger_curl_is_proportional_and_bounded(self) -> None:
        half = finger_joint_angles_rad(0.75)
        closed = finger_joint_angles_rad(1.5)
        for name in closed:
            if name.startswith("thumb"):
                continue
            self.assertAlmostEqual(half[name], closed[name] / 2.0)
            self.assertLessEqual(abs(closed[name]), 1.18)

    def test_each_simulated_arm_joint_has_actuator_metadata(self) -> None:
        self.assertEqual(set(ARM_ACTUATOR_INFO), set(ARM_MOTOR_FORCE_NM))
        for actuator in ARM_ACTUATOR_INFO.values():
            self.assertTrue(actuator["model"])
            self.assertTrue(actuator["published_rating"])


class TorqueHistoryTests(unittest.TestCase):
    def test_history_keeps_recent_samples_and_scales_graph_points(self) -> None:
        history = TorqueHistory(("shoulder", "elbow"), max_samples=3)
        for value in (1.0, 2.0, 3.0, 4.0):
            history.append({"shoulder": value, "elbow": -value})
        self.assertEqual(list(history.values["shoulder"]), [2.0, 3.0, 4.0])
        points = history.points("shoulder", 0, 100, 0, 100, limit=4.0)
        self.assertEqual(points[0::2], [0.0, 50.0, 100.0])
        self.assertEqual(points[-1], 0.0)


if __name__ == "__main__":
    unittest.main()
