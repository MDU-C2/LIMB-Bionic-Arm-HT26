# Majd, Christian
"""PyBullet task scene and target collision helpers."""

from __future__ import annotations

import math
import os

import pybullet as p
import pybullet_data
from sim.contact_feedback import estimate_contact_force_n
from sim.dynamics import ArmDynamics
from sim.robot_model import (
    LEFT_ARM_JOINTS,
    LEFT_GRIP_OFFSET_M,
    LEFT_HAND_LINK,
)

from .arm_control import (
    LimbArm,
    ManualMotion,
    build_joint_index_map,
    build_link_name_index_map,
    resolve_link_index,
    set_live_alignment_pose,
    set_ready_pose,
    sync_to_pybullet,
)
from .settings import (
    ARM_BASE_POSITION_M,
    ARM_BASE_YAW_DEG,
    CONTACT_STIFFNESS_N_PER_M,
    FINGER_JOINT_TOKENS,
    OVERVIEW_CAMERA_DISTANCE_M,
    OVERVIEW_CAMERA_PITCH_DEG,
    OVERVIEW_CAMERA_TARGET,
    OVERVIEW_CAMERA_YAW_DEG,
    PHYSICS_SUBSTEPS,
    SIMULATION_FREQUENCY_HZ,
    TABLE_CENTER_XY,
    TABLE_THICKNESS_M,
    TABLE_TOP_HEIGHT_M,
    TARGET_HEIGHT_M,
    TARGET_NORMAL_MASS_KG,
    TARGET_RADIUS_M,
)


class SceneMixin:
    """Build the task scene and manage target collisions."""

    def initialize_scene(self) -> None:
        """Create the PyBullet world and cup."""
        simulator_dir = os.path.abspath(os.path.dirname(__file__))
        self.model_dir = os.path.abspath(os.path.join(simulator_dir, "..", "sim"))
        urdf_path = os.path.join(self.model_dir, "arm", "left_arm.urdf")

        print(f"Simulator mode: {self.args.mode}")
        print(f"Simulator directory: {simulator_dir}")
        print(f"URDF path: {urdf_path}")

        try:
            physics_client = p.connect(p.GUI)
        except p.error as error:
            raise SystemExit(f"Could not start PyBullet: {error}") from error

        if physics_client < 0:
            raise SystemExit("Could not connect to the PyBullet GUI.")

        self.preview_visible = False
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
        self.initialize_table()

        # Cup starts on the table's left, to the right of the left shoulder.
        margin = TARGET_RADIUS_M + 0.01
        self.target_start_position = [
            max(self.table_bounds[0][0] + margin, -0.20),
            max(self.table_bounds[0][1] + margin,
                min(self.table_bounds[1][1] - margin, 0.60)),
            TABLE_TOP_HEIGHT_M + TARGET_HEIGHT_M / 2 + 0.002,
        ]
        self.target_destination_position = [
            min(self.table_bounds[1][0] - margin, 0.04),
            max(self.table_bounds[0][1] + margin,
                min(self.table_bounds[1][1] - margin, 0.48)),
            self.target_start_position[2],
        ]
        destination_visual = p.createVisualShape(
            p.GEOM_CYLINDER, radius=TARGET_RADIUS_M + 0.005, length=0.001,
            rgbaColor=[0.18, 0.65, 0.35, 0.65],
        )
        # A visual landing marker must never obstruct the arm or falling cup.
        self.destination_marker_body = p.createMultiBody(
            baseMass=0, baseVisualShapeIndex=destination_visual,
            basePosition=[*self.target_destination_position[:2], TABLE_TOP_HEIGHT_M + 0.001],
        )
        self.target_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
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
        self.target_body = p.createMultiBody(
            baseMass=TARGET_NORMAL_MASS_KG,
            baseCollisionShapeIndex=cup_collision_shape,
            baseVisualShapeIndex=cup_visual_shape,
            basePosition=self.target_start_position,
            baseOrientation=self.target_start_orientation,
        )

    def initialize_table(self) -> None:
        """Use measured tabletop dimensions rather than the large demo URDF."""
        width = self.args.table_width_cm / 100
        depth = self.args.table_depth_cm / 100
        center_x, center_y = TABLE_CENTER_XY
        self.table_bounds = (
            (center_x - width / 2, center_y - depth / 2, TABLE_TOP_HEIGHT_M - TABLE_THICKNESS_M),
            (center_x + width / 2, center_y + depth / 2, TABLE_TOP_HEIGHT_M),
        )
        leg_half_width = 0.02
        leg_height = TABLE_TOP_HEIGHT_M - TABLE_THICKNESS_M
        sizes = [[width / 2, depth / 2, TABLE_THICKNESS_M / 2]]
        centers = [[center_x, center_y, TABLE_TOP_HEIGHT_M - TABLE_THICKNESS_M / 2]]
        for x_sign in (-1, 1):
            for y_sign in (-1, 1):
                sizes.append([leg_half_width, leg_half_width, leg_height / 2])
                centers.append([
                    center_x + x_sign * (width / 2 - leg_half_width),
                    center_y + y_sign * (depth / 2 - leg_half_width),
                    leg_height / 2,
                ])
        collision = p.createCollisionShapeArray(
            shapeTypes=[p.GEOM_BOX] * len(sizes), halfExtents=sizes,
            collisionFramePositions=centers,
        )
        visual = p.createVisualShapeArray(
            shapeTypes=[p.GEOM_BOX] * len(sizes), halfExtents=sizes,
            visualFramePositions=centers,
            rgbaColors=[[0.70, 0.54, 0.36, 1]] + [[0.22, 0.25, 0.29, 1]] * 4,
        )
        self.table_body = p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=collision, baseVisualShapeIndex=visual,
        )
        print(f"Tabletop: {width * 100:g} x {depth * 100:g} cm")

    def anchor_target_to_world(self) -> int:
        """Hold the target at its start pose until grasping begins."""
        constraint = p.createConstraint(
            parentBodyUniqueId=self.target_body,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            parentFrameOrientation=[0, 0, 0, 1],
            childFramePosition=self.target_start_position,
            childFrameOrientation=self.target_start_orientation,
        )
        p.changeConstraint(constraint, maxForce=10_000, erp=1.0)
        return constraint

    def initialize_arm(self) -> None:
        """Load the existing arm model and resolve its joints and links."""
        self.target_anchor_constraint = self.anchor_target_to_world()
        p.resetDebugVisualizerCamera(
            cameraDistance=OVERVIEW_CAMERA_DISTANCE_M,
            cameraYaw=OVERVIEW_CAMERA_YAW_DEG,
            cameraPitch=OVERVIEW_CAMERA_PITCH_DEG,
            cameraTargetPosition=OVERVIEW_CAMERA_TARGET,
        )

        try:
            print("Attempting to load robot...")
            base_orientation = p.getQuaternionFromEuler(
                [0, 0, math.radians(ARM_BASE_YAW_DEG)]
            )

            # PyBullet on Windows cannot open a non-ASCII absolute path reliably. Load
            # the repository-relative path while this directory is the working folder.
            previous_directory = os.getcwd()
            os.chdir(self.model_dir)
            try:
                p.setAdditionalSearchPath(".")
                self.robot_body = p.loadURDF(
                    "arm/left_arm.urdf",
                    ARM_BASE_POSITION_M,
                    base_orientation,
                    useFixedBase=True,
                )
            finally:
                os.chdir(previous_directory)
                p.setAdditionalSearchPath(pybullet_data.getDataPath())

            print("\nRobot loaded successfully!")

            p.changeDynamics(
                self.target_body,
                -1,
                mass=TARGET_NORMAL_MASS_KG,
                lateralFriction=2.5,
                spinningFriction=0.1,
                rollingFriction=0.1,
            )
            for joint_index in range(p.getNumJoints(self.robot_body)):
                p.changeDynamics(
                    self.robot_body,
                    joint_index,
                    lateralFriction=2.5,
                    spinningFriction=0.1,
                    rollingFriction=0.001,
                )
                if p.getJointInfo(self.robot_body, joint_index)[2] != p.JOINT_FIXED:
                    p.setJointMotorControl2(
                        self.robot_body, joint_index, p.VELOCITY_CONTROL, force=0.0
                    )

        except p.error as error:
            print("\n--- ERROR LOADING ROBOT ---")
            print("Check that left_arm.urdf and its STL files are in src/simulation/sim/arm.")
            print(f"PyBullet error: {error}")
            p.disconnect()
            raise SystemExit(1) from error

        # Controls and dashboard
        print("Initializing 'brain'...")

        self.arm = LimbArm()
        if self.args.control == "camera-imu":
            set_live_alignment_pose(self.arm)
        else:
            set_ready_pose(self.arm)
        self.manual_motion = ManualMotion()
        self.joint_indices = build_joint_index_map(self.robot_body, p)
        sync_to_pybullet(self.arm, self.robot_body, self.joint_indices, client=p, use_motors=False)
        self.dynamics = ArmDynamics(
            self.robot_body,
            p,
            arm_joint_names=LEFT_ARM_JOINTS,
            hand_link=LEFT_HAND_LINK,
            grip_offset=LEFT_GRIP_OFFSET_M,
        )
        self.link_indices_by_name = build_link_name_index_map(self.robot_body, p)

        try:
            self.hand_link_name, self.hand_link_index = resolve_link_index(
                self.link_indices_by_name,
                ("left_hand", "LeftHand"),
                "hand",
            )
            print(f"Link 'Hand' found: {self.hand_link_name} (Index: {self.hand_link_index})")

            wrist_link_name, wrist_link_index = resolve_link_index(
                self.link_indices_by_name,
                ("left_wrist_rotation", "left_hand"),
                "wrist",
            )
            print(f"Link 'Wrist' found: {wrist_link_name} (Index: {wrist_link_index})")

            elbow_link_name, elbow_link_index = resolve_link_index(
                self.link_indices_by_name,
                ("left_forearm", "LeftForeArm"),
                "forearm",
            )
            print(f"Link 'Elbow' found: {elbow_link_name} (Index: {elbow_link_index})")
        except KeyError as error:
            print(f"ERROR: {error}. Link positions will be unavailable.")
            print("Links found:", list(self.link_indices_by_name.keys()))
            self.hand_link_index = wrist_link_index = elbow_link_index = -1

        self.finger_contact_link_indices = {
            index
            for name, index in self.link_indices_by_name.items()
            if any(finger in name.lower() for finger in FINGER_JOINT_TOKENS)
        }
        self.fingertip_link_indices = {
            finger: self.link_indices_by_name.get(f"{finger}_link_3", -1)
            for finger in FINGER_JOINT_TOKENS
        }
        self.hand_contact_link_indices = set(self.finger_contact_link_indices)
        if self.hand_link_index >= 0:
            self.hand_contact_link_indices.add(self.hand_link_index)

    def set_target_robot_collision(self, enabled: bool) -> None:
        """A constrained payload should not collide with the hand carrying it."""
        for link_index in range(-1, p.getNumJoints(self.robot_body)):
            p.setCollisionFilterPair(
                self.robot_body, self.target_body, link_index, -1, int(enabled)
            )

    def target_contact_feedback(self) -> tuple[set[int], dict[str, float]]:
        """Return cup contacts and virtual force values at the five fingertips."""
        force_by_finger = {finger: 0.0 for finger in FINGER_JOINT_TOKENS}
        try:
            points = p.getContactPoints(bodyA=self.robot_body, bodyB=self.target_body)
        except p.error:
            return set(), force_by_finger

        contact_links = {point[3] for point in points if point[3] >= 0}
        finger_by_link = {
            link_index: finger
            for finger, link_index in self.fingertip_link_indices.items()
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

    def update_camera(self) -> None:
        """Apply the selected overview, shoulder, or hand view."""
        if self.camera_mode == "overview" and self.camera_changed:
            p.resetDebugVisualizerCamera(
                cameraDistance=OVERVIEW_CAMERA_DISTANCE_M,
                cameraYaw=self.default_cam_yaw,
                cameraPitch=self.default_cam_pitch,
                cameraTargetPosition=self.default_cam_target,
            )
        elif self.camera_mode == "shoulder":
            p.resetDebugVisualizerCamera(
                cameraDistance=0.65,
                cameraYaw=-40,
                cameraPitch=-18,
                cameraTargetPosition=[
                    self.robot_position[0],
                    self.robot_position[1] + 0.18,
                    self.robot_position[2] - 0.03,
                ],
            )
        elif self.camera_mode == "hand":
            p.resetDebugVisualizerCamera(
                cameraDistance=0.48,
                cameraYaw=-45,
                cameraPitch=-22,
                cameraTargetPosition=self.hand_position,
            )
