# Majd, Christian
"""Cup reach and grasp behavior from the existing simulator."""

from __future__ import annotations

import math

import pybullet as p
import pygame
from sim.contact_feedback import grasp_is_ready
from sim.joint_limits import shared_grip_step

from .arm_control import (
    clamp,
    set_elbow,
    set_shoulder,
    set_wrist,
    step_toward,
)
from .kinematics import calculate_ik_angles, is_reachable
from .settings import (
    AUTO_GRASP_CURL_PER_SECOND,
    AUTO_GRASP_WRIST_DEG,
    FINGER_CURL_PER_SECOND,
    FINGER_JOINT_TOKENS,
    FINGERTIP_CONTACT_FORCE_N,
    GRASP_CAPTURE_DISTANCE_M,
    MAX_GRASP_CURL,
    MIN_GRASP_CURL,
    MIN_GRASP_FINGERTIPS,
    SAFE_REACH_DISTANCE_M,
    TARGET_NORMAL_MASS_KG,
)


class GraspMixin:
    """Handle safe approach, finger movement, grasping, and release."""

    def update_grasp(self) -> None:
        """Advance the existing cup interaction and grip logic."""
        current_contact_links, self.fingertip_force_n = self.target_contact_feedback()
        self.hand_target_contact = bool(current_contact_links & self.hand_contact_link_indices)
        self.active_fingertips = {
            finger
            for finger, force_n in self.fingertip_force_n.items()
            if force_n >= FINGERTIP_CONTACT_FORCE_N
        }

        self.h_reach_requested = self.keys[pygame.K_h] or self.bullet_key_down("h")
        if not self.h_reach_requested:
            self.h_contact_stop = False
        elif (
            self.hand_target_contact or self.hand_target_distance <= SAFE_REACH_DISTANCE_M
        ) and not self.h_contact_stop:
            self.h_contact_stop = True
            self.notice_text = "Safe reach distance reached - H movement stopped"
            self.notice_until_ms = pygame.time.get_ticks() + 1800

        if self.auto_grasp_pending and self.auto_grasp_phase == "open":
            if self.arm.hand.curl <= 0.01:
                self.arm.hand.curl = 0.0
                self.auto_grasp_phase = "reach"
        elif (
            self.auto_grasp_pending
            and self.auto_grasp_phase == "reach"
            and (self.hand_target_contact or self.hand_target_distance <= SAFE_REACH_DISTANCE_M)
        ):
            self.auto_grasp_phase = "close"
            self.notice_text = "Cup reached - closing fingers"
            self.notice_until_ms = pygame.time.get_ticks() + 1800

        auto_reach_active = self.auto_grasp_pending and self.auto_grasp_phase == "reach"
        if (self.h_reach_requested and not self.h_contact_stop) or auto_reach_active:
            try:
                relative_x, relative_y, relative_z = self.target_relative_position
                horizontal_distance = math.hypot(relative_x, relative_y)
                target_shoulder_y, target_elbow_x = calculate_ik_angles(
                    horizontal_distance,
                    relative_z,
                )
                local_azimuth = math.atan2(relative_y, relative_x)
                target_shoulder_z = math.degrees(math.pi / 2.0 - local_azimuth)
                while target_shoulder_z > 180:
                    target_shoulder_z -= 360
                while target_shoulder_z <= -180:
                    target_shoulder_z += 360

                set_shoulder(
                    self.arm,
                    x=step_toward(
                        self.arm.shoulder.angle_x,
                        0.0,
                        "shoulder_x",
                        elapsed_s=self.loop_dt_s,
                    ),
                    y=step_toward(
                        self.arm.shoulder.angle_y,
                        target_shoulder_y,
                        "shoulder_y",
                        elapsed_s=self.loop_dt_s,
                    ),
                    z=step_toward(
                        self.arm.shoulder.angle_z,
                        target_shoulder_z,
                        "shoulder_z",
                        elapsed_s=self.loop_dt_s,
                    ),
                    mode="abs",
                )
                set_elbow(
                    self.arm,
                    x=step_toward(
                        self.arm.elbow.angle_x,
                        target_elbow_x,
                        "elbow_x",
                        elapsed_s=self.loop_dt_s,
                    ),
                    mode="abs",
                )
                if auto_reach_active:
                    set_wrist(
                        self.arm,
                        step_toward(
                            self.arm.wrist.rotation,
                            AUTO_GRASP_WRIST_DEG,
                            "wrist_rotation",
                            elapsed_s=self.loop_dt_s,
                        ),
                        mode="abs",
                    )
            except ValueError as error:
                self.notice_text = f"Target cannot be reached: {error}"
                self.notice_until_ms = pygame.time.get_ticks() + 1800

        if self.auto_grasp_pending and self.auto_grasp_phase in {"open", "close"}:
            auto_hand_step = AUTO_GRASP_CURL_PER_SECOND * self.loop_dt_s
            if self.auto_grasp_phase == "open":
                self.arm.hand.curl = max(0.0, self.arm.hand.curl - auto_hand_step)
            else:
                self.arm.hand.curl = min(MAX_GRASP_CURL, self.arm.hand.curl + auto_hand_step)
        else:
            hand_step = FINGER_CURL_PER_SECOND * self.loop_dt_s
            delta_hand = (
                hand_step if self.keys[pygame.K_f] or self.bullet_key_down("f") else 0
            ) + (-hand_step if self.keys[pygame.K_g] or self.bullet_key_down("g") else 0)
            if delta_hand:
                new_value = self.arm.hand.curl + delta_hand
                self.arm.hand.curl = clamp(
                    new_value, self.arm.hand.min_curl, self.arm.hand.max_curl
                )
            elif (
                self.args.control == "camera-imu"
                and self.live_feedback_active
                and self.live_snapshot is not None
                and self.live_snapshot.hand_curl is not None
            ):
                target_curl = self.live_snapshot.hand_curl * MAX_GRASP_CURL
                hand_step = shared_grip_step(
                    target_curl - self.arm.hand.curl, self.loop_dt_s, MAX_GRASP_CURL
                )
                self.arm.hand.curl += clamp(
                    target_curl - self.arm.hand.curl,
                    -hand_step,
                    hand_step,
                )

        # Match LIMB-HT25 manual handling: holding F inside 20 cm closes the hand
        # and immediately secures the object. Space keeps the assisted workflow.
        manual_grasp_requested = (
            self.keys[pygame.K_f] or self.bullet_key_down("f")
        ) and self.hand_target_distance <= GRASP_CAPTURE_DISTANCE_M

        if (
            (self.interact_requested or self.auto_grasp_pending or manual_grasp_requested)
            and self.grasp_transform is None
            and self.hand_link_index != -1
        ):
            if self.interact_requested and self.auto_grasp_pending:
                self.auto_grasp_pending = False
                self.auto_grasp_phase = "reach"
                self.notice_text = "Assisted grasp canceled"
                self.notice_until_ms = pygame.time.get_ticks() + 1800
                print(self.notice_text)
            elif manual_grasp_requested or (
                self.auto_grasp_pending
                and self.auto_grasp_phase == "close"
                and grasp_is_ready(
                    self.arm.hand.curl,
                    self.hand_target_distance,
                    len(self.active_fingertips),
                    min_curl=MIN_GRASP_CURL,
                    capture_distance_m=GRASP_CAPTURE_DISTANCE_M,
                    min_fingertip_contacts=MIN_GRASP_FINGERTIPS,
                )
            ):
                self.auto_grasp_pending = False
                self.auto_grasp_phase = "reach"
                self.latched_fingertip_force_n = dict(self.fingertip_force_n)
                target_position_world, target_orientation_world = p.getBasePositionAndOrientation(
                    self.target_body
                )
                if self.target_anchor_constraint is not None:
                    p.removeConstraint(self.target_anchor_constraint)
                    self.target_anchor_constraint = None
                hand_state = p.getLinkState(self.robot_body, self.hand_link_index)
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

                if self.dynamic_mode:
                    self.set_target_robot_collision(False)
                    inverse_com_position, inverse_com_orientation = p.invertTransform(
                        hand_state[0], hand_state[1]
                    )
                    target_position_in_com, target_orientation_in_com = p.multiplyTransforms(
                        inverse_com_position,
                        inverse_com_orientation,
                        target_position_world,
                        target_orientation_world,
                    )
                    self.grasp_constraint = p.createConstraint(
                        self.robot_body,
                        self.hand_link_index,
                        self.target_body,
                        -1,
                        p.JOINT_FIXED,
                        (0, 0, 0),
                        target_position_in_com,
                        (0, 0, 0),
                        target_orientation_in_com,
                        (0, 0, 0, 1),
                    )
                    contact_force_n = sum(self.latched_fingertip_force_n.values())
                    p.changeConstraint(
                        self.grasp_constraint,
                        maxForce=max(10.0, min(40.0, 10.0 * contact_force_n)),
                        erp=0.5,
                    )
                else:
                    p.changeDynamics(self.target_body, -1, mass=0.0)
                    p.setCollisionFilterGroupMask(
                        self.target_body,
                        -1,
                        collisionFilterGroup=0,
                        collisionFilterMask=0,
                    )
                self.grasp_transform = (target_position_in_hand, target_orientation_in_hand)
                self.notice_text = (
                    "Manual F grip secured"
                    if manual_grasp_requested
                    else f"Grip secured ({len(self.active_fingertips)} fingertip contacts)"
                )
                self.notice_until_ms = pygame.time.get_ticks() + 1800
                print(self.notice_text)
            elif (
                self.auto_grasp_pending
                and self.auto_grasp_phase == "close"
                and self.arm.hand.curl >= MAX_GRASP_CURL
            ):
                self.auto_grasp_pending = False
                self.auto_grasp_phase = "reach"
                self.notice_text = f"Grip failed: {len(self.active_fingertips)}/{MIN_GRASP_FINGERTIPS} fingertip contacts"
                self.notice_until_ms = pygame.time.get_ticks() + 2600
                print(self.notice_text)
            elif self.interact_requested:
                target_reachable, target_reason = is_reachable(self.target_relative_position)
                if target_reachable:
                    self.auto_grasp_pending = True
                    self.auto_grasp_phase = "open" if self.arm.hand.curl > 0.01 else "reach"
                    self.notice_text = (
                        f"Safe reach started ({self.hand_target_distance:.2f} m away)"
                    )
                else:
                    self.notice_text = target_reason.title()
                self.notice_until_ms = pygame.time.get_ticks() + 2200
                print(self.notice_text)

        elif (
            self.interact_requested or self.keys[pygame.K_g] or self.bullet_key_down("g")
        ) and self.grasp_transform is not None:
            self.grasp_transform = None
            self.latched_fingertip_force_n = {finger: 0.0 for finger in FINGER_JOINT_TOKENS}
            if self.grasp_constraint is not None:
                p.removeConstraint(self.grasp_constraint)
                self.grasp_constraint = None
                self.set_target_robot_collision(True)

            p.changeDynamics(self.target_body, -1, mass=TARGET_NORMAL_MASS_KG)
            p.setCollisionFilterGroupMask(
                self.target_body,
                -1,
                collisionFilterGroup=1,
                collisionFilterMask=1,
            )
            p.resetBaseVelocity(self.target_body, linearVelocity=[0, 0, -0.2])
            self.notice_text = "Target released"
            self.notice_until_ms = pygame.time.get_ticks() + 1800
            print(self.notice_text)
