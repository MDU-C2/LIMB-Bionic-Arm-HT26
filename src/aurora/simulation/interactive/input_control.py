# Majd, Christian
"""Keyboard events, live calibration, and arm control."""

from __future__ import annotations

import math

import pybullet as p
import pygame

from aurora.sensors.fusion import anatomical_reference_angles, interactive_control_targets
from aurora.simulation.interactive.arm_control import (
    get_angles_deg,
    set_elbow,
    set_live_alignment_pose,
    set_ready_pose,
    set_shoulder,
    set_wrist,
    step_toward,
    sync_to_pybullet,
)
from aurora.simulation.interactive.settings import (
    CAMERA_MODES,
    FINGER_JOINT_TOKENS,
    SIMULATION_FREQUENCY_HZ,
    TARGET_NORMAL_MASS_KG,
)


class InputMixin:
    """Read keyboard and live sensor commands."""

    def prepare_frame(self) -> bool:
        """Read current positions and the latest camera/BLE control state."""
        # Keep motor speed tied to elapsed time when rendering slows down.
        previous_loop_ms = self.clock.get_time()
        elapsed_s = (
            previous_loop_ms / 1000.0 if previous_loop_ms > 0 else 1.0 / SIMULATION_FREQUENCY_HZ
        )
        self.steps_due = self.step_clock.advance(elapsed_s)
        self.wall_time_s = self.step_clock.wall_time_s
        self.loop_dt_s = self.step_clock.period_s

        try:
            self.target_position, _ = p.getBasePositionAndOrientation(self.target_body)
            self.robot_position, robot_orientation = p.getBasePositionAndOrientation(
                self.robot_body
            )
        except p.error:
            self.running = False
            return False

        # IK and reachability use the mounted arm's frame, including its yaw.
        inverse_position, inverse_orientation = p.invertTransform(
            self.robot_position,
            robot_orientation,
        )
        self.target_relative_position, _ = p.multiplyTransforms(
            inverse_position,
            inverse_orientation,
            self.target_position,
            (0, 0, 0, 1),
        )
        self.hand_target_distance = math.dist(self.hand_position, self.target_position)

        if self.live_sensor_input is not None:
            try:
                self.live_snapshot = self.live_sensor_input.poll()
            except Exception as error:
                self.notice_text = f"Live control stopped: {error}"
                print(self.notice_text)
                self.running = False
                return False
            if self.live_snapshot.stop_requested:
                self.running = False
                return False
            if not self.live_snapshot.calibration_valid:
                self.live_feedback_active = False
            guide = getattr(self, "live_setup", None)
            if guide is not None:
                guide.update(self.live_snapshot)
                if guide.stage != "running":
                    self.live_feedback_active = False
        return True

    def process_events(self) -> None:
        """Handle keyboard actions and live alignment/calibration."""
        reset_requested = getattr(self, "live_reset_requested", False)
        self.live_reset_requested = False
        self.camera_changed = False
        calibrate_requested = False
        start_live_requested = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_t:
                    self.hud_visible = not self.hud_visible
                    self.notice_text = (
                        "Target guide shown" if self.hud_visible else "Target guide hidden"
                    )
                    self.notice_until_ms = pygame.time.get_ticks() + 1800

                if event.key == pygame.K_SPACE:
                    self.interact_requested = True

                if event.key == pygame.K_r:
                    reset_requested = True

                if event.key == pygame.K_k and self.live_sensor_input is not None:
                    calibrate_requested = True

                if event.key == pygame.K_l and self.live_sensor_input is not None:
                    start_live_requested = True

                if event.key in (pygame.K_c, pygame.K_TAB):
                    camera_index = (CAMERA_MODES.index(self.camera_mode) + 1) % len(CAMERA_MODES)
                    self.camera_mode = CAMERA_MODES[camera_index]
                    self.camera_changed = True
                    self.notice_text = f"Camera: {self.camera_mode}"
                    self.notice_until_ms = pygame.time.get_ticks() + 1800

                if event.key in (pygame.K_1, pygame.K_2, pygame.K_3):
                    self.camera_mode = CAMERA_MODES[event.key - pygame.K_1]
                    self.camera_changed = True
                    self.notice_text = f"Camera: {self.camera_mode}"
                    self.notice_until_ms = pygame.time.get_ticks() + 1800

                if event.key == pygame.K_p:
                    self.preview_visible = not self.preview_visible
                    p.configureDebugVisualizer(p.COV_ENABLE_GUI, int(self.preview_visible))
                    for preview in (
                        p.COV_ENABLE_RGB_BUFFER_PREVIEW,
                        p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                        p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
                    ):
                        p.configureDebugVisualizer(preview, int(self.preview_visible))
                    self.notice_text = (
                        "Camera previews shown"
                        if self.preview_visible
                        else "Camera previews hidden"
                    )
                    self.notice_until_ms = pygame.time.get_ticks() + 1800

                if event.key == pygame.K_i:
                    if self.sensor_window.state() == "withdrawn":
                        self.sensor_window.deiconify()
                        self.sensor_window.lift()
                        self.notice_text = "Sensor dashboard shown"
                    else:
                        self.sensor_window.withdraw()
                        self.notice_text = "Sensor dashboard hidden"
                    self.notice_until_ms = pygame.time.get_ticks() + 1800

        # The scene can receive focus instead of the separate Pygame controller.
        # Accept the essential task keys from either window.
        self.bullet_events = p.getKeyboardEvents()

        if self.bullet_events.get(ord(" "), 0) & p.KEY_WAS_TRIGGERED:
            self.interact_requested = True
        if self.bullet_key_triggered("r"):
            reset_requested = True
        if self.bullet_key_triggered("k") and self.live_sensor_input is not None:
            calibrate_requested = True
        if self.bullet_key_triggered("l") and self.live_sensor_input is not None:
            start_live_requested = True

        if (start_live_requested or calibrate_requested) and (
            getattr(self, "live_setup", None) is not None or getattr(self.args, "replay", None)
        ):
            self.advance_live_setup()
            start_live_requested = calibrate_requested = False
        if start_live_requested:
            tracking_ready = self.live_snapshot is not None and (
                self.live_snapshot.camera_ready or self.live_snapshot.imu_ready
            )
            if tracking_ready:
                self.live_sensor_input.calibrate(
                    anatomical_reference_angles(get_angles_deg(self.arm))
                )
                # Do not apply the snapshot captured before calibration.  Keeping
                # the reference pose for one more frame prevents a visible jump.
                self.live_snapshot = None
                self.live_feedback_active = True
                self.notice_text = "Live feedback started from the matched tabletop pose"
                self.notice_until_ms = pygame.time.get_ticks() + 3000
            else:
                self.notice_text = (
                    "No camera or IMU tracking yet - keep the arm matched and try L again"
                )
                self.notice_until_ms = pygame.time.get_ticks() + 3000

        if calibrate_requested:
            self.live_sensor_input.calibrate(anatomical_reference_angles(get_angles_deg(self.arm)))
            self.live_snapshot = None
            self.notice_text = "Live sensors recalibrated at the current pose"
            self.notice_until_ms = pygame.time.get_ticks() + 2600

        self.keys = pygame.key.get_pressed()
        if self.keys[pygame.K_ESCAPE]:
            self.running = False

        if reset_requested:
            self.interact_requested = False
            self.auto_grasp_pending = False
            self.auto_grasp_phase = "reach"
            self.h_contact_stop = False
            self.grasp_transform = None
            self.latched_fingertip_force_n = {finger: 0.0 for finger in FINGER_JOINT_TOKENS}
            if self.grasp_constraint is not None:
                p.removeConstraint(self.grasp_constraint)
                self.grasp_constraint = None
            self.set_target_robot_collision(True)
            self.target_collision_pending = False
            if self.target_anchor_constraint is not None:
                p.removeConstraint(self.target_anchor_constraint)
                self.target_anchor_constraint = None

            p.changeDynamics(self.target_body, -1, mass=TARGET_NORMAL_MASS_KG)
            p.setCollisionFilterGroupMask(
                self.target_body,
                -1,
                collisionFilterGroup=1,
                collisionFilterMask=1,
            )
            p.resetBasePositionAndOrientation(
                self.target_body,
                self.target_start_position,
                self.target_start_orientation,
            )
            p.resetBaseVelocity(self.target_body, [0, 0, 0], [0, 0, 0])
            self.target_anchor_constraint = self.anchor_target_to_world()

            if self.live_sensor_input is not None:
                self.live_feedback_active = False
                if getattr(self.args, "replay", None):
                    self.pause_live_control()
                set_live_alignment_pose(self.arm)
                if getattr(self, "live_setup", None) is not None:
                    self.live_setup.pause()
                elif self.args.transport == "serial":
                    self.setup_text.set(
                        "USB control paused. Match the tabletop pose, then align and start."
                    )
            else:
                set_ready_pose(self.arm)
            self.manual_motion.reset()
            if p.isConnected():
                sync_to_pybullet(
                    self.arm, self.robot_body, self.joint_indices, client=p, use_motors=False
                )
                p.performCollisionDetection()
            self.notice_text = (
                "Paused at tabletop pose - follow Live control setup in the sensor window"
                if self.live_sensor_input is not None
                else "Arm and target reset"
            )
            self.notice_until_ms = pygame.time.get_ticks() + 2200

    def bullet_key_down(self, letter: str) -> bool:
        return any(
            self.bullet_events.get(ord(key), 0) & p.KEY_IS_DOWN
            for key in (letter.lower(), letter.upper())
        )

    def bullet_key_triggered(self, letter: str) -> bool:
        return any(
            self.bullet_events.get(ord(key), 0) & p.KEY_WAS_TRIGGERED
            for key in (letter.lower(), letter.upper())
        )

    def update_arm(self) -> None:
        """Apply live or manual arm movement at the existing speed limits."""
        speed_mult = 1.0
        if self.keys[pygame.K_LCTRL] or self.keys[pygame.K_RCTRL]:
            speed_mult = 0.25

        if self.h_reach_requested or self.auto_grasp_pending:
            self.manual_motion.reset()
        elif self.args.control == "camera-imu" and self.live_feedback_active:

            def live_step(current, target, joint, *, elapsed_s):
                if not self.dynamic_mode:
                    return target
                return step_toward(current, target, joint, elapsed_s=elapsed_s)

            targets = (
                (
                    self.live_snapshot.robot_targets
                    or interactive_control_targets(self.live_snapshot.angles)
                )
                if self.live_snapshot is not None
                else None
            )
            if targets:
                set_shoulder(
                    self.arm,
                    x=live_step(
                        self.arm.shoulder.angle_x,
                        targets.get("shoulder_x", self.arm.shoulder.angle_x),
                        "shoulder_x",
                        elapsed_s=self.loop_dt_s,
                    ),
                    y=live_step(
                        self.arm.shoulder.angle_y,
                        targets.get("shoulder_y", self.arm.shoulder.angle_y),
                        "shoulder_y",
                        elapsed_s=self.loop_dt_s,
                    ),
                    z=live_step(
                        self.arm.shoulder.angle_z,
                        targets.get("shoulder_z", self.arm.shoulder.angle_z),
                        "shoulder_z",
                        elapsed_s=self.loop_dt_s,
                    ),
                    mode="abs",
                )
                set_elbow(
                    self.arm,
                    x=live_step(
                        self.arm.elbow.angle_x,
                        targets.get("elbow_x", self.arm.elbow.angle_x),
                        "elbow_x",
                        elapsed_s=self.loop_dt_s,
                    ),
                    mode="abs",
                )
            # The old scene keeps manual controls for joints without a live target.
            # Keep manual rotation available when neither calibrated IMUs nor camera supply it.
            if not targets or "shoulder_x" not in targets:
                twist_keys = {
                    pygame.K_e: self.keys[pygame.K_e] or self.bullet_key_down("e"),
                    pygame.K_q: self.keys[pygame.K_q] or self.bullet_key_down("q"),
                }
                delta = self.manual_motion.step(
                    twist_keys,
                    pygame.K_e,
                    pygame.K_q,
                    "shoulder_x",
                    self.arm.shoulder.angle_x,
                    speed_mult,
                    self.loop_dt_s,
                )
                set_shoulder(self.arm, x=delta, mode="rel")
            else:
                self.manual_motion.velocity_deg_s["shoulder_x"] = 0.0
            wrist_keys = {
                pygame.K_RIGHT: self.keys[pygame.K_RIGHT]
                or bool(self.bullet_events.get(p.B3G_RIGHT_ARROW, 0) & p.KEY_IS_DOWN),
                pygame.K_LEFT: self.keys[pygame.K_LEFT]
                or bool(self.bullet_events.get(p.B3G_LEFT_ARROW, 0) & p.KEY_IS_DOWN),
            }
            if targets and "wrist_rotation" in targets:
                set_wrist(
                    self.arm,
                    live_step(
                        self.arm.wrist.rotation,
                        targets["wrist_rotation"],
                        "wrist_rotation",
                        elapsed_s=self.loop_dt_s,
                    ),
                )
                self.manual_motion.velocity_deg_s["wrist_rotation"] = 0.0
            else:
                delta = self.manual_motion.step(
                    wrist_keys,
                    pygame.K_RIGHT,
                    pygame.K_LEFT,
                    "wrist_rotation",
                    self.arm.wrist.rotation,
                    speed_mult,
                    self.loop_dt_s,
                )
                set_wrist(self.arm, delta, mode="rel")
        elif self.args.control == "camera-imu":
            self.manual_motion.reset()
        else:
            delta_should_y = self.manual_motion.step(
                self.keys,
                pygame.K_s,
                pygame.K_w,
                "shoulder_y",
                self.arm.shoulder.angle_y,
                speed_mult,
                self.loop_dt_s,
            )
            delta_should_z = self.manual_motion.step(
                self.keys,
                pygame.K_d,
                pygame.K_a,
                "shoulder_z",
                self.arm.shoulder.angle_z,
                speed_mult,
                self.loop_dt_s,
            )
            delta_should_x = self.manual_motion.step(
                self.keys,
                pygame.K_e,
                pygame.K_q,
                "shoulder_x",
                self.arm.shoulder.angle_x,
                speed_mult,
                self.loop_dt_s,
            )
            set_shoulder(
                self.arm,
                x=delta_should_x,
                y=delta_should_y,
                z=delta_should_z,
                mode="rel",
            )

            delta_elbow_x = self.manual_motion.step(
                self.keys,
                pygame.K_UP,
                pygame.K_DOWN,
                "elbow_x",
                self.arm.elbow.angle_x,
                speed_mult,
                self.loop_dt_s,
            )
            delta_wrist = self.manual_motion.step(
                self.keys,
                pygame.K_RIGHT,
                pygame.K_LEFT,
                "wrist_rotation",
                self.arm.wrist.rotation,
                speed_mult,
                self.loop_dt_s,
            )
            set_elbow(self.arm, x=delta_elbow_x, mode="rel")
            set_wrist(self.arm, delta_wrist, mode="rel")
