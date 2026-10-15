# Majd, Christian
"""Start the scene, update each frame and close the simulator."""

from __future__ import annotations

import contextlib
import csv
import tkinter as tk

import pybullet as p
import pygame
from ai.live_sensor_input import LiveSensorInput

from .dashboard import DashboardMixin
from .grasp_control import GraspMixin
from .hud import HudMixin
from .input_control import InputMixin
from .physics import PhysicsMixin
from .scene import SceneMixin
from .settings import (
    CAMERA_MODES,
    FINGER_JOINT_TOKENS,
    OVERVIEW_CAMERA_PITCH_DEG,
    OVERVIEW_CAMERA_TARGET,
    OVERVIEW_CAMERA_YAW_DEG,
    SIMULATION_FREQUENCY_HZ,
)


class InteractiveSimulator(
    SceneMixin, InputMixin, GraspMixin, PhysicsMixin, DashboardMixin, HudMixin
):
    """Own the task scene and call each part once per frame."""

    def __init__(self, args):
        self.args = args
        self.dynamic_mode = args.mode == "dynamic"
        self.live_sensor_input = None
        self.sensor_window = None
        self.live_signal_graphs = None
        self.telemetry_file = None
        self.torque_csv_file = None

    def run(self) -> None:
        try:
            self.initialize_scene()
            self.initialize_arm()
            self.initialize_dashboard()
            self.initialize_hud()
            self.initialize_task()
            self.start_live_input()
            while self.running and p.isConnected():
                if not self.prepare_frame():
                    continue
                self.process_events()
                self.update_grasp()
                self.update_arm()
                if not self.advance_physics():
                    continue
                self.update_camera()
                self.draw_hud()
                self.update_dashboard()
                self.clock.tick(SIMULATION_FREQUENCY_HZ)
        finally:
            self.close()

    def initialize_task(self) -> None:
        """Initialize task state and optional telemetry files."""
        self.running = True
        self.hud_visible = True
        self.grasp_transform = None
        self.grasp_constraint = None
        self.auto_grasp_pending = False
        self.auto_grasp_phase = "reach"
        self.h_contact_stop = False
        self.latched_fingertip_force_n = {finger: 0.0 for finger in FINGER_JOINT_TOKENS}
        self.reach_line_id = -1
        self.target_text_id = -1

        self.camera_mode = CAMERA_MODES[0]
        self.default_cam_yaw = OVERVIEW_CAMERA_YAW_DEG
        self.default_cam_pitch = OVERVIEW_CAMERA_PITCH_DEG
        self.default_cam_target = list(OVERVIEW_CAMERA_TARGET)
        self.notice_text = "Ready - click this window to control the arm"
        self.notice_until_ms = 0

        self.default_position = (0.0, 0.0, 0.0)
        self.default_orientation = (0.0, 0.0, 0.0)
        self.hand_position = self.default_position
        self.imu_euler_rad = self.default_orientation
        self.measured_angles_deg = {name: 0.0 for name in self.joint_indices}
        self.measured_motor_torque = {name: 0.0 for name in self.joint_indices}
        self.physics_snapshot = None
        self.frame_number = 0
        self.telemetry_file = None
        if self.args.telemetry_out is not None:
            self.args.telemetry_out.parent.mkdir(parents=True, exist_ok=True)
            self.telemetry_file = self.args.telemetry_out.open("w", encoding="utf-8")
            print(f"Physics telemetry: {self.args.telemetry_out}")

        self.torque_csv_file = None
        self.torque_csv_writer = None
        if self.args.torque_out is not None:
            self.args.torque_out.parent.mkdir(parents=True, exist_ok=True)
            self.torque_csv_file = self.args.torque_out.open("w", newline="", encoding="utf-8")
            self.torque_csv_writer = csv.DictWriter(
                self.torque_csv_file,
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
            self.torque_csv_writer.writeheader()
            print(f"Torque CSV: {self.args.torque_out}")

    def start_live_input(self) -> None:
        """Start camera and BLE/serial input when live control is selected."""
        self.live_sensor_input = None
        self.live_snapshot = None
        self.live_feedback_active = self.args.control != "camera-imu"
        if self.args.control == "camera-imu":
            self.live_sensor_input = LiveSensorInput(
                port=self.args.port,
                baud=self.args.baud,
                side=self.args.side,
                camera_weight=self.args.camera_weight,
                depth=self.args.depth,
                transport=self.args.transport,
                device=self.args.device,
                address=self.args.address,
                grip_source=self.args.grip_source,
                shoulder_twist_axis=self.args.shoulder_twist_axis,
                wrist_twist_axis=self.args.wrist_twist_axis,
            )
            try:
                self.live_sensor_input.start()
                self.notice_text = (
                    "Match the neutral tabletop pose, then press L and hold still for 0.5 s"
                )
                self.notice_until_ms = pygame.time.get_ticks() + 6000
                sensor_connection = (
                    f"BLE {self.args.address or self.args.device}"
                    if self.args.transport == "ble"
                    else f"{self.args.port} at {self.args.baud} baud"
                )
                print(f"Live control: {sensor_connection}, {self.args.side} arm")
            except Exception as error:
                # run() owns cleanup, including partial camera and BLE startup.
                raise SystemExit(
                    f"Could not start live camera + IMU + EMG control: {error}"
                ) from error

    def close(self) -> None:
        print("Simulation finished.")
        if self.live_sensor_input is not None:
            self.live_sensor_input.close()
        if self.sensor_window is not None:
            with contextlib.suppress(tk.TclError):
                self.sensor_window.destroy()
        pygame.quit()
        if self.telemetry_file is not None:
            self.telemetry_file.close()
        if self.torque_csv_file is not None:
            self.torque_csv_file.close()
        if p.isConnected():
            p.disconnect()

    def calibrate_live_emg(self) -> None:
        if self.live_sensor_input is not None:
            self.live_sensor_input.calibrate_emg()

    def calibrate_live_emg_strength(self) -> None:
        if self.live_sensor_input is not None:
            self.live_sensor_input.calibrate_emg_strength()
