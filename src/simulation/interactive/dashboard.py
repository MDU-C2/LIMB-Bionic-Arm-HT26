# Majd, Christian
"""Tkinter joint, torque, and sensor monitor."""

from __future__ import annotations

import math
import tkinter as tk

from sim.controller_params import (
    ARM_MOTOR_FORCE_NM,
)

from .arm_control import (
    rad_to_deg,
)
from .live_signal_graphs import LiveSignalGraphs
from .settings import (
    FINGER_JOINT_TOKENS,
    FINGERTIP_CONTACT_FORCE_N,
    SIMULATION_FREQUENCY_HZ,
)
from .torque_graph import TorqueGraph


class DashboardMixin:
    """Build and refresh the simulator sensor dashboard."""

    def initialize_dashboard(self) -> None:
        """Create joint, live sensor, contact, and torque panels."""
        print("Initializing sensor window (Tkinter)...")
        self.sensor_window = tk.Tk()
        self.sensor_window.title(
            "LIMB Live Sensor Monitor"
            if self.args.control == "camera-imu"
            else "LIMB Sensor Dashboard"
        )
        self.sensor_window.geometry("500x805+35+35")
        self.sensor_window.protocol("WM_DELETE_WINDOW", self.sensor_window.withdraw)

        if self.args.control == "camera-imu" and self.args.transport == "ble":
            self.live_signal_graphs = LiveSignalGraphs(
                self.sensor_window, self.calibrate_live_emg, self.calibrate_live_emg_strength
            )
            tk.Button(
                self.sensor_window,
                text="Show live BLE graphs",
                command=self.live_signal_graphs.show,
            ).pack(anchor="w", padx=10, pady=4)

        self.sensor_vars = {}
        tk_font = ("Consolas", 11)
        tk_font_bold = ("Consolas", 12, "bold")

        joint_frame = tk.Frame(self.sensor_window, padx=10, pady=10)
        joint_frame.pack(fill="x")

        tk.Label(joint_frame, text="--- JOINT POSE (Simulated) ---", font=tk_font_bold).pack(
            anchor="w"
        )

        self.joint_names_map = {
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
        row.pack(anchor="w")

        for name, key in self.joint_names_map.items():
            self.sensor_vars[f"{key}_angle"] = tk.StringVar(value="--.- deg")
            self.sensor_vars[f"{key}_torque"] = tk.StringVar(value="N/A")

            row = tk.Frame(joint_frame)
            tk.Label(row, text=f"{name}:", width=13, font=tk_font_bold).pack(side=tk.LEFT)
            tk.Label(
                row,
                textvariable=self.sensor_vars[f"{key}_angle"],
                width=12,
                anchor="w",
                font=tk_font,
            ).pack(side=tk.LEFT)
            tk.Label(
                row,
                textvariable=self.sensor_vars[f"{key}_torque"],
                width=10,
                anchor="w",
                font=tk_font,
            ).pack(side=tk.LEFT)
            row.pack(anchor="w")

        self.torque_graph = TorqueGraph(
            self.sensor_window,
            tk,
            {key: name for name, key in self.joint_names_map.items()},
            torque_limit_nm=max(ARM_MOTOR_FORCE_NM.values()),
            sample_rate_hz=SIMULATION_FREQUENCY_HZ,
        )
        self.torque_graph.redraw(self.dynamic_mode)

        imu_frame = tk.Frame(self.sensor_window, padx=10, pady=10)
        imu_frame.pack(fill="x")

        tk.Label(imu_frame, text="--- IMU SENSOR (Hand) ---", font=tk_font_bold).pack(anchor="w")
        self.sensor_vars["imu_roll"] = tk.StringVar(value="Roll:  --.-")
        self.sensor_vars["imu_pitch"] = tk.StringVar(value="Pitch: --.-")
        self.sensor_vars["imu_yaw"] = tk.StringVar(value="Yaw:   --.-")

        tk.Label(imu_frame, textvariable=self.sensor_vars["imu_roll"], font=tk_font).pack(
            anchor="w"
        )
        tk.Label(imu_frame, textvariable=self.sensor_vars["imu_pitch"], font=tk_font).pack(
            anchor="w"
        )
        tk.Label(imu_frame, textvariable=self.sensor_vars["imu_yaw"], font=tk_font).pack(anchor="w")

        bio_frame = tk.Frame(self.sensor_window, padx=10, pady=10)
        bio_frame.pack(fill="x")

        tk.Label(bio_frame, text="--- HAND INPUT AND CONTACT ---", font=tk_font_bold).pack(
            anchor="w"
        )
        self.sensor_vars["emg_shoulder"] = tk.StringVar(value="EMG: not available in keyboard mode")
        self.sensor_vars["pressure_hand"] = tk.StringVar(value="Fingertip contact: 0/5")

        tk.Label(bio_frame, textvariable=self.sensor_vars["emg_shoulder"], font=tk_font).pack(
            anchor="w"
        )
        tk.Label(bio_frame, textvariable=self.sensor_vars["pressure_hand"], font=tk_font).pack(
            anchor="w"
        )
        for finger in FINGER_JOINT_TOKENS:
            self.sensor_vars[f"pressure_{finger}"] = tk.StringVar(
                value=f"{finger.title():6}: 0.00 N  [----------]"
            )
            tk.Label(
                bio_frame,
                textvariable=self.sensor_vars[f"pressure_{finger}"],
                font=tk_font,
            ).pack(anchor="w")

        physics_frame = tk.Frame(self.sensor_window, padx=10, pady=6)
        physics_frame.pack(fill="x")
        tk.Label(physics_frame, text="--- PHYSICS (URDF ESTIMATE) ---", font=tk_font_bold).pack(
            anchor="w"
        )
        self.sensor_vars["gravity_effort"] = tk.StringVar(value="Gravity hold: --")
        self.sensor_vars["grip_speed"] = tk.StringVar(value="Grip speed: --")
        tk.Label(physics_frame, textvariable=self.sensor_vars["gravity_effort"], font=tk_font).pack(
            anchor="w"
        )
        tk.Label(physics_frame, textvariable=self.sensor_vars["grip_speed"], font=tk_font).pack(
            anchor="w"
        )

        live_frame = tk.Frame(self.sensor_window, padx=10, pady=6)
        live_frame.pack(fill="x")
        tk.Label(live_frame, text="--- LIVE CONTROL SOURCES ---", font=tk_font_bold).pack(
            anchor="w"
        )
        self.sensor_vars["live_status"] = tk.StringVar(
            value="Keyboard control"
            if self.args.control == "keyboard"
            else "Starting camera, IMUs, and EMG"
        )
        self.sensor_vars["live_shoulder"] = tk.StringVar(value="Shoulder IMU: --")
        self.sensor_vars["live_wrist"] = tk.StringVar(value="Wrist IMU: --")
        self.sensor_vars["live_rotation"] = tk.StringVar(
            value="Upper-arm twist: waiting for camera"
        )
        tk.Label(live_frame, textvariable=self.sensor_vars["live_status"], font=tk_font).pack(
            anchor="w"
        )
        tk.Label(live_frame, textvariable=self.sensor_vars["live_shoulder"], font=tk_font).pack(
            anchor="w"
        )
        tk.Label(live_frame, textvariable=self.sensor_vars["live_wrist"], font=tk_font).pack(
            anchor="w"
        )
        tk.Label(live_frame, textvariable=self.sensor_vars["live_rotation"], font=tk_font).pack(
            anchor="w"
        )

    def update_dashboard(self) -> None:
        """Refresh the monitor from this frame's measurements."""
        try:
            for key in self.joint_names_map.values():
                self.sensor_vars[f"{key}_angle"].set(f"{self.hardware_angles_deg[key]:.1f} deg")
                self.sensor_vars[f"{key}_torque"].set(
                    f"{self.measured_motor_torque[key]:.2f} Nm" if self.dynamic_mode else "N/A"
                )

            if self.physics_snapshot is not None:
                shoulder_column = self.dynamics.names.index("jLeftShoulder_roty")
                gravity_hold = self.physics_snapshot["gravity_torque_Nm"][shoulder_column]
                velocity = self.physics_snapshot["end_effector"]["velocity_m_s"]
                grip_speed = math.sqrt(sum(component**2 for component in velocity))
                self.sensor_vars["gravity_effort"].set(
                    f"Shoulder gravity hold: {gravity_hold:.2f} Nm"
                )
                self.sensor_vars["grip_speed"].set(f"Grip speed: {grip_speed:.3f} m/s")

            self.sensor_vars["imu_roll"].set(f"Roll:  {rad_to_deg(self.imu_euler_rad[0]):.1f}")
            self.sensor_vars["imu_pitch"].set(f"Pitch: {rad_to_deg(self.imu_euler_rad[1]):.1f}")
            self.sensor_vars["imu_yaw"].set(f"Yaw:   {rad_to_deg(self.imu_euler_rad[2]):.1f}")

            if self.live_snapshot is not None:
                live_phase = (
                    "Feedback ACTIVE"
                    if self.live_feedback_active
                    else "MATCH TABLETOP POSE - press L when matched"
                )
                self.sensor_vars["live_status"].set(f"{live_phase} | {self.live_snapshot.status}")
                self.sensor_vars["live_shoulder"].set(
                    format_live_imu("shoulder", self.live_snapshot.sensors)
                )
                self.sensor_vars["live_wrist"].set(
                    format_live_imu("wrist", self.live_snapshot.sensors)
                )
                rotation = (self.live_snapshot.angles or {}).get("shoulder_rotation_proxy")
                self.sensor_vars["live_rotation"].set(
                    f"Upper-arm twist: {self.live_snapshot.rotation_source} {rotation:+.1f} deg"
                    if rotation is not None
                    else "Twist: hold still 0.5 s / bend elbow in view / Q-E manual"
                )
                if self.live_snapshot.emg_raw is None:
                    self.sensor_vars["emg_shoulder"].set("EMG: waiting for a valid ADC sample")
                elif self.live_snapshot.emg_activation is None:
                    self.sensor_vars["emg_shoulder"].set(
                        f"EMG: {self.live_snapshot.emg_raw} ADC · calibrating rest"
                    )
                else:
                    self.sensor_vars["emg_shoulder"].set(
                        f"EMG: {self.live_snapshot.emg_raw} ADC · "
                        f"activation {self.live_snapshot.emg_activation * 100:.1f}% · "
                        f"grip source {self.live_snapshot.grip_source}"
                    )

            displayed_fingertip_force_n = {
                finger: max(
                    self.fingertip_force_n[finger],
                    self.latched_fingertip_force_n[finger]
                    if self.grasp_transform is not None
                    else 0.0,
                )
                for finger in FINGER_JOINT_TOKENS
            }
            displayed_contacts = sum(
                force_n >= FINGERTIP_CONTACT_FORCE_N
                for force_n in displayed_fingertip_force_n.values()
            )
            total_contact_force_n = sum(displayed_fingertip_force_n.values())
            self.sensor_vars["pressure_hand"].set(
                f"Virtual fingertip contact: {displayed_contacts}/5  "
                f"total {total_contact_force_n:.2f} N"
            )
            for finger, force_n in displayed_fingertip_force_n.items():
                filled = round(min(force_n / 2.0, 1.0) * 10)
                bar = "#" * filled + "-" * (10 - filled)
                self.sensor_vars[f"pressure_{finger}"].set(
                    f"{finger.title():6}: {force_n:5.2f} N  [{bar}]"
                )

            if self.live_signal_graphs is not None and self.live_sensor_input is not None:
                self.live_signal_graphs.update(self.live_sensor_input.reader)
            self.sensor_window.update()

        except tk.TclError:
            print("Sensor window closed.")
            self.running = False


def format_live_imu(role: str, sensors: dict[str, dict[str, object]]) -> str:
    """Return one compact live-IMU line for the separate sensor window."""
    sensor = sensors.get(role, {})
    if sensor.get("connected") is not True:
        return f"{role.title()} IMU: disconnected"
    accel = sensor.get("accel_g")
    gyro = sensor.get("gyro_dps")
    if not isinstance(accel, dict) or not isinstance(gyro, dict):
        return f"{role.title()} IMU: incomplete packet"
    try:
        return (
            f"{role.title()} IMU: A {float(accel['x']):+.2f}/"
            f"{float(accel['y']):+.2f}/{float(accel['z']):+.2f} g  "
            f"G {float(gyro['x']):+.1f}/{float(gyro['y']):+.1f}/"
            f"{float(gyro['z']):+.1f} deg/s"
        )
    except (KeyError, TypeError, ValueError):
        return f"{role.title()} IMU: incomplete packet"
