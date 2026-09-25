"""Dedicated live window for the shoulder and wrist IMUs."""

from __future__ import annotations

from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import ttk


SRC_DIR = Path(__file__).resolve().parents[1]
for module_dir in (SRC_DIR / "recording", SRC_DIR / "simulation" / "ai"):
    if str(module_dir) not in sys.path:
        sys.path.insert(0, str(module_dir))

from imu_protocol import ROLE_LABELS, extract_imus
from sensor_fusion import DualImuArmEstimator


class ImuMonitorWindow:
    """Render dual-IMU values outside the main control-center window."""

    def __init__(self, parent: tk.Misc, reconnect_command, disconnect_command) -> None:
        self.window = tk.Toplevel(parent)
        self.window.title("AURORA | Shoulder and wrist IMUs")
        self.window.geometry("900x520")
        self.window.minsize(700, 460)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.closed = False
        self.estimator = DualImuArmEstimator()
        self.last_sample_time: float | None = None
        self.connection = tk.StringVar(value="Waiting for serial connection")
        self.device = tk.StringVar(value="ESP32: no data")
        self.joints = tk.StringVar(value="Calibrating after both IMUs report data")
        self.sensor_values: dict[str, dict[str, tk.StringVar]] = {}

        root = ttk.Frame(self.window, padding=18)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)

        ttk.Label(root, text="Live IMU monitor", font=("Segoe UI", 17, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            root,
            text="One ESP32 serial stream · no data is recorded in this window",
            foreground="#667085",
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(2, 12))
        ttk.Label(root, textvariable=self.connection, font=("Segoe UI", 10, "bold")).grid(
            row=2, column=0, sticky="w"
        )
        actions = ttk.Frame(root)
        actions.grid(row=2, column=1, sticky="e")
        ttk.Button(actions, text="Reconnect", command=reconnect_command).pack(
            side="left", padx=(0, 8)
        )
        ttk.Button(actions, text="Disconnect", command=disconnect_command).pack(side="left")
        ttk.Label(root, textvariable=self.device, foreground="#667085").grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(5, 14)
        )

        for column, role in enumerate(("shoulder", "wrist")):
            frame = ttk.LabelFrame(root, text=ROLE_LABELS[role], padding=14)
            frame.grid(
                row=4, column=column, sticky="nsew",
                padx=((0, 7) if column == 0 else (7, 0)),
            )
            frame.columnconfigure(0, weight=1)
            values = {
                "state": tk.StringVar(value="Not detected"),
                "meta": tk.StringVar(value="Address: -    Temperature: -"),
                "accel": tk.StringVar(value="X  -        Y  -        Z  -"),
                "gyro": tk.StringVar(value="X  -        Y  -        Z  -"),
            }
            self.sensor_values[role] = values
            ttk.Label(frame, textvariable=values["state"], font=("Segoe UI", 11, "bold")).grid(
                row=0, column=0, sticky="w"
            )
            ttk.Label(frame, textvariable=values["meta"], foreground="#667085").grid(
                row=1, column=0, sticky="w", pady=(2, 14)
            )
            ttk.Label(frame, text="Acceleration (g)", font=("Segoe UI", 9, "bold")).grid(
                row=2, column=0, sticky="w"
            )
            ttk.Label(frame, textvariable=values["accel"], font=("Cascadia Mono", 10)).grid(
                row=3, column=0, sticky="w", pady=(3, 13)
            )
            ttk.Label(frame, text="Angular velocity (°/s)", font=("Segoe UI", 9, "bold")).grid(
                row=4, column=0, sticky="w"
            )
            ttk.Label(frame, textvariable=values["gyro"], font=("Cascadia Mono", 10)).grid(
                row=5, column=0, sticky="w", pady=(3, 0)
            )

        fused = ttk.LabelFrame(root, text="Relative arm estimate", padding=14)
        fused.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        fused.columnconfigure(0, weight=1)
        ttk.Label(fused, textvariable=self.joints, font=("Cascadia Mono", 10)).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Button(fused, text="Calibrate current pose", command=self.calibrate).grid(
            row=0, column=1, padx=(12, 0)
        )

    @property
    def exists(self) -> bool:
        return not self.closed and bool(self.window.winfo_exists())

    def focus(self) -> None:
        if self.exists:
            self.window.deiconify()
            self.window.lift()
            self.window.focus_force()

    def set_connection(self, text: str) -> None:
        if self.exists:
            self.connection.set(text)

    @staticmethod
    def _axes(value: object, decimals: int) -> str:
        if not isinstance(value, dict):
            return "X  -        Y  -        Z  -"
        try:
            return "   ".join(
                f"{axis.upper()} {float(value[axis]): .{decimals}f}"
                for axis in ("x", "y", "z")
            )
        except (KeyError, TypeError, ValueError):
            return "X  -        Y  -        Z  -"

    def show_packet(self, packet: dict[str, object]) -> None:
        if not self.exists:
            return
        device = str(packet.get("device", "ESP32"))
        uptime = packet.get("uptime_ms")
        uptime_text = (
            f" · uptime {float(uptime) / 1000:.1f} s"
            if isinstance(uptime, (int, float)) and not isinstance(uptime, bool)
            else ""
        )
        self.device.set(f"ESP32: {device}{uptime_text}")
        sensors = extract_imus(packet)
        connected = 0
        for role in ("shoulder", "wrist"):
            sensor = sensors.get(role, {})
            values = self.sensor_values[role]
            if sensor.get("connected") is not True:
                values["state"].set(f"Disconnected · {sensor.get('error', 'no data')}")
                values["meta"].set("Address: -    Temperature: -")
                values["accel"].set(self._axes(None, 4))
                values["gyro"].set(self._axes(None, 2))
                continue
            connected += 1
            values["state"].set("Connected")
            temperature = sensor.get("temperature_c")
            temperature_text = (
                f"{float(temperature):.1f} °C"
                if isinstance(temperature, (int, float)) and not isinstance(temperature, bool)
                else "-"
            )
            values["meta"].set(
                f"Address: {sensor.get('address', '?')}    Temperature: {temperature_text}"
            )
            values["accel"].set(self._axes(sensor.get("accel_g"), 4))
            values["gyro"].set(self._axes(sensor.get("gyro_dps"), 2))

        now = time.monotonic()
        dt = 0.02 if self.last_sample_time is None else now - self.last_sample_time
        self.last_sample_time = now
        estimate = self.estimator.update(sensors, dt)
        if estimate is None:
            self.joints.set(f"Waiting for both sensors ({connected}/2 connected)")
        else:
            self.joints.set(
                "Shoulder flex {shoulder_flexion:6.1f}°   "
                "abduction {shoulder_abduction:6.1f}°   "
                "rotation {shoulder_rotation_proxy:6.1f}°   "
                "elbow {elbow_flexion:6.1f}°".format(**estimate)
            )

    def calibrate(self) -> None:
        self.estimator.reset_calibration()
        self.last_sample_time = None
        self.joints.set("Calibration reset · hold the arm still for the next sample")

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.window.destroy()
        except tk.TclError:
            pass
