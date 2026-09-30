"""Dedicated live window for the shoulder/wrist IMUs and EMG channel."""

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

from emg_protocol import EmgActivationEstimator, extract_emg
from imu_protocol import ROLE_LABELS, extract_imus, i2c_wiring_hint
from sensor_fusion import DualImuArmEstimator


class ImuMonitorWindow:
    """Render the ESP32 IMU and EMG streams outside the main window."""

    def __init__(self, parent: tk.Misc, reconnect_command, disconnect_command) -> None:
        self.window = tk.Toplevel(parent)
        self.window.title("AURORA | Live IMU and EMG sensors")
        self.window.geometry("900x650")
        self.window.minsize(700, 580)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.closed = False
        self.estimator = DualImuArmEstimator()
        self.emg_estimator = EmgActivationEstimator()
        self.last_sample_time: float | None = None
        self.connection = tk.StringVar(value="Waiting for serial connection")
        self.device = tk.StringVar(value="ESP32: no data")
        self.joints = tk.StringVar(value="Waiting for IMU data")
        self.emg_value = tk.StringVar(value="Waiting for EMG data")
        self.emg_meta = tk.StringVar(value="ADC channel: -    GPIO: -")
        self.emg_progress = tk.DoubleVar(value=0.0)
        self.sensor_values: dict[str, dict[str, tk.StringVar]] = {}

        root = ttk.Frame(self.window, padding=18)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)

        ttk.Label(root, text="Live sensor monitor", font=("Segoe UI", 17, "bold")).grid(
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
        ttk.Label(
            root,
            textvariable=self.device,
            foreground="#667085",
            wraplength=850,
            justify="left",
        ).grid(
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

        emg = ttk.LabelFrame(root, text="EMG muscle signal", padding=14)
        emg.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(14, 0))
        emg.columnconfigure(0, weight=1)
        ttk.Label(emg, textvariable=self.emg_value, font=("Cascadia Mono", 10)).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(emg, textvariable=self.emg_meta, foreground="#667085").grid(
            row=1, column=0, sticky="w", pady=(3, 0)
        )
        ttk.Progressbar(
            emg,
            variable=self.emg_progress,
            maximum=100.0,
            length=250,
        ).grid(row=0, column=1, rowspan=2, sticky="e", padx=(12, 0))

        fused = ttk.LabelFrame(root, text="Relative arm estimate", padding=14)
        fused.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(14, 0))
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
        now = time.monotonic()
        dt = 0.02 if self.last_sample_time is None else now - self.last_sample_time
        self.last_sample_time = now
        device = str(packet.get("device", "ESP32"))
        sensors = extract_imus(packet)
        roles = tuple(
            role for role in ("shoulder", "wrist")
            if sensors.get(role, {}).get("connected") is True
        )
        mode_text = (
            "Dual IMU (2/2)"
            if len(roles) == 2
            else f"Single IMU (1/2: {roles[0]})"
            if len(roles) == 1
            else "No IMU detected (0/2)"
        )
        uptime = packet.get("uptime_ms")
        uptime_text = (
            f" · uptime {float(uptime) / 1000:.1f} s"
            if isinstance(uptime, (int, float)) and not isinstance(uptime, bool)
            else ""
        )
        i2c = packet.get("i2c")
        i2c_text = ""
        if isinstance(i2c, dict):
            sda_pin = i2c.get("sda_pin", "?")
            scl_pin = i2c.get("scl_pin", "?")
            profile = i2c.get("profile", "unknown")
            sda_pullup = i2c.get("external_sda_pullup")
            scl_pullup = i2c.get("external_scl_pullup")
            if isinstance(sda_pullup, bool) and isinstance(scl_pullup, bool):
                pullup_text = (
                    "external pull-ups OK"
                    if sda_pullup and scl_pullup
                    else f"external pull-ups SDA={'yes' if sda_pullup else 'no'} "
                    f"SCL={'yes' if scl_pullup else 'no'}"
                )
                pullup_text = f" · {pullup_text}"
            else:
                pullup_text = ""
            i2c_text = (
                f" · I2C {profile} SDA{sda_pin}/SCL{scl_pin}{pullup_text}"
            )
            if len(roles) == 0:
                hint = i2c_wiring_hint(i2c)
                if hint and not (sda_pullup and scl_pullup):
                    i2c_text = f"{i2c_text} · {hint}"
        self.device.set(
            f"ESP32: {device} · {mode_text}{uptime_text}{i2c_text}"
        )
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

        emg = extract_emg(packet)
        if emg.get("connected") is True:
            raw = int(emg["adc_raw"])
            activation = self.emg_estimator.update(raw, dt)
            if activation is None:
                percent = round(self.emg_estimator.calibration_progress * 100)
                self.emg_value.set(f"Raw {raw:4d} ADC   Calibrating rest {percent:3d}%")
                self.emg_progress.set(percent)
            else:
                self.emg_value.set(
                    f"Raw {raw:4d} ADC   Activation {activation * 100:5.1f}%"
                )
                self.emg_progress.set(activation * 100.0)
            self.emg_meta.set(
                f"ADC channel: {emg.get('adc_channel', '?')}    "
                f"GPIO: {emg.get('gpio', '?')}"
            )
        else:
            self.emg_value.set(f"Disconnected · {emg.get('error', 'no data')}")
            self.emg_meta.set("ADC channel: -    GPIO: -")
            self.emg_progress.set(0.0)

        estimate = self.estimator.update(sensors, dt)
        if estimate is None:
            self.joints.set(f"Waiting for an IMU ({connected}/2 connected)")
            return
        if not estimate:
            self.joints.set(
                "Wrist IMU connected · camera or shoulder IMU needed for arm control"
            )
            return
        elbow = (
            f"{estimate['elbow_flexion']:6.1f}°"
            if "elbow_flexion" in estimate
            else "-- (camera or second IMU needed)"
        )
        rotation = (
            f"{estimate['shoulder_rotation_proxy']:6.1f}°"
            if "shoulder_rotation_proxy" in estimate
            else "-- (camera needed)"
        )
        self.joints.set(
            (
                "Shoulder flex {shoulder_flexion:6.1f}°   "
                "left/right {shoulder_abduction:6.1f}°   "
                f"rotation {rotation}   "
                f"elbow {elbow}"
            ).format(**estimate)
        )

    def calibrate(self) -> None:
        self.estimator.reset_calibration()
        self.emg_estimator.reset()
        self.last_sample_time = None
        self.joints.set("Calibration reset · hold the arm still for the next sample")
        self.emg_value.set("Calibration reset · relax the muscle briefly")
        self.emg_progress.set(0.0)

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.window.destroy()
        except tk.TclError:
            pass
