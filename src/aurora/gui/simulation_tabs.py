# Majd, Christian
"""Simulation setup and live-control launchers."""

from __future__ import annotations

import math
from datetime import datetime
from tkinter import BooleanVar, filedialog, messagebox, ttk

from aurora.gui.motion_tools import MotionToolsMixin
from aurora.gui.project_support import (
    DEFAULT_RECORDINGS_DIRECTORY,
    DEFAULT_SIMULATION_OUTPUT_DIRECTORY,
    INTERACTIVE_SIMULATION_SCRIPT,
    REPOSITORY_ROOT,
)


class SimulationTabsMixin(MotionToolsMixin):
    """Build and run simulation and motion-analysis tools."""

    def _build_simulation_tab(self, tab: ttk.Frame) -> None:
        """Populate keyboard and live-sensor controls for the interactive simulator."""
        tab.columnconfigure(0, weight=1)
        self._heading(
            tab,
            "AURORA simulation",
            "Use the left-arm task scene with keyboard control or live camera + IMUs + EMG.",
        )

        interactive = self._card(
            tab,
            3,
            "Interactive task simulator",
            "Open the table-and-target scene with five arm actuators and hand controls. "
            "Physics mode uses gravity and position motors; direct mode previews joint poses.",
        )
        self.interactive_button = ttk.Button(
            interactive,
            text="Start with keyboard",
            style="Accent.TButton",
            command=self.start_interactive_simulation,
        )
        self.interactive_button.grid(row=2, column=0, sticky="w")
        ttk.Checkbutton(
            interactive,
            text="Physics mode: gravity and motor speed limits (slower than a person)",
            variable=self.dynamic_simulation,
            style="Card.TCheckbutton",
        ).grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Checkbutton(
            interactive,
            text="Save torque CSV in outputs/simulation",
            variable=self.save_simulation_torque,
            style="Card.TCheckbutton",
        ).grid(row=4, column=0, sticky="w", pady=(4, 0))

        table_settings = ttk.Frame(interactive, style="Card.TFrame")
        table_settings.grid(row=5, column=0, sticky="w", pady=(8, 0))
        ttk.Label(table_settings, text="Tabletop width × depth (cm)", style="Card.TLabel").pack(
            side="left"
        )
        ttk.Entry(table_settings, textvariable=self.table_width_cm, width=8).pack(
            side="left", padx=6
        )
        ttk.Label(table_settings, text="×", style="Card.TLabel").pack(side="left")
        ttk.Entry(table_settings, textvariable=self.table_depth_cm, width=8).pack(
            side="left", padx=6
        )

        fusion = self._card(
            tab,
            2,
            "Interactive simulator with live sensors",
            "1. Connect and open setup. Movement stays paused.\n"
            "2. Sit upright. Upper arm relaxed, elbow about 90°, open hand at marker 1, thumb up.\n"
            "3. Follow Align → Relaxed EMG → Firm grip in the setup window.\n"
            "4. Return the open hand to marker 1 and press Start control.",
        )
        fusion_settings = ttk.Frame(fusion, style="Card.TFrame")
        fusion_settings.grid(row=5, column=0, sticky="ew", pady=(8, 0))
        fusion_settings.columnconfigure(0, weight=1)
        ttk.Label(fusion_settings, text="BLE device name", style="Card.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Entry(fusion_settings, textvariable=self.recording_ble_device, state="readonly").grid(
            row=1, column=0, sticky="ew", pady=(4, 0)
        )
        ttk.Button(fusion_settings, text="Change", command=self._edit_ble_device).grid(
            row=1, column=1, padx=(8, 0)
        )
        ttk.Label(fusion_settings, text="Camera correction (0–1)", style="Card.TLabel").grid(
            row=0, column=2, sticky="w", padx=(12, 0)
        )
        ttk.Entry(fusion_settings, textvariable=self.fusion_camera_weight, width=10).grid(
            row=1, column=2, padx=(12, 0), pady=(4, 0)
        )
        ttk.Label(fusion_settings, text="Tracked arm", style="Card.TLabel").grid(
            row=0, column=3, sticky="w", padx=(12, 0)
        )
        ttk.Combobox(
            fusion_settings,
            textvariable=self.recording_camera_side,
            values=("left", "right"),
            state="readonly",
            width=9,
        ).grid(row=1, column=3, padx=(12, 0), pady=(4, 0))
        ttk.Label(fusion_settings, text="Grip source", style="Card.TLabel").grid(
            row=2, column=0, sticky="w", pady=(8, 0)
        )
        ttk.Combobox(
            fusion_settings,
            textvariable=self.fusion_grip_source,
            values=("Camera", "EMG", "Auto"),
            state="readonly",
            width=14,
        ).grid(row=3, column=0, sticky="w", pady=(4, 0))
        ttk.Label(
            fusion_settings,
            text="EMG: follow relaxed muscle and firm grip in the live setup window.",
            style="Card.TLabel",
        ).grid(row=3, column=1, columnspan=3, sticky="w", padx=(12, 0))

        for column, label, variable in (
            (0, "Upper-arm twist axis", self.fusion_shoulder_twist_axis),
            (2, "Forearm twist axis", self.fusion_wrist_twist_axis),
        ):
            ttk.Label(fusion_settings, text=label, style="Card.TLabel").grid(
                row=4, column=column, sticky="w", pady=(8, 0)
            )
            ttk.Combobox(
                fusion_settings,
                textvariable=variable,
                values=("x", "y", "z", "-x", "-y", "-z"),
                state="readonly",
                width=14,
            ).grid(row=5, column=column, sticky="w", pady=(4, 0))
        ttk.Label(
            fusion_settings,
            text="0x6B on upper arm, 0x6A on forearm. Selected axes run along each limb. "
            "Keep shoulders, hips, elbow and wrist in view.",
            style="Card.TLabel",
        ).grid(row=6, column=0, columnspan=4, sticky="w", pady=(6, 0))

        fusion_actions = ttk.Frame(fusion, style="Card.TFrame")
        fusion_actions.grid(row=2, column=0, sticky="w")
        self.fusion_button = ttk.Button(
            fusion_actions,
            text="1. Connect + open live setup",
            style="Accent.TButton",
            command=self.start_live_control,
        )
        self.fusion_button.pack(side="left", padx=(0, 8))
        ttk.Button(
            fusion_settings,
            text="Open live BLE graphs",
            command=lambda: self.start_sensor_preview("record_ble_sensors.py"),
        ).grid(row=7, column=0, sticky="w", pady=(10, 0))
        ttk.Button(
            fusion_settings,
            text="Open camera monitor",
            command=self.open_camera_monitor,
        ).grid(row=7, column=1, sticky="w", padx=8, pady=(10, 0))
        ttk.Checkbutton(
            fusion_settings,
            text="Use stereo depth",
            variable=self.fusion_depth,
            style="Card.TCheckbutton",
        ).grid(row=7, column=2, sticky="w", pady=(10, 0))

        self.replay_button = ttk.Button(
            fusion,
            text="Replay a recorded cup trial (no sensors)",
            command=self.start_recorded_control,
        )
        self.replay_button.grid(row=3, column=0, sticky="w", pady=(10, 0))
        advanced_live = BooleanVar(value=False)
        ttk.Checkbutton(
            fusion,
            text="Live settings (arm, grip source, mounting axes and camera)",
            variable=advanced_live,
            command=lambda: (
                fusion_settings.grid() if advanced_live.get() else fusion_settings.grid_remove()
            ),
            style="Card.TCheckbutton",
        ).grid(row=4, column=0, sticky="w", pady=(10, 0))
        fusion_settings.grid_remove()

    def start_recorded_control(self) -> None:
        if self.simulation_python is None:
            messagebox.showerror("Environment missing", "Run project setup first.")
            return
        trial = filedialog.askopenfilename(
            title="Choose trial.json from a recorded cup trial",
            initialdir=REPOSITORY_ROOT / DEFAULT_RECORDINGS_DIRECTORY,
            filetypes=(("Trial metadata", "trial.json"),),
        )
        if not trial:
            return
        table_arguments = self._table_arguments()
        if table_arguments is None:
            return
        command = [
            str(self.simulation_python),
            "-u",
            str(INTERACTIVE_SIMULATION_SCRIPT),
            "--replay",
            trial,
            "--grip-source",
            "emg",
            "--mode",
            "dynamic" if self.dynamic_simulation.get() else "kinematic",
            "--camera-weight",
            self.fusion_camera_weight.get(),
            *table_arguments,
        ]
        self._start_program("Recorded live control", "simulation", command)

    def start_interactive_simulation(self) -> None:
        """Launch the full AURORA task simulator in the managed environment."""
        if self.simulation_python is None:
            messagebox.showerror(
                "Simulation environment missing",
                "Run Setup.bat (Windows) or Setup.command (macOS/Linux), then refresh Info.",
            )
            return
        command = [str(self.simulation_python), "-u", str(INTERACTIVE_SIMULATION_SCRIPT)]
        table_arguments = SimulationTabsMixin._table_arguments(self)
        if table_arguments is None:
            return
        command += table_arguments
        command += ["--mode", "dynamic" if self.dynamic_simulation.get() else "kinematic"]
        if self.dynamic_simulation.get() and self.save_simulation_torque.get():
            output_name = f"torque_{datetime.now():%Y%m%d_%H%M%S}.csv"
            torque_output = REPOSITORY_ROOT / DEFAULT_SIMULATION_OUTPUT_DIRECTORY / output_name
            command += ["--torque-out", str(torque_output)]
        self._start_program(
            "Interactive task simulator",
            "simulation",
            command,
            parallel=all(process.kind == "preview" for process in self._active_processes()),
        )

    def start_live_control(self) -> None:
        """Launch the interactive task scene under camera and available-IMU control."""
        if self.simulation_python is None:
            messagebox.showerror(
                "Simulation environment missing",
                "Run Setup.bat (Windows) or Setup.command (macOS/Linux), then refresh Info.",
            )
            return
        device = self.recording_ble_device.get().strip()
        if not device:
            messagebox.showerror("BLE device required", "Enter the ESP32 BLE device name first.")
            return
        table_arguments = SimulationTabsMixin._table_arguments(self)
        if table_arguments is None:
            return
        try:
            weight = float(self.fusion_camera_weight.get().strip())
            if not 0.0 <= weight <= 1.0:
                raise ValueError("Camera correction must be 0 to 1.")
        except ValueError as error:
            messagebox.showerror("Invalid fusion settings", str(error))
            return
        camera_preview = self.processes.get("preview:record_oak_pose.py")
        if camera_preview is not None and camera_preview.process.poll() is None:
            messagebox.showinfo(
                "Camera already in use",
                "Close the standalone camera monitor before starting live control. "
                "The interactive simulator opens its own camera monitor.",
            )
            return
        ble_preview = self.processes.get("preview:record_ble_sensors.py")
        if ble_preview is not None and ble_preview.process.poll() is None:
            messagebox.showinfo(
                "BLE device already in use",
                "Close the standalone BLE monitor before starting live control. "
                "The simulator opens its own sensor monitor.",
            )
            return
        self._stop_serial_dashboard()
        if self.imu_monitor is not None and self.imu_monitor.exists:
            self.imu_monitor.close()
            self.imu_monitor = None
        command = [
            str(self.simulation_python),
            "-u",
            str(INTERACTIVE_SIMULATION_SCRIPT),
            "--mode",
            "dynamic" if self.dynamic_simulation.get() else "kinematic",
            "--control",
            "camera-imu",
            "--transport",
            "ble",
            "--device",
            device,
            "--side",
            self.recording_camera_side.get(),
            "--camera-weight",
            str(weight),
            "--grip-source",
            self.fusion_grip_source.get().lower(),
            "--shoulder-twist-axis",
            self.fusion_shoulder_twist_axis.get(),
            "--wrist-twist-axis",
            self.fusion_wrist_twist_axis.get(),
        ]
        command += table_arguments
        if self.fusion_depth.get():
            command.append("--depth")
        if self.dynamic_simulation.get() and self.save_simulation_torque.get():
            output_name = f"torque_{datetime.now():%Y%m%d_%H%M%S}.csv"
            torque_output = REPOSITORY_ROOT / DEFAULT_SIMULATION_OUTPUT_DIRECTORY / output_name
            command += ["--torque-out", str(torque_output)]
        self._start_program("Interactive live sensor control", "simulation", command)

    def _table_arguments(self) -> list[str] | None:
        values = []
        try:
            for attribute in ("table_width_cm", "table_depth_cm"):
                variable = getattr(self, attribute, None)
                value = float(variable.get()) if variable is not None else 60
                if not math.isfinite(value) or value < 12:
                    raise ValueError("Table dimensions must be at least 12 cm.")
                values.append(str(value))
        except ValueError as error:
            messagebox.showerror("Invalid table size", str(error))
            return None
        return ["--table-width-cm", values[0], "--table-depth-cm", values[1]]
