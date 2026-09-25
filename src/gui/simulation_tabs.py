"""Simulation and Motion AI pages for the AURORA control center."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from project_support import (
    DEFAULT_RECORDINGS_DIRECTORY,
    DEFAULT_REFERENCE_DIRECTORY,
    DEFAULT_SIMULATION_OUTPUT_DIRECTORY,
    INTERACTIVE_SIMULATION_SCRIPT,
    MOVEMENT_RECOGNITION_SCRIPT,
    POSE_RECORDING_SCRIPT,
    REPOSITORY_ROOT,
    display_path,
    project_path,
)


class SimulationTabsMixin:
    """Build and run simulation and motion-analysis tools."""

    def _build_simulation_tab(self, tab: ttk.Frame) -> None:
        """Populate keyboard and live-sensor controls for the interactive simulator."""
        tab.columnconfigure(0, weight=1)
        self._heading(
            tab,
            "LIMB simulation",
            "Use the interactive task scene with keyboard control or live camera + IMUs.",
        )

        interactive = self._card(
            tab,
            2,
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
            text="Physics mode (gravity and motor torques)",
            variable=self.dynamic_simulation,
            style="Card.TCheckbutton",
        ).grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Checkbutton(
            interactive,
            text="Save torque CSV in outputs/simulation",
            variable=self.save_simulation_torque,
            style="Card.TCheckbutton",
        ).grid(row=4, column=0, sticky="w", pady=(4, 0))

        fusion = self._card(
            tab,
            3,
            "Interactive simulator with live sensors",
            "Open the same table-and-target scene, show the camera monitor, and control "
            "the arm from the OAK-D plus shoulder and wrist IMUs.",
        )
        fusion_settings = ttk.Frame(fusion, style="Card.TFrame")
        fusion_settings.grid(row=2, column=0, sticky="ew")
        fusion_settings.columnconfigure(0, weight=1)
        ttk.Label(fusion_settings, text="ESP32 port", style="Card.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        self.fusion_port_box = ttk.Combobox(
            fusion_settings, textvariable=self.recording_serial_port
        )
        self.fusion_port_box.grid(row=1, column=0, sticky="ew", pady=(4, 0))
        ttk.Label(fusion_settings, text="Baud", style="Card.TLabel").grid(
            row=0, column=1, sticky="w", padx=(12, 0)
        )
        ttk.Entry(
            fusion_settings, textvariable=self.recording_serial_baud, width=10
        ).grid(row=1, column=1, padx=(12, 0), pady=(4, 0))
        ttk.Label(fusion_settings, text="Camera correction", style="Card.TLabel").grid(
            row=0, column=2, sticky="w", padx=(12, 0)
        )
        ttk.Entry(
            fusion_settings, textvariable=self.fusion_camera_weight, width=10
        ).grid(row=1, column=2, padx=(12, 0), pady=(4, 0))
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

        fusion_actions = ttk.Frame(fusion, style="Card.TFrame")
        fusion_actions.grid(row=3, column=0, sticky="w", pady=(12, 0))
        self.fusion_button = ttk.Button(
            fusion_actions,
            text="Start live interactive control",
            style="Accent.TButton",
            command=self.start_live_sensor_fusion,
        )
        self.fusion_button.pack(side="left", padx=(0, 8))
        ttk.Button(
            fusion_actions,
            text="Open IMU monitor",
            command=self.open_sensor_window,
        ).pack(side="left", padx=(0, 8))
        ttk.Button(
            fusion_actions,
            text="Open camera monitor",
            command=self.open_camera_monitor,
        ).pack(side="left", padx=(0, 14))
        ttk.Checkbutton(
            fusion_actions,
            text="Use stereo depth",
            variable=self.fusion_depth,
            style="Card.TCheckbutton",
        ).pack(side="left")

    def _build_motion_ai_tab(self, tab: ttk.Frame) -> None:
        """Populate movement recognition and camera-pose playback controls."""
        tab.columnconfigure(0, weight=1)
        self._heading(
            tab,
            "Motion AI",
            "Inspect a camera pose recording, compare it with references, or play it on the arm.",
        )

        recording = self._card(
            tab,
            2,
            "Pose recording",
            "Choose a JSON recording containing shoulder, elbow, and hand landmarks.",
        )
        recording_row = ttk.Frame(recording, style="Card.TFrame")
        recording_row.grid(row=2, column=0, sticky="ew")
        recording_row.columnconfigure(0, weight=1)
        ttk.Entry(recording_row, textvariable=self.motion_recording).grid(
            row=0, column=0, sticky="ew"
        )
        ttk.Button(
            recording_row,
            text="Browse",
            command=self._browse_motion_recording,
        ).grid(row=0, column=1, padx=(8, 0))

        recognition = self._card(
            tab,
            3,
            "Movement profile recognition",
            "Compare the recording with reference JSON files using the GRU model.",
        )
        reference_row = ttk.Frame(recognition, style="Card.TFrame")
        reference_row.grid(row=2, column=0, sticky="ew")
        reference_row.columnconfigure(0, weight=1)
        ttk.Entry(reference_row, textvariable=self.motion_references).grid(
            row=0, column=0, sticky="ew"
        )
        ttk.Button(
            reference_row,
            text="Browse references",
            command=self._browse_motion_references,
        ).grid(row=0, column=1, padx=(8, 0))
        self.analyze_motion_button = ttk.Button(
            recognition,
            text="Analyze recording",
            style="Accent.TButton",
            command=self.start_movement_recognition,
        )
        self.analyze_motion_button.grid(row=3, column=0, sticky="w", pady=(12, 0))
        ttk.Label(
            recognition,
            textvariable=self.motion_ai_status,
            style="CardText.TLabel",
        ).grid(row=4, column=0, sticky="w", pady=(8, 0))

        playback = self._card(
            tab,
            4,
            "Pose playback",
            "Map the camera landmarks to the real arm limits and animate the fingers.",
        )
        playback_actions = ttk.Frame(playback, style="Card.TFrame")
        playback_actions.grid(row=2, column=0, sticky="w")
        self.pose_playback_button = ttk.Button(
            playback_actions,
            text="Play on arm",
            style="Accent.TButton",
            command=self.start_pose_recording_simulation,
        )
        self.pose_playback_button.pack(side="left", padx=(0, 14))
        ttk.Checkbutton(
            playback_actions,
            text="Loop playback",
            variable=self.loop_pose_playback,
            style="Card.TCheckbutton",
        ).pack(side="left")

    def _browse_motion_recording(self) -> None:
        """Choose a pose-recording JSON file."""
        current = project_path(self.motion_recording.get() or DEFAULT_RECORDINGS_DIRECTORY)
        initial = current.parent if current.is_file() else current
        if not initial.is_dir():
            initial = REPOSITORY_ROOT
        selected = filedialog.askopenfilename(
            title="Select pose recording",
            initialdir=initial,
            filetypes=(("JSON recording", "*.json"), ("All files", "*.*")),
        )
        if selected:
            self.motion_recording.set(display_path(Path(selected)))

    def _browse_motion_references(self) -> None:
        """Choose the folder containing reference pose recordings."""
        current = project_path(self.motion_references.get() or DEFAULT_REFERENCE_DIRECTORY)
        initial = current if current.is_dir() else REPOSITORY_ROOT
        selected = filedialog.askdirectory(title="Select reference folder", initialdir=initial)
        if selected:
            self.motion_references.set(display_path(Path(selected)))

    def _selected_motion_recording(self) -> Path | None:
        """Validate and return the selected pose recording."""
        value = self.motion_recording.get().strip()
        if not value:
            messagebox.showerror("Recording required", "Choose a pose recording first.")
            return None
        recording = project_path(value)
        if not recording.is_file():
            messagebox.showerror("Recording not found", f"File not found:\n{recording}")
            return None
        return recording

    def start_movement_recognition(self) -> None:
        """Analyze a pose recording with the migrated GRU model."""
        recording = self._selected_motion_recording()
        if recording is None:
            return
        references = project_path(self.motion_references.get().strip())
        if not references.is_dir():
            messagebox.showerror("References not found", f"Folder not found:\n{references}")
            return
        if self.simulation_python is None or not self.movement_ai_ready:
            messagebox.showerror(
                "Motion AI unavailable",
                "Install the environment from src/simulation/environment.yml, then refresh Info.",
            )
            return
        command = [
            str(self.simulation_python),
            "-u",
            str(MOVEMENT_RECOGNITION_SCRIPT),
            str(recording.resolve()),
            "--references",
            str(references.resolve()),
        ]
        self._start_program("Movement recognition", "motion_ai", command)

    def start_pose_recording_simulation(self) -> None:
        """Play a camera pose recording on the limited arm model."""
        recording = self._selected_motion_recording()
        if recording is None:
            return
        if self.simulation_python is None:
            messagebox.showerror(
                "Simulation environment missing",
                "Create aurora-simulation from src/simulation/environment.yml, then refresh Info.",
            )
            return
        command = [
            str(self.simulation_python),
            "-u",
            str(POSE_RECORDING_SCRIPT),
            str(recording.resolve()),
        ]
        if self.loop_pose_playback.get():
            command.append("--loop")
        self._start_program("Pose recording playback", "motion_ai", command)

    def start_interactive_simulation(self) -> None:
        """Launch the full LIMB task simulator in the managed environment."""
        if self.simulation_python is None:
            messagebox.showerror(
                "Simulation environment missing",
                "Create aurora-simulation from src/simulation/environment.yml, then refresh Info.",
            )
            return
        command = [str(self.simulation_python), "-u", str(INTERACTIVE_SIMULATION_SCRIPT)]
        command += ["--mode", "dynamic" if self.dynamic_simulation.get() else "kinematic"]
        if self.dynamic_simulation.get() and self.save_simulation_torque.get():
            output_name = f"torque_{datetime.now():%Y%m%d_%H%M%S}.csv"
            torque_output = REPOSITORY_ROOT / DEFAULT_SIMULATION_OUTPUT_DIRECTORY / output_name
            command += ["--torque-out", str(torque_output)]
        self._start_program(
            "Interactive task simulator",
            "simulation",
            command,
            parallel=all(
                process.kind == "preview" for process in self._active_processes()
            ),
        )

    def start_live_sensor_fusion(self) -> None:
        """Launch the interactive task scene under camera and dual-IMU control."""
        if self.simulation_python is None:
            messagebox.showerror(
                "Simulation environment missing",
                "Create aurora-simulation from src/simulation/environment.yml, then refresh Info.",
            )
            return
        port = self.recording_serial_port.get().strip()
        if not port:
            messagebox.showerror("Serial port required", "Choose the ESP32 serial port first.")
            return
        try:
            baud = int(self.recording_serial_baud.get().strip() or "115200")
            weight = float(self.fusion_camera_weight.get().strip())
            if baud <= 0 or not 0.0 <= weight <= 1.0:
                raise ValueError("Baud must be positive and camera correction must be 0 to 1.")
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
            "--port",
            port,
            "--baud",
            str(baud),
            "--side",
            self.recording_camera_side.get(),
            "--camera-weight",
            str(weight),
        ]
        if self.fusion_depth.get():
            command.append("--depth")
        if self.dynamic_simulation.get() and self.save_simulation_torque.get():
            output_name = f"torque_{datetime.now():%Y%m%d_%H%M%S}.csv"
            torque_output = REPOSITORY_ROOT / DEFAULT_SIMULATION_OUTPUT_DIRECTORY / output_name
            command += ["--torque-out", str(torque_output)]
        self._start_program("Interactive live sensor control", "simulation", command)

    # Recording programs
