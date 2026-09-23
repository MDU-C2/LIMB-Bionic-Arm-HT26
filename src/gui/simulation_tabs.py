"""Simulation and Motion AI pages for the AURORA control center."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from project_support import (
    DEFAULT_SIMULATION_OUTPUT_DIRECTORY,
    INTERACTIVE_SIMULATION_SCRIPT,
    MANUAL_SIMULATION_SCRIPT,
    MOVEMENT_RECOGNITION_SCRIPT,
    POSE_RECORDING_SCRIPT,
    REPOSITORY_ROOT,
    SIMULATION_SCRIPT,
    display_path,
    has_trajectory_data,
    project_path,
)


class SimulationTabsMixin:
    """Build and run simulation and motion-analysis tools."""

    def _build_simulation_tab(self, tab: ttk.Frame) -> None:
        """Populate controls for the interactive and trajectory simulators."""
        tab.columnconfigure(0, weight=1)
        self._heading(
            tab,
            "LIMB simulation",
            "Run the full arm task simulator, play recordings, or inspect joints manually.",
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
            text="Open full simulator",
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

        playback = self._card(
            tab,
            3,
            "Trajectory playback",
            "Select a trial folder containing a saved rollout or angles.npz.",
        )
        path_row = ttk.Frame(playback, style="Card.TFrame")
        path_row.grid(row=2, column=0, sticky="ew")
        path_row.columnconfigure(0, weight=1)
        ttk.Entry(path_row, textvariable=self.trajectory_path).grid(row=0, column=0, sticky="ew")
        ttk.Button(path_row, text="Browse", command=self._browse_trajectory).grid(
            row=0, column=1, padx=(8, 0)
        )

        options = ttk.Frame(playback, style="Card.TFrame")
        options.grid(row=3, column=0, sticky="w", pady=(12, 0))
        ttk.Checkbutton(
            options,
            text="Loop playback",
            variable=self.loop_playback,
            style="Card.TCheckbutton",
        ).pack(side="left", padx=(0, 18))
        ttk.Checkbutton(
            options,
            text="Refit DMP from angles",
            variable=self.refit_dmp,
            style="Card.TCheckbutton",
        ).pack(side="left")

        playback_actions = ttk.Frame(playback, style="Card.TFrame")
        playback_actions.grid(row=4, column=0, sticky="w", pady=(14, 0))
        self.playback_button = ttk.Button(
            playback_actions,
            text="Play trajectory",
            style="Accent.TButton",
            command=self.start_simulation,
        )
        self.playback_button.pack(side="left", padx=(0, 8))
        ttk.Button(
            playback_actions,
            text="Open data folder",
            style="Secondary.TButton",
            command=lambda: self._open_user_path(self.trajectory_path.get()),
        ).pack(side="left")

        manual = self._card(
            tab,
            4,
            "Manual joint control",
            "Open the arm model and move elbow and shoulder joints with degree sliders.",
        )
        self.manual_button = ttk.Button(
            manual,
            text="Open joint sliders",
            style="Accent.TButton",
            command=self.start_manual_simulation,
        )
        self.manual_button.grid(row=2, column=0, sticky="w")

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

    def _browse_trajectory(self) -> None:
        """Choose a trajectory directory and store a portable path."""
        current = project_path(self.trajectory_path.get())
        initial = current if current.is_dir() else REPOSITORY_ROOT
        selected = filedialog.askdirectory(title="Select trajectory folder", initialdir=initial)
        if selected:
            self.trajectory_path.set(display_path(Path(selected)))

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
        )

    def start_simulation(self) -> None:
        """Validate the selected data and launch trajectory playback."""
        if self.simulation_python is None:
            messagebox.showerror(
                "Simulation environment missing",
                "Create aurora-simulation from src/simulation/environment.yml, then refresh Info.",
            )
            return
        trajectory_value = self.trajectory_path.get().strip()
        if not trajectory_value:
            messagebox.showerror("Trajectory required", "Choose a trajectory folder first.")
            return
        trajectory = project_path(trajectory_value)
        if not trajectory.is_dir():
            messagebox.showerror("Trajectory not found", f"Folder not found:\n{trajectory}")
            return
        if not has_trajectory_data(trajectory):
            messagebox.showerror(
                "Trajectory data not found",
                "The selected folder has no supported angles or DMP rollout NPZ file.",
            )
            return
        command = [
            str(self.simulation_python),
            "-u",
            str(SIMULATION_SCRIPT),
            "--path",
            str(trajectory.resolve()),
        ]
        if self.loop_playback.get():
            command.append("--loop")
        if self.refit_dmp.get():
            command.append("--refit")
        self._start_program("Trajectory playback", "simulation", command)

    def start_manual_simulation(self) -> None:
        """Launch the lightweight PyBullet joint-slider tool."""
        if self.simulation_python is None:
            messagebox.showerror(
                "Simulation environment missing",
                "Create aurora-simulation from src/simulation/environment.yml, then refresh Info.",
            )
            return
        self._start_program(
            "Manual simulation",
            "simulation",
            [str(self.simulation_python), "-u", str(MANUAL_SIMULATION_SCRIPT)],
        )

    # Recording programs
