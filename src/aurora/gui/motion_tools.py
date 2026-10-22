# Majd, Christian
"""Movement-profile comparison and pose playback controls."""

from __future__ import annotations

from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from aurora.gui.project_support import (
    DEFAULT_RECORDINGS_DIRECTORY,
    DEFAULT_REFERENCE_DIRECTORY,
    MOVEMENT_RECOGNITION_SCRIPT,
    POSE_RECORDING_SCRIPT,
    REPOSITORY_ROOT,
    display_path,
    project_path,
)


class MotionToolsMixin:
    """Build the movement tools and launch their workers."""

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
            command=self.start_pose_playback,
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
        """Compare a recorded movement with the saved references."""
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
                "Run Setup.bat (Windows) or Setup.command (macOS/Linux), then refresh Info.",
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

    def start_pose_playback(self) -> None:
        """Play a camera pose recording on the limited arm model."""
        recording = self._selected_motion_recording()
        if recording is None:
            return
        if self.simulation_python is None:
            messagebox.showerror(
                "Simulation environment missing",
                "Run Setup.bat (Windows) or Setup.command (macOS/Linux), then refresh Info.",
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
