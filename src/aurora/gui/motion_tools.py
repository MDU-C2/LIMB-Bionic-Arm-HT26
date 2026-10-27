# Majd, Christian
"""Movement-profile comparison and pose playback controls."""

from __future__ import annotations

import math
import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from aurora.gui.project_support import (
    DEFAULT_RECORDINGS_DIRECTORY,
    DEFAULT_REFERENCE_DIRECTORY,
    MOVEMENT_MODEL,
    MOVEMENT_RECOGNITION_SCRIPT,
    REPOSITORY_ROOT,
    console_python,
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
            "Camera movement profiles (research)",
            "Capture profiles, compare whole movements and review training runs. This is separate from EMG grip recognition.",
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
        shortcuts = ttk.Frame(recording, style="Card.TFrame")
        shortcuts.grid(row=7, column=0, sticky="w", pady=8)
        for title, path in (
            ("Use captured profiles", REPOSITORY_ROOT / DEFAULT_REFERENCE_DIRECTORY),
            ("Use inherited LIMB1 profiles", REPOSITORY_ROOT / "data/legacy/limb1/training"),
        ):
            ttk.Button(
                shortcuts,
                text=title,
                command=lambda path=path: self.motion_references.set(str(path)),
                state="normal" if path.is_dir() else "disabled",
            ).pack(side="left", padx=4)
        ttk.Label(shortcuts, text="Unavailable folders stay disabled.").pack(side="left", padx=8)

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
        self.movement_model_path = MOVEMENT_MODEL
        self.movement_model_status = tk.StringVar(self, value="Bundled movement-profile model")
        self.movement_threshold = tk.StringVar(self, value="0.51")
        model_row = ttk.Frame(recognition, style="Card.TFrame")
        model_row.grid(row=5, column=0, sticky="w", pady=8)
        ttk.Button(
            model_row, text="Review / select movement model", command=self._select_movement_model
        ).pack(side="left", padx=4)
        ttk.Button(
            model_row, text="Use bundled model", command=self._use_bundled_movement_model
        ).pack(side="left", padx=4)
        ttk.Label(model_row, text="Unknown-distance threshold").pack(side="left", padx=6)
        ttk.Entry(model_row, textvariable=self.movement_threshold, width=8).pack(side="left")
        ttk.Label(
            recognition,
            textvariable=self.movement_model_status,
            wraplength=780,
            style="CardText.TLabel",
        ).grid(row=6, column=0, sticky="w")

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
        if not self.movement_ai_ready:
            messagebox.showerror(
                "Motion AI unavailable",
                "Run Setup, then Refresh status under Help & status.",
            )
            return
        try:
            threshold = float(self.movement_threshold.get())
            if not math.isfinite(threshold) or threshold <= 0:
                raise ValueError("Unknown-distance threshold must be positive and finite.")
        except ValueError as error:
            messagebox.showerror("Check recognition settings", str(error))
            return
        command = [
            str(console_python(Path(sys.executable))),
            "-u",
            str(MOVEMENT_RECOGNITION_SCRIPT),
            str(recording.resolve()),
            "--references",
            str(references.resolve()),
            "--model",
            str(self.movement_model_path),
            "--threshold",
            str(threshold),
        ]
        self._start_program("Movement recognition", "motion_ai", command)

    def _use_bundled_movement_model(self):
        self.movement_model_path = MOVEMENT_MODEL
        self.movement_model_status.set("Bundled movement-profile model; used for the next analysis")
        self._update_controls()
        self._refresh_info()

    def _select_movement_model(self):
        selected = filedialog.askopenfilename(
            title="Choose a movement ONNX model from a completed training run",
            filetypes=(("Movement model", "*.onnx"),),
        )
        if not selected:
            return

        def load(cancelled):
            from aurora.json_utils import read_json_object
            from aurora.ml.experiment import verify_model_manifest
            from aurora.ml.features import PREPROCESSING
            from aurora.ml.recognition import MovementRecognizer
            from aurora.recording.emg_labels import digest

            model = Path(selected).resolve()
            manifest_path = model.with_suffix(".json")
            if not manifest_path.is_file():
                manifest_path = model.parent / "training.json"
            manifest = read_json_object(manifest_path)
            if manifest.get("status") not in {"complete", "inherited"} or manifest.get(
                "artifacts", {}
            ).get(model.name) != digest(model):
                raise ValueError(
                    "Select a model with a matching completed training manifest or inherited model sidecar."
                )
            if any(
                manifest.get("preprocessing", {}).get(key) != value
                for key, value in PREPROCESSING.items()
            ):
                raise ValueError("The model preprocessing does not match movement profiles.")
            verify_model_manifest(model)
            MovementRecognizer(model)
            return model, manifest, manifest_path

        def ready(result, error):
            if error:
                messagebox.showerror("Cannot select movement model", str(error))
                return
            model, manifest, manifest_path = result
            dialog = tk.Toplevel(self)
            dialog.title("AURORA | Review movement model")
            ttk.Label(
                dialog,
                text=f"Model: {model.name}\nStatus: {manifest['status']}\nSplit: {manifest.get('split_method', 'inherited baseline')}\n{manifest.get('results', {})}\n\nProfile identification is not EMG classification or anatomical accuracy.\nSelection applies to the next analysis and preserves the bundled model.",
                padding=15,
                wraplength=580,
            ).pack()
            ttk.Button(
                dialog,
                text="Open training / provenance",
                command=lambda: self._open_existing(manifest_path),
            ).pack(pady=6)

            def select():
                self.movement_model_path = model
                self.movement_model_status.set(f"Selected for next profile analysis: {model}")
                self._update_controls()
                self._refresh_info()
                dialog.destroy()

            ttk.Button(dialog, text="Use for movement-profile recognition", command=select).pack(
                pady=10
            )

        self.background_tasks.submit("movement_model", load, ready)

    def start_pose_playback(self) -> None:
        recording = self._selected_motion_recording()
        if recording is not None:
            self.workflow.show_tool(
                "pose_play", {"recording": str(recording), "loop": self.loop_pose_playback.get()}
            )
