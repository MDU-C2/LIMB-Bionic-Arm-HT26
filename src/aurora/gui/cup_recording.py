# Majd
"""Simple controls for repeated cup transfers."""

from __future__ import annotations

import json
import math
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

from aurora.gui.project_support import console_python, project_path
from aurora.paths import PACKAGE_DIRECTORY

CUP_PROCESS_KEY = "recording:cup_trials"


class CupRecordingMixin:
    def _initialize_cup_recording_state(self) -> None:
        self.cup_duration = tk.StringVar(value="0")
        self.cup_condition = tk.StringVar(value="natural")
        self._cup_calibration_stage = "connect"
        self._cup_calibration_busy = False
        self.cup_status = tk.StringVar(
            value="Connect the camera and BLE, then calibrate before recording."
        )
        self.cup_next_trial = tk.StringVar(value="T001")
        self.recording_advanced = tk.BooleanVar(value=False)
        self._cup_ready = False
        self._cup_recording = False
        self._cup_camera_handoff_pending = False

    def _build_cup_recording(self, tab) -> None:
        card = self._card(
            tab,
            2,
            "Cup transfers",
            "Sit upright with the open hand at 1, thumb up and elbow about 90°. "
            "Keep shoulders, hips, arm and table in the camera view.",
        )
        fields = ttk.Frame(card, style="Card.TFrame")
        fields.grid(row=2, column=0, sticky="ew")
        self.cup_fields = []
        for column, (label, variable) in enumerate(
            (
                ("Subject", self.recording_subject),
                ("Arm", self.recording_camera_side),
                ("Seconds per trial (0 = manual)", self.cup_duration),
            )
        ):
            fields.columnconfigure(column, weight=1)
            ttk.Label(fields, text=label, style="Card.TLabel").grid(
                row=0, column=column, sticky="w"
            )
            widget = (
                ttk.Combobox(
                    fields,
                    textvariable=variable,
                    values=("left", "right"),
                    state="readonly",
                    width=8,
                )
                if column == 1
                else ttk.Entry(fields, textvariable=variable)
            )
            widget.grid(row=1, column=column, sticky="ew", padx=(0, 12), pady=(4, 12))
            self.cup_fields.append(widget)
        ttk.Label(fields, text="Pace", style="Card.TLabel").grid(row=2, column=0, sticky="w")
        self.cup_condition_field = ttk.Combobox(
            fields,
            textvariable=self.cup_condition,
            values=("natural", "slow", "fast"),
            state="readonly",
            width=14,
        )
        self.cup_condition_field.grid(row=3, column=0, sticky="w", pady=(4, 10))
        ttk.Label(
            fields,
            text="1: hand start  →  2: grasp cup  →  3: place + release  →  4: hand finish",
            style="CardText.TLabel",
            wraplength=620,
        ).grid(row=2, column=1, rowspan=2, columnspan=2, sticky="w")
        prepare = ttk.Frame(card, style="Card.TFrame")
        prepare.grid(row=3, column=0, sticky="w")
        self.cup_connect_button = ttk.Button(
            prepare, text="1. Connect camera + BLE", command=self.prepare_cup_recording
        )
        self.cup_connect_button.pack(side="left", padx=(0, 8))
        self.cup_calibration_buttons = []
        for label, action in (
            ("2. Hold still at 1: align IMUs", "imu"),
            ("3. Relaxed EMG", "rest"),
            ("4. Firm grip (3 s)", "grip"),
        ):
            button = ttk.Button(
                prepare, text=label, command=lambda value=action: self._cup_command(value)
            )
            button.pack(side="left", padx=(0, 8))
            self.cup_calibration_buttons.append(button)
        ttk.Label(card, textvariable=self.cup_status, style="Card.TLabel", wraplength=820).grid(
            row=4, column=0, sticky="w", pady=12
        )
        actions = ttk.Frame(card, style="Card.TFrame")
        actions.grid(row=5, column=0, sticky="w")
        self.cup_record_button = ttk.Button(
            actions,
            text="Record next trial",
            style="Accent.TButton",
            command=lambda: self._cup_command("record"),
        )
        self.cup_record_button.pack(side="left", padx=(0, 8))
        self.cup_stop_button = ttk.Button(
            actions, text="Stop and save", command=lambda: self._cup_command("stop")
        )
        self.cup_stop_button.pack(side="left", padx=(0, 8))
        self.cup_finish_button = ttk.Button(
            actions, text="Finish subject", command=lambda: self._cup_command("close")
        )
        self.cup_finish_button.pack(side="left", padx=(0, 8))
        self.cup_graphs_button = ttk.Button(
            actions, text="Signals", command=lambda: self._cup_command("graphs")
        )
        self.cup_graphs_button.pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="Saved recordings", command=self._open_recordings_root).pack(
            side="left"
        )
        ttk.Label(
            card,
            text="Trial numbers advance automatically. Finish the subject before entering the next ID.",
            style="CardText.TLabel",
            wraplength=820,
        ).grid(row=6, column=0, sticky="w", pady=(10, 0))
        markers = ttk.Frame(card, style="Card.TFrame")
        markers.grid(row=7, column=0, sticky="w", pady=(8, 0))
        self.cup_marker_buttons = []
        for number, label in ((2, "Mark 2: grasp"), (3, "Mark 3: release"), (4, "Mark 4 + save")):
            button = ttk.Button(
                markers, text=label, command=lambda n=number: self._cup_command("marker", marker=n)
            )
            button.pack(side="left", padx=(0, 8))
            self.cup_marker_buttons.append(button)
        ttk.Label(
            card,
            text="Markers are optional operator timestamps. Press Stop and save if you did not mark them.",
            style="CardText.TLabel",
            wraplength=820,
        ).grid(row=8, column=0, sticky="w", pady=(6, 0))

    def prepare_cup_recording(self) -> None:
        if self._cup_camera_handoff_pending:
            return
        camera_preview = self.processes.get("preview:record_oak_pose.py")
        active = self._active_processes()
        if any(process is not camera_preview for process in active):
            messagebox.showinfo(
                "Programs running", "Stop the active program before preparing a subject."
            )
            return
        if camera_preview is not None and camera_preview.process.poll() is None:
            self._cup_camera_handoff_pending = True
            self.cup_status.set("Closing the camera preview before recording...")
            self._stop_processes([camera_preview])
            deadline = time.monotonic() + 15

            def finish_camera_preview():
                if camera_preview.process.poll() is None:
                    if time.monotonic() < deadline:
                        self.after(100, finish_camera_preview)
                        return
                    self._cup_camera_handoff_pending = False
                    self.cup_status.set("The camera preview did not close. Stop it and retry.")
                    self._update_controls()
                    return
                self._cup_camera_handoff_pending = False
                self.prepare_cup_recording()

            self.after(100, finish_camera_preview)
            self._update_controls()
            return
        subject = self.recording_subject.get().strip()
        try:
            from aurora.recording.common import safe_label

            if not subject or safe_label(subject) != subject:
                raise ValueError("Use a subject ID such as S01, with letters, numbers, - or _.")
            duration = float(self.cup_duration.get())
            if not math.isfinite(duration) or duration < 0:
                raise ValueError("Trial duration must be 0 or more seconds.")
            output = project_path(self.recording_output.get().strip())
            if not self.recording_output.get().strip():
                raise ValueError("Choose an output folder in Advanced recording options.")
        except ValueError as error:
            messagebox.showerror("Recording settings", str(error))
            return
        if self._release_ble_for(self.prepare_cup_recording):
            return
        self.recording_advanced.set(False)
        self.recording_advanced_panel.grid_remove()
        python = self.simulation_python or console_python(Path(sys.executable))
        command = [
            str(python),
            "-u",
            str(PACKAGE_DIRECTORY / "recording" / "record_cup_trials.py"),
            "--subject-id",
            subject,
            "--side",
            self.recording_camera_side.get(),
            "--device",
            self.recording_ble_device.get(),
            "--output",
            str(output.resolve()),
            "--duration",
            str(duration),
            "--shoulder-twist-axis",
            self.fusion_shoulder_twist_axis.get(),
            "--wrist-twist-axis",
            self.fusion_wrist_twist_axis.get(),
        ]
        preview_x = min(
            self.window.winfo_rootx() + self.window.winfo_width() + 12,
            max(0, self.window.winfo_screenwidth() - 500),
        )
        preview_y = max(0, min(self.window.winfo_rooty(), self.window.winfo_screenheight() - 420))
        command.extend(("--preview-position", str(preview_x), str(preview_y)))
        self._cup_ready = self._cup_recording = False
        self.cup_status.set("Connecting camera and BLE...")
        self._start_program("Cup recording", "recording", command, process_key=CUP_PROCESS_KEY)

    def _cup_command(self, action: str, **values) -> None:
        if action == "marker":
            values["host_monotonic_ns"] = time.monotonic_ns()
        if action == "record":
            values["condition"] = self.cup_condition.get()
        if not self._send_process_input(
            CUP_PROCESS_KEY, json.dumps({"action": action, **values}) + "\n"
        ):
            self.cup_status.set("Connect camera + BLE first.")

    def _handle_cup_event(self, line: str) -> None:
        try:
            data = json.loads(line.removeprefix("[CUP] "))
        except ValueError:
            return
        event = data.get("event")
        if event == "status":
            self._cup_ready = data["ready"]
            self._cup_recording = data["recording"]
            self._cup_calibration_stage = data.get("calibration_stage", "connect")
            self._cup_calibration_busy = data.get("calibration_busy", False)
            self.cup_next_trial.set(data["next_trial"])
            self.cup_status.set(data["message"])
        elif event == "recording":
            self._cup_recording = True
            self.cup_status.set(
                f"Recording {data['trial']} ({data.get('condition', 'natural')}). Hand 1 → cup 2 → release at 3 → hand 4."
            )
        elif event == "saved":
            self._cup_recording = False
            self.cup_status.set(
                "Trial saved." if data["complete"] else "Trial saved as incomplete. Check Activity."
            )
            self._refresh_recordings()
        elif event in ("cue", "error"):
            self.cup_status.set(data["message"])
        self._update_cup_controls()

    def _update_cup_controls(self) -> None:
        if not hasattr(self, "cup_connect_button"):
            return
        managed = self.processes.get(CUP_PROCESS_KEY)
        connected = managed is not None and managed.process.poll() is None
        blocked = (
            any(item.key != "preview:record_oak_pose.py" for item in self._active_processes())
            or self._ble_handoff_pending
            or self._cup_camera_handoff_pending
        )
        self.cup_connect_button.configure(state="disabled" if blocked else "normal")
        for index, widget in enumerate(self.cup_fields):
            widget.configure(
                state="disabled" if connected else "readonly" if index == 1 else "normal"
            )
        for stage, button in zip(
            ("imu", "rest", "grip"), self.cup_calibration_buttons, strict=True
        ):
            button.configure(
                state="normal"
                if connected
                and not self._cup_recording
                and self._cup_calibration_stage == stage
                and not self._cup_calibration_busy
                else "disabled"
            )
        self.cup_condition_field.configure(state="disabled" if self._cup_recording else "readonly")
        for button in self.cup_marker_buttons:
            button.configure(state="normal" if self._cup_recording else "disabled")
        self.cup_record_button.configure(
            state="normal"
            if connected and self._cup_ready and not self._cup_recording
            else "disabled",
            text=f"Record {self.cup_next_trial.get()}",
        )
        self.cup_stop_button.configure(
            state="normal" if connected and self._cup_recording else "disabled"
        )
        self.cup_finish_button.configure(state="normal" if connected else "disabled")
        self.cup_graphs_button.configure(state="normal" if connected else "disabled")
        self.recording_advanced_button.configure(state="disabled" if connected else "normal")
