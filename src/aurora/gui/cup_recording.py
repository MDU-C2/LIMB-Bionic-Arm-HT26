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
from aurora.recording.cup_protocol import PACE_PROMPTS, START_POSE, TRIAL_SECONDS

CUP_PROCESS_KEY = "recording:cup_trials"


class CupRecordingMixin:
    def _initialize_cup_recording_state(self) -> None:
        self.cup_duration = tk.StringVar(value=str(int(TRIAL_SECONDS)))
        self.cup_condition = tk.StringVar(value="natural")
        self._cup_streams_ready = False
        self._cup_can_start = False
        self._cup_can_prepare = False
        self._cup_can_record = False
        self._cup_protocol_phase = "ready"
        self.cup_cue = tk.StringVar(value="PREPARE")
        self.cup_instruction = tk.StringVar(value=START_POSE)
        self.cup_progress = tk.StringVar(value="Saved trials: natural 0 | slow 0 | fast 0")
        self.cup_status = tk.StringVar(
            value="Connect camera + BLE, prepare the subject, then choose the trial pace."
        )
        self.cup_next_trial = tk.StringVar(value="T001")
        self._cup_ready = False
        self._cup_recording = False
        self._cup_camera_handoff_pending = False
        self.cup_condition.trace_add("write", self._cup_pace_changed)

    def _cup_pace_changed(self, *_args):
        if self._cup_protocol_phase not in ("ready", "rest"):
            return
        self.cup_instruction.set(START_POSE + " " + PACE_PROMPTS[self.cup_condition.get()])
        managed = self.processes.get(CUP_PROCESS_KEY)
        if managed is not None and managed.process.poll() is None:
            self._cup_command("pace", condition=self.cup_condition.get())

    def _build_cup_recording(self, tab) -> None:
        card = self._card(
            tab,
            2,
            "Cup transfers",
            "Subject sits upright, open hand at 1 and cup at 2. "
            "Keep shoulders, arm, hand and cup path in view. Hips may be behind the table.",
        )
        fields = ttk.Frame(card, style="Card.TFrame")
        fields.grid(row=2, column=0, sticky="ew")
        self.cup_fields = []
        for column, (label, variable) in enumerate(
            (
                ("Subject", self.recording_subject),
                ("Arm", self.recording_camera_side),
                ("Recording window [s] (0 = manual)", self.cup_duration),
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
            if column < 2:
                self.workflow.bind_context(widget)
            self.cup_fields.append(widget)
        fields.columnconfigure(3, weight=1)
        ttk.Label(fields, text="Pace", style="Card.TLabel").grid(row=0, column=3, sticky="w")
        self.cup_condition_field = ttk.Combobox(
            fields,
            textvariable=self.cup_condition,
            values=("natural", "slow", "fast"),
            state="readonly",
            width=14,
        )
        self.cup_condition_field.grid(row=1, column=3, sticky="ew", pady=(4, 12))
        ttk.Label(
            fields,
            text="Stereo depth is used for subject calibration and cup trials.",
            style="CardText.TLabel",
        ).grid(row=2, column=0, columnspan=4, sticky="w", pady=(0, 8))
        prepare = ttk.Frame(card, style="Card.TFrame")
        prepare.grid(row=3, column=0, sticky="w")
        self.cup_connect_button = ttk.Button(
            prepare, text="1. Connect camera + BLE", command=self.prepare_cup_recording
        )
        self.cup_connect_button.pack(side="left", padx=(0, 8))
        self.cup_reference_button = ttk.Button(
            prepare,
            text="2. Calibrate subject",
            command=lambda: self._cup_command("reference"),
        )
        self.cup_reference_button.pack(side="left", padx=8)
        self.cup_reference_step_button = ttk.Button(
            prepare,
            text="Start step",
            command=lambda: self._cup_command("reference_step"),
        )
        self.cup_reference_step_button.pack(side="left", padx=8)
        cue = ttk.Frame(card, style="Card.TFrame")
        cue.grid(row=4, column=0, sticky="ew", pady=(10, 0))
        cue.columnconfigure(0, weight=1)
        ttk.Label(
            cue, textvariable=self.cup_cue, style="Card.TLabel", font=("Segoe UI", 20, "bold")
        ).grid(row=0, column=0, sticky="w")
        instruction = ttk.Label(
            cue,
            textvariable=self.cup_instruction,
            style="Card.TLabel",
            wraplength=620,
            font=("Segoe UI", 12),
        )
        instruction.grid(row=1, column=0, sticky="ew", pady=(4, 0))
        cue.bind(
            "<Configure>",
            lambda event: instruction.configure(wraplength=max(200, event.width - 12)),
        )
        ttk.Label(card, textvariable=self.cup_status, style="CardText.TLabel", wraplength=820).grid(
            row=5, column=0, sticky="w", pady=8
        )
        actions = ttk.Frame(card, style="Card.TFrame")
        actions.grid(row=6, column=0, sticky="w")
        self.cup_record_button = ttk.Button(
            actions,
            text="Record next trial",
            style="Accent.TButton",
            command=lambda: self._cup_command("record"),
        )
        self.cup_record_button.grid(row=0, column=0, padx=(0, 8), pady=3, sticky="w")
        self.cup_stop_button = ttk.Button(
            actions, text="Stop and save", command=lambda: self._cup_command("stop")
        )
        self.cup_stop_button.grid(row=0, column=1, padx=(0, 8), pady=3, sticky="w")
        self.cup_retry_button = ttk.Button(
            actions, text="Restart / repeat", command=lambda: self._cup_command("retry")
        )
        self.cup_retry_button.grid(row=0, column=2, padx=(0, 8), sticky="w")
        self.cup_practice_button = ttk.Button(
            actions, text="Practice (no files)", command=lambda: self._cup_command("practice")
        )
        self.cup_practice_button.grid(row=1, column=0, padx=(0, 8), pady=3, sticky="w")
        self.cup_finish_button = ttk.Button(
            actions, text="Finish subject", command=lambda: self._cup_command("close")
        )
        self.cup_finish_button.grid(row=1, column=2, padx=(0, 8), sticky="w")
        self.cup_graphs_button = ttk.Button(
            actions, text="Signals", command=lambda: self._cup_command("graphs")
        )
        self.cup_graphs_button.grid(row=1, column=1, padx=(0, 8), sticky="w")
        ttk.Button(actions, text="Saved recordings", command=self.workflow.review_recordings).grid(
            row=1, column=3, sticky="w"
        )
        ttk.Label(
            card,
            textvariable=self.cup_progress,
            style="CardText.TLabel",
            wraplength=820,
        ).grid(row=7, column=0, sticky="w", pady=(8, 0))
        ttk.Label(
            card,
            text="Operator signals the start at GO. The recording window saves automatically. "
            "Observe 1 -> 2 -> 3 -> 4; review movement times afterwards. "
            "Repeat keeps the earlier attempt. The subject does not need to see the screen.",
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
                raise ValueError("Choose the cup-trial output folder under Record.")
        except ValueError as error:
            messagebox.showerror("Recording settings", str(error))
            return
        if self._release_ble_for(self.prepare_cup_recording):
            return
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
        command.append("--depth")
        self._cup_ready = self._cup_recording = False
        self._cup_streams_ready = False
        self._cup_can_start = False
        self._cup_protocol_phase = "ready"
        self._cup_can_prepare = self._cup_can_record = False
        self.cup_status.set("Connecting camera and BLE...")
        self._start_program(
            "Cup recording",
            "recording",
            command,
            process_key=CUP_PROCESS_KEY,
            stop_input='{"action":"connection_closed"}\n',
        )

    def _cup_command(self, action: str, **values) -> None:
        if action in ("record", "practice"):
            values["condition"] = self.cup_condition.get()
            try:
                duration = float(self.cup_duration.get())
                if not math.isfinite(duration) or duration < 0:
                    raise ValueError
            except ValueError:
                self.cup_status.set("Enter a recording window of 0 or more seconds.")
                return
            values["duration"] = duration
        sent = self._send_process_input(
            CUP_PROCESS_KEY, json.dumps({"action": action, **values}) + "\n"
        )
        if not sent:
            self.cup_status.set("Connect camera + BLE first.")
        elif action in ("record", "practice", "reference", "reference_step"):
            self._cup_can_start = False
            self._cup_can_prepare = self._cup_can_record = False
            self._cup_protocol_phase = "arming"
            self.cup_cue.set("PREPARING")
            self._update_cup_controls()

    def _set_cup_protocol(self, data):
        previous_phase = self._cup_protocol_phase
        changed = self.cup_cue.get() != data["cue"]
        reference_step_changed = data.get("reference_step") != getattr(
            self, "_cup_reference_step", None
        )
        self._cup_reference_step = data.get("reference_step")
        self._cup_protocol_phase = data["phase"]
        self.cup_cue.set(data["cue"])
        self.cup_instruction.set(data["instruction"] + " " + data["pace_instruction"])
        if data["practice"]:
            self.cup_instruction.set(self.cup_instruction.get() + " Practice: no files saved.")
        if changed and (
            data["phase"] in ("countdown", "reference_countdown")
            or data["cue"] == "GO"
            or (data["phase"] == "reference" and previous_phase != "reference")
            or (data["phase"] == "reference_prepare" and reference_step_changed)
        ):
            self.bell()

    def _handle_cup_event(self, line: str) -> None:
        try:
            data = json.loads(line.removeprefix("[CUP] "))
        except ValueError:
            return
        event = data.get("event")
        if event not in ("status", "protocol"):
            message = data.get("message")
            if event == "saved":
                message = "Saved " + data["path"]
            if message:
                self._append_log("[Cup recording] " + message + "\n")
        if event == "status":
            self._cup_ready = data["ready"]
            self._cup_recording = data["recording"]
            self._cup_streams_ready = data.get("streams_ready", False)
            self.ble_sensor_connection.set(
                "BLE: cup recorder receiving EMG and both IMUs. Use Signals below."
                if self._cup_streams_ready
                else "BLE: waiting for the cup recorder's sensor streams."
            )
            self._cup_can_start = data.get("can_start", False)
            self._cup_can_prepare = data.get("can_prepare", False)
            self._cup_can_record = data.get("can_record", False)
            if "protocol" in data:
                self._set_cup_protocol(data["protocol"])
            if self._cup_protocol_phase == "ready" and not self._cup_can_record:
                if data.get("calibration_ready"):
                    self.cup_cue.set("CHECK CAMERA VIEW")
                    self.cup_instruction.set(
                        data.get("stereo", {}).get(
                            "message", "Keep both shoulders and the arm in view."
                        )
                    )
                else:
                    self.cup_cue.set(
                        "CALIBRATE SUBJECT" if self._cup_can_prepare else "CHECK CONNECTIONS"
                    )
                    self.cup_instruction.set(
                        "Press 2. Calibrate subject and guide each movement shown here."
                        if self._cup_can_prepare
                        else "Wait for camera, both IMUs, raw EMG and stereo depth. Check the status below."
                    )
            if "saved_counts" in data:
                self.cup_progress.set(
                    "Saved trials: "
                    + " | ".join(f"{pace} {count}" for pace, count in data["saved_counts"].items())
                )
            self.cup_next_trial.set(data["next_trial"])
            self.cup_status.set(data["message"])
        elif event in ("recording", "practice", "reference"):
            self._cup_recording = event != "practice"
            self.cup_status.set(
                data.get(
                    "message", "Preparing. Subject stays at 1; operator signals the start at GO."
                )
            )
        elif event == "protocol":
            self._set_cup_protocol(data)
        elif event == "retry":
            self.cup_condition.set(data["condition"])
            self.cup_status.set(data["message"])
        elif event == "saved":
            self._cup_recording = False
            self.cup_status.set(
                "Calibration saved. Check Signals and start a cup trial."
                if data["complete"] and data.get("reference")
                else "Trial saved."
                if data["complete"]
                else "Capture saved as incomplete. Check Activity."
            )
            checks = data.get("checks")
            if (
                data.get("reference")
                and checks
                and not all(
                    checks.get(key, False)
                    for key in (
                        "imu_zero_ready",
                        "emg_rest_ready",
                        "emg_grip_ready",
                        "sensor_to_arm_ready",
                    )
                )
            ):
                self.cup_status.set(
                    "Calibration needs repeating. "
                    + (
                        checks.get("alignment_message", "")
                        if not checks.get("sensor_to_arm_ready")
                        else checks.get("emg_message", "")
                    )
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
        active = self._cup_protocol_phase in (
            "arming",
            "baseline",
            "countdown",
            "movement",
            "practice",
            "reference",
            "reference_prepare",
            "reference_countdown",
        )
        blocked = (
            any(item.key != "preview:record_oak_pose.py" for item in self._active_processes())
            or self._ble_handoff_pending
            or self._cup_camera_handoff_pending
        )
        self.cup_connect_button.configure(state="disabled" if blocked else "normal")
        if hasattr(self, "cup_reference_button"):
            self.cup_reference_button.configure(
                state="normal" if connected and self._cup_can_prepare and not active else "disabled"
            )
        if hasattr(self, "cup_reference_step_button"):
            self.cup_reference_step_button.configure(
                state="normal"
                if connected and self._cup_protocol_phase == "reference_prepare"
                else "disabled"
            )
        for index, widget in enumerate(self.cup_fields):
            widget.configure(
                state="normal"
                if index == 2 and not active
                else "disabled"
                if connected
                else "readonly"
                if index == 1
                else "normal"
            )
        self.cup_condition_field.configure(state="disabled" if active else "readonly")
        self.cup_record_button.configure(
            state="normal" if connected and self._cup_can_record and not active else "disabled",
            text=f"3. Start {self.cup_next_trial.get()}",
        )
        try:
            timed_capture = float(self.cup_duration.get()) > 0
        except ValueError:
            timed_capture = True  # The operator may be editing the field before connecting.
        self.cup_stop_button.configure(
            state="normal" if connected and active else "disabled",
            text="Cancel reference"
            if self._cup_protocol_phase.startswith("reference")
            else "Cancel countdown"
            if self._cup_protocol_phase in ("arming", "baseline", "countdown")
            else "Finish practice"
            if self._cup_protocol_phase == "practice"
            else "Stop early"
            if timed_capture
            else "Stop and save",
        )
        self.cup_retry_button.configure(
            state="normal" if connected else "disabled",
            text="Restart calibration"
            if self._cup_protocol_phase.startswith("reference")
            else "Restart / repeat",
        )
        self.cup_practice_button.configure(
            state="normal" if connected and self._cup_can_start and not active else "disabled"
        )
        self.cup_finish_button.configure(state="normal" if connected else "disabled")
        self.cup_graphs_button.configure(state="normal" if connected else "disabled")
        self.recording_advanced_button.configure(state="disabled" if connected else "normal")
