"""Connect existing AURORA services to named desktop workflows."""

from __future__ import annotations

import sys
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from uuid import uuid4

from aurora.gui.project_support import console_python, has_python_dependencies, project_path
from aurora.gui.tool_catalog import TOOLS, arguments_for
from aurora.paths import REPOSITORY_ROOT, RESOURCE_DIRECTORY
from aurora.recording.common import STOP_COMMAND

COOPERATIVE_MODULES = frozenset(
    (
        "aurora.gui.tool_jobs",
        "aurora.recording.emg_processing",
        "aurora.recording.capture_movement",
        "aurora.recording.record_ble_sensors",
        "aurora.simulation.interactive",
        "aurora.simulation.joint_sliders",
        "aurora.simulation.trajectory_playback",
        "aurora.simulation.pose_playback",
        "aurora.ml.train_emg",
        "aurora.ml.train_gru",
    )
)


class WorkflowTools:
    def __init__(self, gui):
        self.gui = gui
        self.dialogs = {}
        self.context_widgets = []
        self.last_output = gui.session.last_output
        self.action_status = tk.StringVar(
            gui,
            value="Choose an action. Progress and errors appear in Activity; Stop active program cancels it.",
        )

    def build_connections(self, page):
        gui = self.gui
        page.columnconfigure(0, weight=1)
        gui._heading(
            page,
            "1. Connect and inspect",
            "Inspect the available signals before calibration. Monitors do not create recordings.",
        )
        ttk.Label(page, textvariable=gui.session.notice, wraplength=820).grid(
            row=7, column=0, sticky="w", pady=8
        )
        ttk.Button(page, text="Review current trial", command=self.review_current).grid(
            row=8, column=0, sticky="w"
        )
        context = gui._card(
            page, 2, "Subject and arm", "These settings are shared with recording and live control."
        )
        row = ttk.Frame(context, style="Card.TFrame")
        row.grid(row=2, column=0, sticky="w")
        for label, variable, choices in (
            ("Subject ID", gui.recording_subject, ()),
            ("Human arm", gui.recording_camera_side, ("left", "right")),
        ):
            ttk.Label(row, text=label).pack(side="left", padx=(0, 6))
            widget = (
                ttk.Combobox(row, textvariable=variable, values=choices, state="readonly", width=9)
                if choices
                else ttk.Entry(row, textvariable=variable, width=14)
            )
            widget.pack(side="left", padx=(0, 16))
            self.context_widgets.append((widget, "readonly" if choices else "normal"))
        sensors = gui._card(
            page,
            3,
            "BLE signals",
            "Open raw EMG and IMU graphs. Rest/grip buttons calibrate the signal display; recording and control have their own guided calibration.",
        )
        row = ttk.Frame(sensors, style="Card.TFrame")
        row.grid(row=2, column=0, sticky="ew")
        ttk.Label(row, text="Device name").pack(side="left")
        ttk.Entry(row, textvariable=gui.recording_ble_device, width=22, state="readonly").pack(
            side="left", padx=8
        )
        self.device_change = ttk.Button(row, text="Change device", command=gui._edit_ble_device)
        self.device_change.pack(side="left", padx=4)
        ttk.Button(row, text="Connect / inspect BLE", command=gui.open_ble_monitor).pack(
            side="left", padx=4
        )
        ttk.Button(row, text="Disconnect BLE", command=gui._stop_ble_dashboard).pack(
            side="left", padx=4
        )
        ttk.Label(
            sensors, textvariable=gui.ble_sensor_connection, style="CardText.TLabel", wraplength=800
        ).grid(row=3, column=0, sticky="w", pady=8)
        camera = gui._card(
            page,
            4,
            "Camera view",
            "Check that the selected arm and body reference are visible. These settings affect preview and raw capture; guided cup trials use stereo depth and Balanced pose detail. Close the preview before a recording or live-control program takes the camera.",
        )
        row = ttk.Frame(camera, style="Card.TFrame")
        row.grid(row=2, column=0, sticky="w")
        gui.camera_monitor_button = ttk.Button(
            row, text="Open camera monitor", command=gui.open_camera_monitor
        )
        gui.camera_monitor_button.pack(side="left", padx=(0, 12))
        ttk.Checkbutton(row, text="Stereo depth", variable=gui.recording_camera_depth).pack(
            side="left", padx=8
        )
        ttk.Label(row, text="Pose detail").pack(side="left", padx=4)
        ttk.Combobox(
            row,
            textvariable=gui.recording_pose_model,
            values=("Fast", "Balanced", "Detailed"),
            state="readonly",
            width=12,
        ).pack(side="left")
        usb = gui._card(
            page,
            5,
            "USB signals (optional)",
            "Use when USB streaming is enabled in the firmware. One monitor owns the selected port.",
        )
        row = ttk.Frame(usb, style="Card.TFrame")
        row.grid(row=2, column=0, sticky="w")
        self.usb_ports = ttk.Combobox(row, textvariable=gui.recording_serial_port, width=16)
        self.usb_ports.pack(side="left", padx=(0, 6))
        ttk.Label(row, text="Baud").pack(side="left")
        ttk.Entry(row, textvariable=gui.recording_serial_baud, width=9).pack(side="left", padx=6)
        ttk.Button(row, text="Refresh ports", command=gui._refresh_recording_ports).pack(
            side="left", padx=4
        )
        ttk.Button(row, text="Connect / inspect USB", command=gui.open_sensor_window).pack(
            side="left", padx=4
        )
        ttk.Button(row, text="Disconnect USB", command=gui._stop_serial_dashboard).pack(
            side="left", padx=4
        )
        ttk.Label(
            usb, textvariable=gui.serial_sensor_connection, style="CardText.TLabel", wraplength=800
        ).grid(row=3, column=0, sticky="w", pady=8)
        next_step = gui._card(
            page,
            6,
            "2. Calibrate, then record or control",
            "Anatomical angles describe the human. Relative changes describe movement since zeroing. Robot commands use separate home offsets, joint limits and motor limits. A missing reference is not a measured zero.",
        )
        row = ttk.Frame(next_step, style="Card.TFrame")
        row.grid(row=2, column=0, sticky="w")
        for label, key in (
            ("Calibrate and record cup trials", "recording"),
            ("Calibrate and control the arm", "simulation"),
            ("Review saved signals", "recordings"),
        ):
            ttk.Button(
                row, text=label, command=lambda key=key: gui.notebook.select(gui.tabs[key])
            ).pack(side="left", padx=(0, 8))

    def build_advanced(self, page):
        gui = self.gui
        page.columnconfigure(0, weight=1)
        gui._heading(
            page,
            "Advanced Tools",
            "Optional research, raw capture, trajectory tools, diagnostics and firmware. Everyday cup trials stay under Record.",
        )
        self.advanced_notebook = ttk.Notebook(page)
        self.advanced_notebook.grid(row=2, column=0, sticky="nsew")
        self.pages = {}
        for key, title in (
            ("movement", "Movement profiles"),
            ("emg", "EMG models"),
            ("trajectory", "DMP / manual joints"),
            ("raw", "Raw capture"),
            ("diagnostics", "Diagnostics"),
            ("firmware", "Firmware"),
        ):
            frame = ttk.Frame(self.advanced_notebook, style="Page.TFrame", padding=8)
            frame.columnconfigure(0, weight=1)
            self.advanced_notebook.add(frame, text=title)
            self.pages[key] = frame
        gui._build_motion_ai_tab(self.pages["movement"])
        emg = self.pages["emg"]
        ttk.Label(
            emg,
            text="Review intervals before training; select a model only after inspecting its participant evaluation.",
            wraplength=780,
        ).grid(row=0, column=0, sticky="w", pady=8)
        for row, key in enumerate(("emg_train", "emg_results"), 1):
            ttk.Button(
                emg, text=TOOLS[key].title, command=lambda key=key: self.show_tool(key)
            ).grid(row=row, column=0, sticky="w", pady=8)
        ttk.Button(
            emg, text="Review and select a live EMG model", command=gui._select_emg_model
        ).grid(row=3, column=0, sticky="w", pady=8)
        ttk.Button(emg, text="Use calibrated EMG amplitude", command=gui._use_amplitude_emg).grid(
            row=4, column=0, sticky="w", pady=8
        )
        movement = ttk.Frame(self.pages["movement"], style="Page.TFrame")
        movement.grid(row=5, column=0, sticky="ew", pady=8)
        for key in ("movement_capture", "movement_train", "movement_results"):
            ttk.Button(
                movement, text=TOOLS[key].title, command=lambda key=key: self.show_tool(key)
            ).pack(side="left", padx=4)
        trajectories = self.pages["trajectory"]
        ttk.Label(
            trajectories,
            text="Direct robot previews and fitted trajectories",
            style="PageTitle.TLabel",
        ).grid(row=0, column=0, sticky="w", pady=8)
        for row, key in enumerate(("joints", "dmp_play", "dmp_export", "pose_play"), 1):
            ttk.Button(
                trajectories, text=TOOLS[key].title, command=lambda key=key: self.show_tool(key)
            ).grid(row=row, column=0, sticky="w", pady=6)
        gui._build_advanced_recording_controls(self.pages["raw"])
        gui._build_robot_tab(self.pages["firmware"])
        diagnostics = self.pages["diagnostics"]
        ttk.Label(diagnostics, text="Choose a focused check", style="PageTitle.TLabel").grid(
            row=0, column=0, sticky="w", pady=8
        )
        diagnostic_keys = (
            "serial_control",
            "legacy_ble_preview",
            "ble_check",
            "usb_check",
            "camera_check",
            "emg_check",
            "replay_check",
            "live_check",
            "recorded_profile",
            "fusion_synthetic",
            "fusion_reference",
            "dynamics",
            "benchmark",
            "inference_benchmark",
        )
        self.diagnostic_names = {TOOLS[key].title: key for key in diagnostic_keys}
        self.diagnostic_choice = tk.StringVar(gui, value=TOOLS["replay_check"].title)
        ttk.Combobox(
            diagnostics,
            textvariable=self.diagnostic_choice,
            values=tuple(self.diagnostic_names),
            state="readonly",
            width=60,
        ).grid(row=1, column=0, sticky="w", pady=8)
        ttk.Button(
            diagnostics,
            text="Open check settings",
            command=lambda: self.show_tool(self.diagnostic_names[self.diagnostic_choice.get()]),
        ).grid(row=2, column=0, sticky="w", pady=8)
        ttk.Label(
            page, textvariable=self.action_status, style="PageSub.TLabel", wraplength=820
        ).grid(row=3, column=0, sticky="w", pady=8)
        ttk.Button(page, text="Open last output location", command=self.open_output).grid(
            row=4, column=0, sticky="w"
        )

    def selected_recording(self):
        gui = self.gui
        selection = gui.recordings_tree.selection()
        return (
            gui.recording_tree_paths.get(selection[0]) if selection else gui.session.selected_file
        )

    def accept_result(self, path, kind):
        if not path.exists():
            raise ValueError("The saved output is not available on this drive.")
        self.last_output.set(str(path))
        selected = path
        if path.is_dir():
            selected = next(
                (
                    path / name
                    for name in (
                        "trial.json",
                        "training.json",
                        "calibration.json",
                        "trajectory.json",
                        "emg.filtered.csv",
                        "pose.json",
                        "emg.csv",
                        "imu.csv",
                        "serial.jsonl",
                        "recording.csv",
                    )
                    if (path / name).is_file()
                ),
                path,
            )
        self.gui.session.selected_file = selected
        self.gui.recordings_root.set(str(path if path.is_dir() else path.parent))
        self.gui.recordings_filter.set("")
        if selected.name == "trial.json":
            self.gui.session.selected_trial = selected
        if kind == "reference":
            self.gui.session.calibration_path = selected
        if kind == "movement":
            self.gui.motion_recording.set(str(selected))
        self.action_status.set(
            f"Saved output selected for review: {selected}. Choose plot, processing, replay or export."
        )
        self.gui._refresh_recordings()

    def update_controls(self):
        busy = any(
            process.kind in {"recording", "simulation"} for process in self.gui._active_processes()
        )
        busy = busy or self.gui._ble_handoff_pending or self.gui._cup_camera_handoff_pending
        self.gui.session.context_locked = busy
        self.context_widgets = [
            (widget, normal) for widget, normal in self.context_widgets if widget.winfo_exists()
        ]
        for widget, normal in self.context_widgets:
            widget.configure(state="disabled" if busy else normal)
        if hasattr(self, "device_change"):
            self.device_change.configure(
                state="disabled" if self.gui._active_processes() else "normal"
            )

    def bind_context(self, widget):
        self.context_widgets.append((widget, str(widget.cget("state"))))
        return widget

    def open_capture_output(self):
        path = project_path(self.gui.recording_output.get())
        try:
            path.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            messagebox.showerror("Cannot open output folder", str(error))
            return
        self.gui._open_existing(path)

    def review_recordings(self):
        if self.gui.session.selected_trial:
            return self.review_current()
        self.gui.recordings_root.set(
            str(project_path(self.gui.recording_output.get()) / self.gui.recording_subject.get())
        )
        self.gui._refresh_recordings()
        self.gui.notebook.select(self.gui.tabs["recordings"])

    def review_current(self):
        path = self.gui.session.selected_trial
        if path is None:
            self.action_status.set(
                "Record a trial first, or choose an existing recording under Review & export."
            )
            self.gui.notebook.select(self.gui.tabs["recordings"])
            return
        self.gui.session.selected_file = path
        self.gui.recordings_root.set(str(path.parent))
        self.gui.recordings_filter.set("")
        self.gui._refresh_recordings()
        self.gui.notebook.select(self.gui.tabs["recordings"])

    def review_output(self):
        value = self.last_output.get()
        if not value:
            return self.open_output()
        path = Path(value)
        self.gui.recordings_root.set(str(path if path.is_dir() else path.parent))
        self.gui._refresh_recordings()
        self.gui.notebook.select(self.gui.tabs["recordings"])

    def build_review_actions(self, parent):
        actions = ttk.Frame(parent)
        actions.grid(row=3, column=0, columnspan=4, sticky="ew", pady=10)
        for index, (label, callback) in enumerate(
            (
                ("Replay selected cup trial", self.replay_selection),
                ("Export recording bundle", lambda: self.show_tool("bundle")),
                ("Trial / calibration summary", self.show_trial_summary),
                ("Select live EMG model", self.gui._select_emg_model),
                ("Use calibrated EMG amplitude", self.gui._use_amplitude_emg),
                ("Open last output location", self.open_output),
                ("Browse last outputs in Review", self.review_output),
            )
        ):
            ttk.Button(actions, text=label, command=callback).grid(
                row=index // 3, column=index % 3, sticky="w", padx=4, pady=5
            )
        ttk.Label(parent, textvariable=self.action_status, wraplength=800).grid(
            row=4, column=0, columnspan=4, sticky="w"
        )

    def trial_path(self):
        selected = self.selected_recording()
        if selected is None:
            raise ValueError("Select a saved cup trial or a file inside that trial first.")
        for candidate in (
            selected if selected.name == "trial.json" else selected / "trial.json",
            selected.parent / "trial.json",
            selected.parent.parent / "trial.json",
        ):
            if candidate.is_file():
                return candidate
        raise ValueError("This file is not part of a saved cup trial. Choose its trial manifest.")

    def replay_selection(self):
        try:
            trial = self.trial_path()
        except ValueError as error:
            messagebox.showinfo("Choose a cup trial", str(error))
            return
        self.gui.start_recorded_control(trial=str(trial))

    def show_trial_summary(self):
        from aurora.json_utils import read_json_object

        try:
            trial = self.trial_path()
        except ValueError as error:
            messagebox.showinfo("Choose a cup trial", str(error))
            return

        def load(cancelled):
            manifest = read_json_object(trial)
            return WorkflowTools.trial_summary(manifest)

        def ready(result, error):
            if error:
                messagebox.showerror("Cannot read trial", str(error))
                return
            window = tk.Toplevel(self.gui)
            window.title("AURORA | Trial and calibration")
            ttk.Label(
                window,
                text="Requested pace and operator cues are not verified movement labels.\nAnatomical measurements, relative changes and limited robot commands are different quantities.",
                wraplength=740,
                padding=12,
            ).pack()
            from tkinter.scrolledtext import ScrolledText

            text = ScrolledText(window, width=95, height=25, wrap="word")
            text.pack(fill="both", expand=True, padx=10)
            text.insert("end", result)
            text.configure(state="disabled")
            ttk.Button(
                window,
                text="Open trial folder",
                command=lambda: self.gui._open_existing(trial.parent),
            ).pack(pady=8)

        self.gui.background_tasks.submit("trial_summary", load, ready)

    @staticmethod
    def trial_summary(manifest):
        def yes(value):
            return "yes" if value is True else "no" if value is False else "not recorded"

        calibration = manifest.get("calibration_status", {})
        lines = [
            f"Subject: {manifest.get('subject_id', 'not recorded')} | Human arm: {manifest.get('side', 'not recorded')}",
            f"Trial: {manifest.get('trial_id', 'not recorded')}",
            f"Requested pace: {manifest.get('pace_condition') or 'reference capture / not specified'}",
            f"Capture complete: {yes(manifest.get('complete'))}",
            f"Stop reason: {manifest.get('stop_reason', 'not recorded')}",
            f"Capture duration: {manifest.get('duration_seconds', 'not recorded')} seconds",
            f"Task completion independently verified: {yes(manifest.get('task_completion_verified'))}",
            f"Movement labels: {manifest.get('movement_labels', 'not reviewed')}",
            "",
            "CALIBRATION",
            f"Gyroscope rest bias ready: {yes(calibration.get('imu_bias_ready'))}",
            f"EMG rest floor ready: {yes(calibration.get('emg_rest_ready'))}",
            f"EMG deliberate grip ready: {yes(calibration.get('emg_grip_ready'))}",
            f"Sensor mounting alignment: {manifest.get('sensor_to_arm_alignment', 'not recorded')}",
            f"Reference session: {manifest.get('subject_reference') or 'not recorded'}",
            f"Connection generation: {manifest.get('connection_generation', 'not recorded')}",
            f"Mounting session: {manifest.get('mounting_session_id', 'not recorded')}",
            "",
            "CAPTURED DATA",
        ]
        for field, label in (
            ("camera_source_frame_count", "Acquired RGB frames"),
            ("camera_video_frame_count", "Encoded video frames (may include duplicates)"),
            ("camera_depth_frame_count", "Lossless depth maps"),
            ("camera_observation_frame_count", "Tracking observations"),
        ):
            lines.append(f"{label}: {manifest.get(field, 'not recorded')}")
        for sensor, health in manifest.get("packet_health", {}).items():
            lines.append(
                f"{sensor.upper()}: {health.get('received_packets', 0)} notifications; {health.get('missing_packets', 0)} missing, {health.get('decode_errors', 0)} decode errors, {health.get('sample_time_gaps', 0)} timing gaps"
            )
        lines += [
            f"Raw packet queue overruns: {manifest.get('raw_packet_queue_overruns', 0)}",
            "",
            "OPERATOR CUES (seconds from trial start)",
        ]
        lines += [
            f"{event.get('time_s', '?')} s: {event.get('cue', 'cue')}"
            for event in manifest.get("cue_events", [])[:30]
        ]
        if manifest.get("shutdown_error"):
            lines += ["", "FINALIZATION ERROR", str(manifest["shutdown_error"])]
        lines += [
            "",
            "Export the subject folder to include separate reference captures. Open the trial metadata for full clock fits and calibration provenance.",
        ]
        return "\n".join(lines)

    def open_advanced(self, page):
        self.gui.notebook.select(self.gui.tabs["advanced"])
        self.advanced_notebook.select(self.pages[page])

    def new_output(self, key, kind):
        category = (
            "training"
            if key in {"emg_train", "movement_train"}
            else "exports"
            if key in {"bundle", "dmp_export", "plot", "emg_results", "process_emg"}
            else "diagnostics"
        )
        suffix = {
            "save_zip": ".zip",
            "save_png": ".png",
            "save_json": ".json",
            "save_optional": ".png",
        }.get(kind, "")
        return str(
            (
                REPOSITORY_ROOT / "outputs"
                if category == "training"
                else project_path(self.gui.recording_output.get()) / "derived"
            )
            / category
            / f"{key}_{datetime.now():%Y%m%d_%H%M%S}_{uuid4().hex[:8]}{suffix}"
        )

    def show_tool(self, key, initial=None):
        gui, tool = self.gui, TOOLS[key]
        existing = self.dialogs.get(key)
        if existing is not None and existing.winfo_exists():
            existing.lift()
            return existing
        dialog = tk.Toplevel(gui)
        dialog.title("AURORA | " + tool.title)
        dialog.transient(gui)
        dialog.geometry("710x650")
        dialog.columnconfigure(0, weight=1)
        dialog.rowconfigure(1, weight=1)
        self.dialogs[key] = dialog
        ttk.Label(
            dialog,
            text=tool.title + "\n" + tool.description,
            padding=12,
            wraplength=660,
            justify="left",
        ).grid(row=0, column=0, sticky="ew")
        canvas = tk.Canvas(dialog, highlightthickness=0)
        canvas.grid(row=1, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(dialog, orient="vertical", command=canvas.yview)
        scroll.grid(row=1, column=1, sticky="ns")
        canvas.configure(yscrollcommand=scroll.set)
        dialog.bind(
            "<MouseWheel>", lambda event: canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")
        )
        form = ttk.Frame(canvas, padding=12)
        form.columnconfigure(1, weight=1)
        form_window = canvas.create_window((0, 0), window=form, anchor="nw")
        canvas.bind(
            "<Configure>", lambda event: canvas.itemconfigure(form_window, width=event.width)
        )
        form.bind("<Configure>", lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
        variables = {}
        defaults = dict(initial or {})
        selected = self.selected_recording()
        shared = {
            "subject": gui.recording_subject,
            "side": gui.recording_camera_side,
            "device": gui.recording_ble_device,
            "name": gui.recording_ble_device,
            "port": gui.recording_serial_port,
            "baud": gui.recording_serial_baud,
        }
        for row, setting in enumerate(tool.settings):
            value = defaults.get(setting.key, setting.default)
            if (
                key in {"movement_capture", "movement_train"}
                and setting.key in {"output", "data"}
                and setting.key not in defaults
            ):
                value = str(project_path(gui.recording_output.get()) / "movement")
            if (
                not value
                and setting.kind == "new_directory"
                or (not value and setting.kind in {"save_json", "save_zip", "save_png"})
            ):
                value = self.new_output(key, setting.kind)
            if (
                not value
                and selected
                and setting.key in {"source", "file", "recording", "trial", "training"}
            ):
                value = str(selected)
            ttk.Label(form, text=setting.label, wraplength=260).grid(
                row=row, column=0, sticky="w", pady=5, padx=(0, 10)
            )
            if setting.kind == "multi":
                choices = ttk.Frame(form)
                choices.grid(row=row, column=1, sticky="w")
                variables[setting.key] = {
                    choice: tk.BooleanVar(dialog, value=choice in value)
                    for choice in setting.choices
                }
                for choice, variable in variables[setting.key].items():
                    ttk.Checkbutton(choices, text=choice.upper(), variable=variable).pack(
                        side="left", padx=4
                    )
                continue
            variable = shared.get(setting.key) or (
                tk.BooleanVar(dialog, value=bool(value))
                if setting.kind == "bool"
                else tk.StringVar(dialog, value=value)
            )
            variables[setting.key] = variable
            if setting.kind == "bool":
                widget = ttk.Checkbutton(form, variable=variable)
            elif setting.kind == "choice":
                widget = ttk.Combobox(
                    form, textvariable=variable, values=setting.choices, state="readonly"
                )
            else:
                widget = ttk.Entry(form, textvariable=variable)
            widget.grid(row=row, column=1, sticky="ew", pady=5)
            if setting.key in shared:
                self.bind_context(widget)
                if gui.session.context_locked:
                    widget.configure(state="disabled")
            if setting.kind in {
                "file",
                "file_optional",
                "recording",
                "directory",
                "new_directory",
                "output_directory",
            } or setting.kind.startswith("save_"):
                ttk.Button(
                    form,
                    text="Browse",
                    command=lambda setting=setting, variable=variable: self.browse(
                        setting, variable
                    ),
                ).grid(row=row, column=2, padx=5)
                if setting.kind == "recording":
                    ttk.Button(
                        form,
                        text="Folder",
                        command=lambda variable=variable: self.browse_folder(variable),
                    ).grid(row=row, column=3)
        status = tk.StringVar(dialog, value="Checking required capabilities...")
        ttk.Label(dialog, textvariable=status, wraplength=660, padding=10).grid(
            row=2, column=0, sticky="ew"
        )
        actions = ttk.Frame(dialog, padding=10)
        actions.grid(row=3, column=0, sticky="ew")
        launch = ttk.Button(
            actions,
            text="Run",
            state="disabled",
            command=lambda: self.launch(key, self.form_values(variables), dialog),
        )
        launch.pack(side="left", padx=5)
        probe_key = f"tool-capability:{key}"

        def close():
            gui.background_tasks.cancel(probe_key)
            dialog.destroy()
            self.dialogs.pop(key, None)

        ttk.Button(actions, text="Cancel", command=close).pack(side="left", padx=5)
        dialog.protocol("WM_DELETE_WINDOW", close)
        dialog.tool_variables = variables
        dialog.run_button = launch
        dialog.tool_status = status

        def check():
            launch.configure(state="disabled")
            status.set("Checking required capabilities...")
            requirements = list(tool.requirements)
            models = self.form_values(variables).get("models", ())
            if key == "emg_train" and set(models) & {"ann", "lstm"}:
                requirements += ["torch", "onnx", "onnxruntime"]
            python = console_python(Path(sys.executable))
            resource = self.script_path(tool.module) if tool.script else None

            def ready(result, error):
                if error or not result[0]:
                    status.set(
                        f"Unavailable: needs {', '.join(requirements)}. See Setup and checks for optional packages."
                        + (f" {error}" if error else "")
                    )
                elif not result[1]:
                    status.set(
                        "Unavailable: the installed application is missing this diagnostic resource. Run Setup or reinstall the package."
                    )
                else:
                    launch.configure(state="normal")
                    status.set(
                        "Ready. Choose inputs and a new output. Progress/errors appear in Activity; Stop active program cancels the running action."
                    )

            gui.background_tasks.submit(
                probe_key,
                lambda cancelled: (
                    has_python_dependencies(python, tuple(requirements)),
                    resource is None or resource.is_file(),
                ),
                ready,
            )

        if key == "emg_train":
            for variable in variables["models"].values():
                variable.trace_add("write", lambda *_: check())

        def recheck():
            has_python_dependencies.cache_clear()
            check()

        ttk.Button(actions, text="Recheck capabilities", command=recheck).pack(side="left", padx=5)
        check()
        return dialog

    @staticmethod
    def form_values(variables):
        return {
            key: [choice for choice, variable in value.items() if variable.get()]
            if isinstance(value, dict)
            else value.get()
            for key, value in variables.items()
        }

    @staticmethod
    def script_path(name):
        return RESOURCE_DIRECTORY / "scripts" / name

    def browse(self, setting, variable):
        if setting.kind in {"directory", "output_directory"}:
            return self.browse_folder(variable)
        if setting.kind == "new_directory":
            parent = filedialog.askdirectory(title="Choose parent for a new output folder")
            if parent:
                variable.set(str(Path(parent) / Path(variable.get()).name))
            return
        if setting.kind.startswith("save_"):
            suffix = {
                "save_zip": ".zip",
                "save_json": ".json",
                "save_jsonl_optional": ".jsonl",
            }.get(setting.kind, ".png")
            selected = filedialog.asksaveasfilename(
                title=setting.label,
                defaultextension=suffix,
                initialfile=Path(variable.get()).name or "export" + suffix,
            )
        else:
            selected = filedialog.askopenfilename(title=setting.label)
        if selected:
            variable.set(selected)

    @staticmethod
    def browse_folder(variable):
        selected = filedialog.askdirectory(title="Choose folder")
        if selected:
            variable.set(selected)

    def launch(self, key, values, dialog=None):
        gui, tool = self.gui, TOOLS[key]
        try:
            if gui._active_processes("recording"):
                if key not in {"plot", "process_emg", "bundle"} or not gui._idle_cup_session():
                    raise ValueError(
                        "Wait for the trial to save before review/export. Finish subject before hardware, training or specialist tools."
                    )
                source = project_path(values.get("source") or values.get("file", "")).resolve()
                trial = gui.session.selected_trial
                derived = (project_path(gui.recording_output.get()) / "derived").resolve()
                if trial is None or not (
                    source.is_relative_to(trial.parent.resolve()) or source.is_relative_to(derived)
                ):
                    raise ValueError(
                        "Choose a saved trial file for review. Finish subject before exporting the whole subject/output folder."
                    )
            if key == "process_emg":
                from aurora.recording.saved_signals import emg_file

                values = {**values, "source": str(emg_file(project_path(values["source"])))}
            arguments = arguments_for(key, values)
            python = console_python(Path(sys.executable))
            command = [str(python), "-u"]
            command += [str(self.script_path(tool.module))] if tool.script else ["-m", tool.module]
            command += arguments
        except (OSError, ValueError, KeyError) as error:
            messagebox.showerror("Check action settings", str(error), parent=dialog or gui.window)
            return False
        kind = (
            "recording"
            if key == "movement_capture"
            else "simulation"
            if tool.hardware or key in {"joints", "dmp_play", "pose_play"}
            else "training"
            if key in {"emg_train", "movement_train"}
            else "analysis"
        )
        started = gui._start_program(
            tool.title,
            kind,
            command,
            parallel=not tool.hardware and kind not in {"recording", "simulation"},
            process_key=f"tool:{key}",
            stop_input=STOP_COMMAND if tool.module in COOPERATIVE_MODULES else None,
            output_path=(values.get("output") or values.get("save"))
            if key != "movement_capture"
            else None,
        )
        if started:
            output = values.get("output") or values.get("save")
            self.last_output.set(str(output) if output else "")
            self.action_status.set(
                tool.title
                + ": started. "
                + (f"Output: {output}. " if output else "")
                + "Progress and errors are in Activity."
            )
            if dialog is not None:
                gui.background_tasks.cancel(f"tool-capability:{key}")
                self.dialogs.pop(key, None)
                dialog.destroy()
        return started

    def open_output(self):
        value = self.last_output.get()
        if not value:
            messagebox.showinfo(
                "No output selected", "This action displays its result in a window or Activity."
            )
            return
        path = Path(value)
        self.gui._open_existing(path if path.is_dir() else path.parent)
