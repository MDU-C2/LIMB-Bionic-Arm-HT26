"""Recording tabs and multi-source recording orchestration for AURORA."""

from __future__ import annotations

from datetime import datetime
import os
from pathlib import Path
import shlex
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from project_support import (
    DEFAULT_RECORDINGS_DIRECTORY,
    RECORDER_DETAILS,
    RECORDING_EXTENSIONS,
    REPOSITORY_ROOT,
    console_python,
    discover_recording_programs,
    display_path,
    project_path,
    serial_ports,
)


MAINTAINED_SOURCES = (
    (
        "record_ble_sensors.py",
        "BLE cuff (EMG + IMU + piezo)",
        "Connect to LIMBServer and save every available cuff stream.",
    ),
    (
        "record_oak_pose.py",
        "OAK-D camera",
        "Save video, body/arm pose, and selected-hand landmarks.",
    ),
    (
        "record_serial_sensors.py",
        "Serial sensor stream",
        "Save newline-delimited output from an ESP32 or another device.",
    ),
)

# Each BLE sensor gets its own preview window, but all three windows are served
# by one BLE process and one connection to the cuff.
SENSOR_PREVIEWS = (
    (
        "emg",
        "record_ble_sensors.py",
        "EMG signal",
        "View the cuff's electrical muscle signal and its recent activity level.",
        "Open EMG preview",
        "emg",
    ),
    (
        "imu",
        "record_ble_sensors.py",
        "IMU motion",
        "View acceleration, rotation speed, and gravity-based arm tilt.",
        "Open IMU preview",
        "imu",
    ),
    (
        "piezo",
        "record_ble_sensors.py",
        "Piezo contact and vibration",
        "View the piezoelectric sensor response to contact, bending, or vibration. It is not a calibrated force value.",
        "Open piezo preview",
        "piezo",
    ),
    (
        "oak",
        "record_oak_pose.py",
        "OAK-D camera",
        "View the camera, body and arm pose, and selected-hand landmarks.",
        "Open camera",
        None,
    ),
    (
        "serial",
        "record_serial_sensors.py",
        "Serial sensor stream",
        "View newline-delimited values from an ESP32 or another serial device.",
        "Open live log",
        None,
    ),
)
DEFAULT_BATCH_SOURCES = {"record_ble_sensors.py", "record_oak_pose.py"}


def batch_arguments(
    filename: str,
    camera_arguments: list[str],
    dataset_label: str = "",
) -> list[str]:
    """Return source-specific arguments without leaking options to other recorders."""
    if filename == "record_oak_pose.py":
        return ["--auto-start", *camera_arguments]
    if filename == "record_ble_sensors.py" and dataset_label:
        return ["--dataset-label", dataset_label]
    return []


class RecordingTabMixin:
    """Provide recording configuration, launch actions, and file browsing."""

    def _initialize_recording_state(self) -> None:
        self.recording_program = tk.StringVar()
        self.recording_arguments = tk.StringVar()
        self.recording_output = tk.StringVar(value=str(DEFAULT_RECORDINGS_DIRECTORY))
        self.recording_subject = tk.StringVar(value="S01")
        self.recording_trial_id = tk.StringVar(value="T01")
        self.recording_test_type = tk.StringVar(value="grip_emg")
        self.recording_duration = tk.StringVar(value="0")
        self.recording_camera_side = tk.StringVar(value="left")
        self.recording_ble_device = tk.StringVar(value="LIMBServer")
        self.recording_ble_dataset = tk.BooleanVar(value=False)
        self.recording_movement_label = tk.StringVar(value="1")
        self.recording_serial_port = tk.StringVar()
        self.recording_serial_baud = tk.StringVar(value="115200")
        self.recording_sources = {
            filename: tk.BooleanVar(value=filename in DEFAULT_BATCH_SOURCES)
            for filename, _title, _description in MAINTAINED_SOURCES
        }
        self.sensor_status = tk.StringVar()
        self.recordings_root = self.recording_output
        self.recordings_filter = tk.StringVar()

        self.recording_programs: dict[str, Path] = {}
        self.recording_files: list[Path] = []
        self.recording_metadata: dict[Path, tuple[int, float]] = {}
        self.recording_tree_paths: dict[str, Path] = {}
        self.recordings_scan_error = ""
        self.recordings_scan_truncated = False
        self.recordings_filter_after: str | None = None

    def _bind_recording_traces(self) -> None:
        self.recording_program.trace_add(
            "write", lambda *_: self._recording_selection_changed()
        )
        self.recordings_filter.trace_add(
            "write", lambda *_: self._schedule_recordings_filter()
        )
        for variable in self.recording_sources.values():
            variable.trace_add("write", lambda *_: self._update_controls())

    def _build_recording_tab(self, tab: ttk.Frame) -> None:
        """Build batch recording controls and an advanced single-recorder path."""
        tab.columnconfigure(0, weight=1)
        self._heading(
            tab,
            "Record a synchronized session",
            "Choose one or more sources. All selected programs launch together and share "
            "the same session, subject, and trial identifiers.",
        )

        source_card = self._card(
            tab,
            2,
            "Sources",
            "BLE cuff and OAK-D are selected by default. Camera recording starts "
            "automatically when a batch is launched.",
        )
        self.recording_source_buttons: dict[str, ttk.Checkbutton] = {}
        for row, (filename, title, description) in enumerate(MAINTAINED_SOURCES, start=2):
            line = ttk.Frame(source_card, style="Card.TFrame")
            line.grid(row=row, column=0, sticky="ew", pady=(0, 6))
            line.columnconfigure(1, weight=1)
            button = ttk.Checkbutton(
                line,
                text=title,
                variable=self.recording_sources[filename],
                style="Card.TCheckbutton",
            )
            button.grid(row=0, column=0, sticky="w", padx=(0, 10))
            ttk.Label(line, text=description, style="Card.TLabel").grid(
                row=0, column=1, sticky="w"
            )
            self.recording_source_buttons[filename] = button

        settings = self._card(
            tab,
            3,
            "Session settings",
            "Every source receives the same metadata and writes a separate folder under Output.",
        )

        session_row = ttk.Frame(settings, style="Card.TFrame")
        session_row.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        for column in range(4):
            session_row.columnconfigure(column, weight=1)
        fields = (
            ("Subject", self.recording_subject),
            ("Trial ID", self.recording_trial_id),
            ("Duration (0 = manual)", self.recording_duration),
        )
        for column, (label, variable) in enumerate(fields):
            ttk.Label(session_row, text=label, style="Card.TLabel").grid(
                row=0, column=column, sticky="w", padx=(12 if column else 0, 0)
            )
            ttk.Entry(session_row, textvariable=variable).grid(
                row=1, column=column, sticky="ew", padx=(12 if column else 0, 0), pady=(4, 0)
            )
        ttk.Label(session_row, text="Test type", style="Card.TLabel").grid(
            row=0, column=3, sticky="w", padx=(12, 0)
        )
        ttk.Combobox(
            session_row,
            textvariable=self.recording_test_type,
            values=("grip_emg", "movement_imu", "combined", "other"),
            state="readonly",
        ).grid(row=1, column=3, sticky="ew", padx=(12, 0), pady=(4, 0))

        device_row = ttk.Frame(settings, style="Card.TFrame")
        device_row.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        device_row.columnconfigure(0, weight=1)
        device_row.columnconfigure(1, weight=1)
        device_row.columnconfigure(2, weight=1)
        ttk.Label(device_row, text="BLE device", style="Card.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Entry(device_row, textvariable=self.recording_ble_device).grid(
            row=1, column=0, sticky="ew", pady=(4, 0)
        )
        ttk.Label(device_row, text="Camera arm", style="Card.TLabel").grid(
            row=0, column=1, sticky="w", padx=(12, 0)
        )
        ttk.Combobox(
            device_row,
            textvariable=self.recording_camera_side,
            values=("left", "right"),
            state="readonly",
        ).grid(row=1, column=1, sticky="ew", padx=(12, 0), pady=(4, 0))
        ttk.Label(device_row, text="Serial port / baud", style="Card.TLabel").grid(
            row=0, column=2, sticky="w", padx=(12, 0)
        )
        serial_row = ttk.Frame(device_row, style="Card.TFrame")
        serial_row.grid(row=1, column=2, sticky="ew", padx=(12, 0), pady=(4, 0))
        serial_row.columnconfigure(0, weight=1)
        self.recording_port_box = ttk.Combobox(
            serial_row, textvariable=self.recording_serial_port
        )
        self.recording_port_box.grid(row=0, column=0, sticky="ew")
        ttk.Entry(serial_row, textvariable=self.recording_serial_baud, width=9).grid(
            row=0, column=1, padx=(8, 0)
        )
        ttk.Button(serial_row, text="Refresh", command=self._refresh_recording_ports).grid(
            row=0, column=2, padx=(8, 0)
        )

        option_row = ttk.Frame(settings, style="Card.TFrame")
        option_row.grid(row=4, column=0, sticky="ew", pady=(0, 10))
        option_row.columnconfigure(3, weight=1)
        ttk.Checkbutton(
            option_row,
            text="Labeled BLE capture",
            variable=self.recording_ble_dataset,
            style="Card.TCheckbutton",
        ).grid(row=0, column=0, sticky="w", padx=(0, 8))
        ttk.Label(option_row, text="Label", style="Card.TLabel").grid(row=0, column=1)
        ttk.Entry(option_row, textvariable=self.recording_movement_label, width=8).grid(
            row=0, column=2, padx=(6, 18)
        )
        ttk.Label(
            option_row,
            text="OAK-D options (for example --depth or --pose-model 2)",
            style="Card.TLabel",
        ).grid(row=0, column=3, sticky="w")
        ttk.Entry(option_row, textvariable=self.recording_arguments).grid(
            row=1, column=3, sticky="ew", pady=(4, 0)
        )

        output_row = ttk.Frame(settings, style="Card.TFrame")
        output_row.grid(row=5, column=0, sticky="ew", pady=(0, 12))
        output_row.columnconfigure(1, weight=1)
        ttk.Label(output_row, text="Output", style="Card.TLabel").grid(
            row=0, column=0, padx=(0, 8)
        )
        ttk.Entry(output_row, textvariable=self.recording_output).grid(
            row=0, column=1, sticky="ew"
        )
        ttk.Button(output_row, text="Browse", command=self._browse_recording_output).grid(
            row=0, column=2, padx=(8, 0)
        )

        action_row = ttk.Frame(settings, style="Card.TFrame")
        action_row.grid(row=6, column=0, sticky="w")
        self.record_button = ttk.Button(
            action_row,
            text="Start selected sources",
            style="Accent.TButton",
            command=self.start_selected_recordings,
        )
        self.record_button.pack(side="left", padx=(0, 8))
        self.stop_recordings_button = ttk.Button(
            action_row,
            text="Stop recording sources",
            style="Danger.TButton",
            command=self.stop_recordings,
            state="disabled",
        )
        self.stop_recordings_button.pack(side="left", padx=(0, 8))
        ttk.Button(
            action_row,
            text="Open output folder",
            command=self._open_recordings_root,
        ).pack(side="left")

        advanced = self._card(
            tab,
            4,
            "Advanced single recorder",
            "Run a discovered training or experimental recorder separately. Optional arguments "
            "are passed to this program exactly as entered.",
        )
        program_row = ttk.Frame(advanced, style="Card.TFrame")
        program_row.grid(row=2, column=0, sticky="ew")
        program_row.columnconfigure(1, weight=1)
        ttk.Label(program_row, text="Program", style="Card.TLabel").grid(
            row=0, column=0, padx=(0, 8)
        )
        self.recording_program_box = ttk.Combobox(
            program_row, textvariable=self.recording_program, state="readonly"
        )
        self.recording_program_box.grid(row=0, column=1, sticky="ew")
        ttk.Button(program_row, text="Refresh", command=self._refresh_recording_programs).grid(
            row=0, column=2, padx=(8, 0)
        )
        self.recording_preview_button = ttk.Button(
            program_row, text="Preview", command=self.preview_selected_source
        )
        self.recording_preview_button.grid(row=0, column=3, padx=(8, 0))
        self.single_record_button = ttk.Button(
            program_row, text="Run selected", command=self.start_single_recording
        )
        self.single_record_button.grid(row=0, column=4, padx=(8, 0))
        self.recording_program_status = ttk.Label(
            advanced, text="", style="Card.TLabel", wraplength=850, justify="left"
        )
        self.recording_program_status.grid(row=3, column=0, sticky="w", pady=(8, 0))

    def _build_sensors_tab(self, tab: ttk.Frame) -> None:
        """Show live source previews and route sources to batch recording."""
        tab.columnconfigure(0, weight=1)
        self._heading(
            tab,
            "Sensor tools",
            "Preview one source without saving, or add it to the multi-source Recording tab.",
        )
        self.sensor_preview_buttons: dict[str, ttk.Button] = {}
        self.sensor_preview_programs: dict[str, str] = {}
        for row, (
            preview_id,
            filename,
            title,
            description,
            button_text,
            ble_sensor,
        ) in enumerate(SENSOR_PREVIEWS, start=2):
            card = self._card(tab, row, title, description)
            preview_button = ttk.Button(
                card,
                text=button_text,
                command=lambda selected=filename, sensor=ble_sensor: self.start_sensor_preview(
                    selected, sensor
                ),
            )
            preview_button.grid(row=2, column=0, sticky="w")
            self.sensor_preview_buttons[preview_id] = preview_button
            self.sensor_preview_programs[preview_id] = filename
            ttk.Button(
                card,
                text="Select for recording",
                style="Accent.TButton",
                command=lambda selected=filename: self._select_recorder(selected),
            ).grid(row=2, column=1, sticky="w", padx=(8, 0))

        ttk.Label(
            tab,
            textvariable=self.sensor_status,
            style="PageSub.TLabel",
            wraplength=850,
            justify="left",
        ).grid(row=2 + len(SENSOR_PREVIEWS), column=0, sticky="w")

    def _build_recordings_tab(self, tab: ttk.Frame) -> None:
        """Populate the searchable recording-file browser."""
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(3, weight=1)
        self._heading(
            tab,
            "Browse recordings",
            "Find captured video, sensor data, and saved trajectories without searching folders.",
        )
        toolbar = ttk.Frame(tab, style="Page.TFrame")
        toolbar.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        toolbar.columnconfigure(1, weight=1)
        ttk.Label(toolbar, text="Folder", style="PageSub.TLabel").grid(
            row=0, column=0, padx=(0, 7)
        )
        ttk.Entry(toolbar, textvariable=self.recordings_root).grid(row=0, column=1, sticky="ew")
        ttk.Button(toolbar, text="Browse", command=self._browse_recordings_root).grid(
            row=0, column=2, padx=(8, 0)
        )
        ttk.Button(toolbar, text="Refresh", command=self._refresh_recordings).grid(
            row=0, column=3, padx=(8, 0)
        )
        ttk.Label(toolbar, text="Filter", style="PageSub.TLabel").grid(
            row=1, column=0, padx=(0, 7), pady=(8, 0)
        )
        ttk.Entry(toolbar, textvariable=self.recordings_filter).grid(
            row=1, column=1, columnspan=3, sticky="ew", pady=(8, 0)
        )

        tree_card = ttk.Frame(tab, style="Card.TFrame", padding=8)
        tree_card.grid(row=3, column=0, sticky="nsew")
        tree_card.columnconfigure(0, weight=1)
        tree_card.rowconfigure(0, weight=1)
        self.recordings_tree = ttk.Treeview(
            tree_card, columns=("type", "size", "modified"), selectmode="browse", height=7
        )
        for column, title in (("#0", "File"), ("type", "Type"), ("size", "Size"), ("modified", "Modified")):
            self.recordings_tree.heading(column, text=title)
        self.recordings_tree.column("#0", width=470, minwidth=220)
        self.recordings_tree.column("type", width=90, anchor="center")
        self.recordings_tree.column("size", width=90, anchor="e")
        self.recordings_tree.column("modified", width=145, anchor="center")
        scrollbar = ttk.Scrollbar(tree_card, orient="vertical", command=self.recordings_tree.yview)
        self.recordings_tree.configure(yscrollcommand=scrollbar.set)
        self.recordings_tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.recordings_tree.bind("<Double-1>", lambda _event: self._open_selected_recording())

        bottom = ttk.Frame(tab, style="Page.TFrame")
        bottom.grid(row=4, column=0, sticky="ew", pady=(10, 0))
        bottom.columnconfigure(0, weight=1)
        self.recordings_status = ttk.Label(bottom, text="", style="PageSub.TLabel")
        self.recordings_status.grid(row=0, column=0, sticky="w")
        ttk.Button(bottom, text="Open folder", command=self._open_recordings_root).grid(
            row=0, column=1, padx=(8, 0)
        )
        ttk.Button(
            bottom,
            text="Open selected",
            style="Accent.TButton",
            command=self._open_selected_recording,
        ).grid(row=0, column=2, padx=(8, 0))

    def _recording_selection_changed(self) -> None:
        program = self.recording_programs.get(self.recording_program.get())
        if program is None:
            text = "No recording program is available."
        else:
            text = RECORDER_DETAILS.get(
                program.name,
                "Discovered repository entry point. Check its --help before running it.",
            )
        if hasattr(self, "recording_program_status"):
            self.recording_program_status.configure(text=text)
        self._update_controls()

    def _refresh_recording_ports(self) -> None:
        ports = serial_ports()
        if hasattr(self, "recording_port_box"):
            self.recording_port_box.configure(values=ports)
        if not self.recording_serial_port.get() and ports:
            self.recording_serial_port.set(ports[0])
        self.sensor_status.set(
            f"Detected serial ports: {', '.join(ports)}" if ports else "No serial ports detected."
        )

    def _select_recorder(self, filename: str) -> None:
        variable = self.recording_sources.get(filename)
        if variable is not None:
            variable.set(True)
        for label, program in self.recording_programs.items():
            if program.name == filename:
                self.recording_program.set(label)
                break
        self.notebook.select(self.tabs["recording"])

    def _refresh_recording_programs(self) -> None:
        previous = self.recording_program.get()
        self.recording_programs = discover_recording_programs()
        values = list(self.recording_programs)
        if hasattr(self, "recording_program_box"):
            self.recording_program_box.configure(values=values)
        if previous in self.recording_programs:
            self.recording_program.set(previous)
        elif values:
            self.recording_program.set(values[0])
        else:
            self.recording_program.set("")

        available_names = {path.name for path in self.recording_programs.values()}
        for filename, button in getattr(self, "recording_source_buttons", {}).items():
            button.configure(state="normal" if filename in available_names else "disabled")
            if filename not in available_names:
                self.recording_sources[filename].set(False)
        self._recording_selection_changed()

    def _recording_setup(self, filenames: list[str]) -> tuple[dict[str, str], list[str]] | None:
        output_value = self.recording_output.get().strip()
        if not output_value:
            messagebox.showerror("Output folder required", "Choose a recording output folder.")
            return None
        try:
            duration = float(self.recording_duration.get().strip())
            if duration < 0:
                raise ValueError("Duration cannot be negative.")
            baud = int(self.recording_serial_baud.get().strip() or "115200")
            if baud <= 0:
                raise ValueError("Baud must be positive.")
            output = project_path(output_value)
            output.mkdir(parents=True, exist_ok=True)
            camera_arguments = self._split_arguments(self.recording_arguments.get())
        except (OSError, ValueError) as error:
            messagebox.showerror("Could not prepare recording", str(error))
            return None

        if "record_serial_sensors.py" in filenames and not self.recording_serial_port.get().strip():
            messagebox.showerror("Serial port required", "Choose the sensor serial port first.")
            return None
        if "record_ble_sensors.py" in filenames and not self.recording_ble_device.get().strip():
            messagebox.showerror("BLE device required", "Enter the LIMB BLE device name first.")
            return None
        if "record_ble_sensors.py" in filenames and self.recording_ble_dataset.get():
            if not self.recording_movement_label.get().strip():
                messagebox.showerror("Movement label required", "Enter a movement label first.")
                return None

        batch_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        environment = {
            "AURORA_OUTPUT_DIR": str(output.resolve()),
            "AURORA_SUBJECT": self.recording_subject.get().strip() or "session",
            "AURORA_DURATION_SECONDS": str(duration),
            "AURORA_CAMERA_SIDE": self.recording_camera_side.get(),
            "AURORA_BLE_DEVICE": self.recording_ble_device.get().strip(),
            "AURORA_SERIAL_PORT": self.recording_serial_port.get().strip(),
            "AURORA_SERIAL_BAUD": str(baud),
            "AURORA_TRIAL_ID": self.recording_trial_id.get().strip(),
            "AURORA_TEST_TYPE": self.recording_test_type.get().strip(),
            "AURORA_SESSION_ID": batch_id,
            "AURORA_BATCH_SOURCES": ",".join(filenames),
        }
        return environment, camera_arguments

    def start_selected_recordings(self) -> None:
        """Launch every checked maintained source as one recording batch."""
        filenames = [
            filename for filename, variable in self.recording_sources.items() if variable.get()
        ]
        if not filenames:
            messagebox.showinfo("Choose sources", "Select at least one recording source.")
            return
        if self._active_processes():
            messagebox.showinfo(
                "Programs running", "Stop the active program or recording batch before starting another."
            )
            return
        setup = self._recording_setup(filenames)
        if setup is None:
            return
        environment, camera_arguments = setup
        python = self.simulation_python or console_python(Path(sys.executable))
        launched = 0
        for filename in filenames:
            program = REPOSITORY_ROOT / "src" / "recording" / filename
            if not program.is_file():
                self._append_log(f"[Recording batch] Missing source: {display_path(program)}\n")
                continue
            dataset_label = (
                self.recording_movement_label.get().strip()
                if self.recording_ble_dataset.get() else ""
            )
            arguments = batch_arguments(filename, camera_arguments, dataset_label)
            display_name = next(
                title for candidate, title, _description in MAINTAINED_SOURCES
                if candidate == filename
            )
            if self._start_program(
                display_name,
                "recording",
                [str(python), "-u", str(program), *arguments],
                environment=environment,
                parallel=True,
                process_key=f"recording:{filename}",
            ):
                launched += 1
        if launched:
            self._append_log(
                f"[Recording batch {environment['AURORA_SESSION_ID']}] "
                f"Started {launched} source(s).\n"
            )

    def start_single_recording(self) -> None:
        """Run one discovered recorder, primarily for training/experimental tools."""
        program = self.recording_programs.get(self.recording_program.get())
        if program is None:
            messagebox.showinfo("Recording unavailable", "No recording program was found in src.")
            return
        if self._active_processes():
            messagebox.showinfo("Programs running", "Stop active programs first.")
            return
        setup = self._recording_setup([program.name])
        if setup is None:
            return
        environment, arguments = setup
        if program.name == "record_ble_sensors.py" and self.recording_ble_dataset.get():
            arguments = ["--dataset-label", self.recording_movement_label.get().strip()]
        elif program.name == "record_oak_pose.py":
            arguments = ["--auto-start", *arguments]
        python = self.simulation_python or console_python(Path(sys.executable))
        self._start_program(
            program.stem,
            "recording",
            [str(python), "-u", str(program), *arguments],
            environment=environment,
            parallel=True,
            process_key=f"recording:{program.name}",
        )

    # Backward-compatible action name used by older extensions.
    start_recording = start_single_recording

    def stop_recordings(self) -> None:
        recordings = self._active_processes("recording")
        if recordings and self._confirm_sensitive_stop(recordings):
            self._stop_processes(recordings)

    def preview_selected_source(self) -> None:
        program = self.recording_programs.get(self.recording_program.get())
        if program is None or program.name not in RECORDER_DETAILS:
            messagebox.showinfo("Preview unavailable", "Select a maintained sensor source first.")
            return
        self.start_sensor_preview(program.name)

    def start_sensor_preview(self, filename: str, ble_sensor: str | None = None) -> None:
        if filename not in RECORDER_DETAILS:
            return
        if any(process.kind != "preview" for process in self._active_processes()):
            messagebox.showinfo(
                "Program running",
                "Stop recording, simulation, or firmware tools before opening a preview.",
            )
            return
        program = REPOSITORY_ROOT / "src" / "recording" / filename
        if not program.is_file():
            messagebox.showerror("Preview unavailable", f"Program not found: {program}")
            return
        if filename == "record_serial_sensors.py" and not self.recording_serial_port.get().strip():
            messagebox.showerror("Serial port required", "Choose a serial port in Recording first.")
            self.notebook.select(self.tabs["recording"])
            return
        if filename == "record_ble_sensors.py" and not self.recording_ble_device.get().strip():
            messagebox.showerror("BLE device required", "Enter the BLE device name in Recording first.")
            self.notebook.select(self.tabs["recording"])
            return
        if filename == "record_ble_sensors.py":
            preview_key = f"preview:{filename}"
            existing = self.processes.get(preview_key)
            if existing is not None and existing.process.poll() is None:
                requested_sensor = ble_sensor or "all"
                if self._send_process_input(preview_key, f"show {requested_sensor}\n"):
                    self.sensor_status.set(
                        f"Opened the {requested_sensor.upper()} preview using the existing BLE connection."
                    )
                    return
                messagebox.showerror(
                    "Preview unavailable",
                    "The BLE preview is running but could not open another sensor window.",
                )
                return
        try:
            baud = int(self.recording_serial_baud.get().strip() or "115200")
            if baud <= 0:
                raise ValueError("Baud must be positive.")
        except ValueError as error:
            messagebox.showerror("Invalid serial baud", str(error))
            return
        environment = {
            "AURORA_BLE_DEVICE": self.recording_ble_device.get().strip(),
            "AURORA_SERIAL_PORT": self.recording_serial_port.get().strip(),
            "AURORA_SERIAL_BAUD": str(baud),
            "AURORA_OUTPUT_DIR": str(project_path(self.recording_output.get()).resolve()),
            "AURORA_SUBJECT": self.recording_subject.get().strip() or "session",
            "AURORA_DURATION_SECONDS": self.recording_duration.get().strip() or "0",
            "AURORA_CAMERA_SIDE": self.recording_camera_side.get(),
            "AURORA_TRIAL_ID": self.recording_trial_id.get().strip(),
            "AURORA_TEST_TYPE": self.recording_test_type.get().strip(),
        }
        arguments = self._split_arguments(self.recording_arguments.get()) \
            if filename == "record_oak_pose.py" else []
        if filename == "record_ble_sensors.py":
            arguments = ["--preview-sensor", ble_sensor or "all"]
        elif filename == "record_serial_sensors.py":
            arguments = ["--preview-window"]
        python = self.simulation_python or console_python(Path(sys.executable))
        preview_names = {
            "emg": "EMG preview",
            "imu": "IMU preview",
            "piezo": "Piezo preview",
        }
        self._start_program(
            (
                "OAK-D camera"
                if filename == "record_oak_pose.py"
                else preview_names.get(
                    ble_sensor or "",
                    f"{filename.removeprefix('record_').removesuffix('.py')} preview",
                )
            ),
            "preview",
            [str(python), "-u", str(program), "--preview", *arguments],
            environment=environment,
            parallel=True,
            process_key=f"preview:{filename}",
        )

    @staticmethod
    def _split_arguments(value: str) -> list[str]:
        if not value.strip():
            return []
        parts = shlex.split(value, posix=os.name != "nt")
        if os.name == "nt":
            parts = [
                part[1:-1]
                if len(part) >= 2 and part[0] == part[-1] and part[0] in "\"'"
                else part
                for part in parts
            ]
        return parts

    def _browse_recording_output(self) -> None:
        current = project_path(self.recording_output.get())
        initial = current if current.is_dir() else REPOSITORY_ROOT
        selected = filedialog.askdirectory(title="Select recording output", initialdir=initial)
        if selected:
            self.recording_output.set(display_path(Path(selected)))
            self._refresh_recordings()

    def _browse_recordings_root(self) -> None:
        current = project_path(self.recordings_root.get())
        initial = current if current.is_dir() else REPOSITORY_ROOT
        selected = filedialog.askdirectory(title="Select recordings folder", initialdir=initial)
        if selected:
            self.recordings_root.set(display_path(Path(selected)))
            self._refresh_recordings()

    def _refresh_recordings(self) -> None:
        self.recording_files = []
        self.recording_metadata.clear()
        self.recordings_scan_error = ""
        self.recordings_scan_truncated = False
        root_value = self.recordings_root.get().strip()
        if not root_value:
            self.recordings_scan_error = "Choose a recordings folder"
            self._render_recordings()
            return
        root = project_path(root_value)
        if root.is_dir():
            try:
                for path in root.rglob("*"):
                    try:
                        if not path.is_file() or path.suffix.lower() not in RECORDING_EXTENSIONS:
                            continue
                        stat = path.stat()
                    except OSError:
                        continue
                    self.recording_files.append(path)
                    self.recording_metadata[path] = (stat.st_size, stat.st_mtime)
                    if len(self.recording_files) >= 2000:
                        self.recordings_scan_truncated = True
                        break
                self.recording_files.sort(
                    key=lambda path: self.recording_metadata[path][1], reverse=True
                )
            except OSError as error:
                self.recordings_scan_error = f"Could not read folder: {error}"
        self._render_recordings()

    def _schedule_recordings_filter(self) -> None:
        if self.recordings_filter_after is not None:
            self.after_cancel(self.recordings_filter_after)
        self.recordings_filter_after = self.after(180, self._render_scheduled_recordings)

    def _render_scheduled_recordings(self) -> None:
        self.recordings_filter_after = None
        self._render_recordings()

    def _render_recordings(self) -> None:
        if not hasattr(self, "recordings_tree"):
            return
        self.recordings_tree.delete(*self.recordings_tree.get_children())
        self.recording_tree_paths.clear()
        root_value = self.recordings_root.get().strip()
        root = project_path(root_value) if root_value else None
        filter_text = self.recordings_filter.get().strip().casefold()
        visible = [
            path for path in self.recording_files
            if not filter_text or filter_text in display_path(path).casefold()
        ]
        for path in visible:
            try:
                relative = str(path.relative_to(root)) if root is not None else path.name
                size_bytes, modified_timestamp = self.recording_metadata[path]
                modified = datetime.fromtimestamp(modified_timestamp).strftime("%Y-%m-%d %H:%M")
            except (KeyError, OSError, ValueError):
                continue
            item = self.recordings_tree.insert(
                "", "end", text=relative,
                values=(path.suffix.lstrip(".").upper(), self._format_size(size_bytes), modified),
            )
            self.recording_tree_paths[item] = path
        if self.recordings_scan_error:
            message = self.recordings_scan_error
        elif root is None:
            message = "Choose a recordings folder"
        elif not root.is_dir():
            message = f"Folder does not exist yet: {display_path(root)}"
        elif not self.recording_files:
            message = "No supported recordings found"
        elif self.recordings_scan_truncated:
            message = "Showing the first 2,000 recording files; choose a narrower folder if needed"
        elif filter_text:
            message = f"Showing {len(visible)} of {len(self.recording_files)} files"
        else:
            message = f"{len(visible)} recording file(s)"
        self.recordings_status.configure(text=message)

    @staticmethod
    def _format_size(size: int) -> str:
        value = float(size)
        for unit in ("B", "KB", "MB", "GB"):
            if value < 1024 or unit == "GB":
                return f"{value:.0f} {unit}" if unit == "B" else f"{value:.1f} {unit}"
            value /= 1024
        return f"{size} B"

    def _open_selected_recording(self) -> None:
        selected = self.recordings_tree.selection()
        if not selected:
            messagebox.showinfo("Select a recording", "Choose a file from the list first.")
            return
        path = self.recording_tree_paths.get(selected[0])
        if path is not None:
            self._open_existing(path)

    def _open_recordings_root(self) -> None:
        root_value = self.recordings_root.get().strip()
        if not root_value:
            messagebox.showerror("Folder required", "Choose a recording output folder.")
            return
        root = project_path(root_value)
        if not root.exists():
            try:
                root.mkdir(parents=True)
            except OSError as error:
                messagebox.showerror("Could not create folder", str(error))
                return
        self._open_existing(root)
