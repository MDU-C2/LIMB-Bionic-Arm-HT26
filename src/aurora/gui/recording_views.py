# Majd, Christian
"""Recording settings widgets."""

from __future__ import annotations

import tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk
from tkinter.scrolledtext import ScrolledText

from aurora.gui.project_support import (
    DOCUMENTATION_FILE,
    REPOSITORY_ROOT,
    display_path,
    project_path,
)
from aurora.gui.recording_settings import BLE_SENSOR_OPTIONS, MAINTAINED_SOURCES


class RecordingViewsMixin:
    """Build the recording settings page."""

    def _build_recording_tab(self, tab: ttk.Frame) -> None:
        """Show cup trials first and keep the other recorders under Advanced."""
        tab.columnconfigure(0, weight=1)
        self._heading(
            tab,
            "Record cup transfers",
            "Connect camera + BLE, calibrate the subject, then record each cup transfer.",
        )
        self._build_cup_recording(tab)
        self.recording_advanced_button = ttk.Button(
            tab,
            text="Raw signals and guided grip trials (Advanced Tools)",
            command=lambda: self.workflow.open_advanced("raw"),
        )
        self.recording_advanced_button.grid(row=3, column=0, sticky="w", pady=(0, 10))
        ttk.Label(tab, text="Choose the cup-trial output folder", style="PageSub.TLabel").grid(
            row=4, column=0, sticky="w"
        )
        output = ttk.Frame(tab, style="Page.TFrame")
        output.grid(row=5, column=0, sticky="ew", pady=6)
        output.columnconfigure(0, weight=1)
        self.workflow.bind_context(ttk.Entry(output, textvariable=self.recording_output)).grid(
            row=0, column=0, sticky="ew"
        )
        self.workflow.bind_context(
            ttk.Button(output, text="Browse", command=self._browse_recording_output)
        ).grid(row=0, column=1, padx=6)

    def _build_advanced_recording_controls(self, tab) -> None:
        source_card = self._card(
            tab,
            2,
            "Sources",
            "Bluetooth and the camera are selected by default.",
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
            ttk.Label(line, text=description, style="Card.TLabel").grid(row=0, column=1, sticky="w")
            self.recording_source_buttons[filename] = button
        ttk.Button(source_card, text="Workflow guide", command=self._open_workflow_guide).grid(
            row=5, column=0, sticky="w", pady=(4, 0)
        )

        settings = self._card(
            tab,
            3,
            "Session settings",
            "Each source saves its own folder under Output.",
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
            entry = ttk.Entry(session_row, textvariable=variable)
            if variable is self.recording_subject:
                self.workflow.bind_context(entry)
            entry.grid(
                row=1, column=column, sticky="ew", padx=(12 if column else 0, 0), pady=(4, 0)
            )
        ttk.Label(session_row, text="Test type", style="Card.TLabel").grid(
            row=0, column=3, sticky="w", padx=(12, 0)
        )
        ttk.Combobox(
            session_row,
            textvariable=self.recording_test_type,
            values=("movement_imu", "camera_imu", "other"),
            state="readonly",
        ).grid(row=1, column=3, sticky="ew", padx=(12, 0), pady=(4, 0))

        device_row = ttk.Frame(settings, style="Card.TFrame")
        device_row.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        device_row.columnconfigure(0, weight=1)
        device_row.columnconfigure(1, weight=1)
        ttk.Label(device_row, text="Camera arm", style="Card.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        self.workflow.bind_context(
            ttk.Combobox(
                device_row,
                textvariable=self.recording_camera_side,
                values=("left", "right"),
                state="readonly",
            )
        ).grid(row=1, column=0, sticky="ew", pady=(4, 0))
        ttk.Button(
            device_row,
            text="Open camera monitor",
            command=self.open_camera_monitor,
        ).grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Label(device_row, text="IMU + EMG ESP32 port / baud", style="Card.TLabel").grid(
            row=0, column=1, sticky="w", padx=(12, 0)
        )
        serial_row = ttk.Frame(device_row, style="Card.TFrame")
        serial_row.grid(row=1, column=1, sticky="ew", padx=(12, 0), pady=(4, 0))
        serial_row.columnconfigure(0, weight=1)
        self.recording_port_box = ttk.Combobox(serial_row, textvariable=self.recording_serial_port)
        self.recording_port_box.grid(row=0, column=0, sticky="ew")
        ttk.Entry(serial_row, textvariable=self.recording_serial_baud, width=9).grid(
            row=0, column=1, padx=(8, 0)
        )
        ttk.Button(serial_row, text="Refresh", command=self._refresh_recording_ports).grid(
            row=0, column=2, padx=(8, 0)
        )
        ttk.Button(serial_row, text="USB monitor", command=self.open_sensor_window).grid(
            row=0, column=3, padx=(8, 0)
        )

        ble_row = ttk.Frame(settings, style="Card.TFrame")
        ble_row.grid(row=4, column=0, sticky="ew", pady=(0, 10))
        ble_row.columnconfigure(0, weight=1)
        ble_row.columnconfigure(1, weight=2)
        ttk.Label(ble_row, text="BLE device name (normally AURORA)", style="Card.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ble_device_row = ttk.Frame(ble_row, style="Card.TFrame")
        ble_device_row.grid(row=1, column=0, sticky="ew", pady=(4, 0))
        ble_device_row.columnconfigure(0, weight=1)
        ttk.Entry(ble_device_row, textvariable=self.recording_ble_device, state="readonly").grid(
            row=0, column=0, sticky="ew"
        )
        ttk.Button(ble_device_row, text="Change", command=self._edit_ble_device).grid(
            row=0, column=1, padx=(8, 0)
        )
        ttk.Label(
            ble_row,
            text="Sensor streams (record both for a complete session)",
            style="Card.TLabel",
        ).grid(row=0, column=1, sticky="w", padx=(12, 0))
        sensor_choices = ttk.Frame(ble_row, style="Card.TFrame")
        sensor_choices.grid(row=1, column=1, sticky="w", padx=(12, 0), pady=(4, 0))
        for column, (name, label) in enumerate(BLE_SENSOR_OPTIONS):
            ttk.Checkbutton(
                sensor_choices,
                text=label,
                variable=self.recording_ble_sensors[name],
                style="Card.TCheckbutton",
            ).grid(row=0, column=column, sticky="w", padx=(0, 12))
        ttk.Button(sensor_choices, text="Live BLE graphs", command=self.open_ble_monitor).grid(
            row=0, column=2, padx=(0, 12)
        )

        grip_row = ttk.Frame(ble_row, style="Card.TFrame")
        grip_row.grid(row=2, column=0, columnspan=2, sticky="w", pady=(10, 0))
        ttk.Checkbutton(
            grip_row,
            text="Guided grip trial",
            variable=self.recording_grip_trial,
            style="Card.TCheckbutton",
        ).pack(side="left")
        ttk.Label(grip_row, text="Repeats", style="Card.TLabel").pack(side="left", padx=(16, 6))
        ttk.Spinbox(
            grip_row, from_=1, to=20, width=4, textvariable=self.recording_grip_repeats
        ).pack(side="left")
        ttk.Label(
            ble_row,
            textvariable=self.recording_grip_status,
            style="Card.TLabel",
            wraplength=800,
        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(6, 0))

        option_row = ttk.Frame(settings, style="Card.TFrame")
        option_row.grid(row=5, column=0, sticky="ew", pady=(0, 10))
        option_row.columnconfigure(0, weight=1)
        ttk.Label(
            option_row,
            text="Camera settings",
            style="Card.TLabel",
        ).grid(row=0, column=0, sticky="w")
        camera = ttk.Frame(option_row, style="Card.TFrame")
        camera.grid(row=1, column=0, sticky="w", pady=4)
        ttk.Checkbutton(camera, text="Stereo depth", variable=self.recording_camera_depth).pack(
            side="left", padx=6
        )
        ttk.Label(camera, text="Pose detail").pack(side="left", padx=6)
        ttk.Combobox(
            camera,
            textvariable=self.recording_pose_model,
            values=("Fast", "Balanced", "Detailed"),
            state="readonly",
            width=12,
        ).pack(side="left")
        legacy = ttk.Frame(option_row, style="Card.TFrame")
        legacy.grid(row=2, column=0, sticky="w", pady=8)
        ttk.Checkbutton(
            legacy,
            text="Legacy labeled BLE windows (operator cues)",
            variable=self.recording_ble_dataset,
        ).pack(side="left")
        ttk.Label(legacy, text="Movement label").pack(side="left", padx=6)
        ttk.Entry(legacy, textvariable=self.recording_movement_label, width=8).pack(side="left")
        ttk.Label(
            option_row,
            text="Raw BLE EMG compatibility: auto = current/timestamped formats; legacy choices require known firmware.",
            wraplength=790,
        ).grid(row=3, column=0, sticky="w")
        ttk.Combobox(
            option_row,
            textvariable=self.recording_emg_format,
            values=("auto", "legacy-fixed", "legacy-prefix"),
            state="readonly",
            width=20,
        ).grid(row=4, column=0, sticky="w", pady=4)

        output_row = ttk.Frame(settings, style="Card.TFrame")
        output_row.grid(row=6, column=0, sticky="ew", pady=(0, 12))
        output_row.columnconfigure(1, weight=1)
        ttk.Label(output_row, text="Output", style="Card.TLabel").grid(row=0, column=0, padx=(0, 8))
        self.workflow.bind_context(ttk.Entry(output_row, textvariable=self.recording_output)).grid(
            row=0, column=1, sticky="ew"
        )
        self.workflow.bind_context(
            ttk.Button(output_row, text="Browse", command=self._browse_recording_output)
        ).grid(row=0, column=2, padx=(8, 0))

        action_row = ttk.Frame(settings, style="Card.TFrame")
        action_row.grid(row=7, column=0, sticky="w")
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
            command=self.workflow.open_capture_output,
        ).pack(side="left")

        advanced = self._card(
            tab,
            4,
            "Individual capture tools",
            "Choose a named workflow. Movement profiles and cup trials open their own guided settings.",
        )
        program_row = ttk.Frame(advanced, style="Card.TFrame")
        program_row.grid(row=2, column=0, sticky="ew")
        program_row.columnconfigure(1, weight=1)
        ttk.Label(program_row, text="Capture", style="Card.TLabel").grid(
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
            program_row, text="Open / start capture", command=self.start_single_recording
        )
        self.single_record_button.grid(row=0, column=4, padx=(8, 0))
        self.recording_program_status = ttk.Label(
            advanced, text="", style="Card.TLabel", wraplength=850, justify="left"
        )
        self.recording_program_status.grid(row=3, column=0, sticky="w", pady=(8, 0))

    def _browse_recording_output(self) -> None:
        current = project_path(self.recording_output.get())
        initial = current if current.is_dir() else REPOSITORY_ROOT
        selected = filedialog.askdirectory(title="Select recording output", initialdir=initial)
        if selected:
            self.recording_output.set(display_path(Path(selected)))
            self.recordings_root.set(self.recording_output.get())
            self._refresh_recordings()

    def _open_workflow_guide(self) -> None:
        guide = DOCUMENTATION_FILE
        try:
            contents = guide.read_text(encoding="utf-8")
        except OSError:
            self._open_existing(guide)
            return
        window = tk.Toplevel(self.window)
        window.title("AURORA workflow guide")
        window.geometry("820x650")
        text = ScrolledText(window, wrap="word", font=("Segoe UI", 11), padx=16, pady=12)
        text.pack(fill="both", expand=True)
        text.insert("1.0", contents)
        text.configure(state="disabled")
