# Majd, Christian
"""Portable Tkinter control center for the AURORA project.

The launcher owns shared discovery, process supervision, and activity logging.
Notebook pages are declared in ``TAB_DEFINITIONS`` so tab order and extension
points stay visible in one place. Recording and firmware programs are discovered
from repository conventions instead of machine-specific paths.
"""

from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

from app_layout import AppLayoutMixin
from ble_dashboard import BleDashboardMixin
from process_manager import ProcessManagerMixin
from project_support import (
    ACCENT,
    CONSOLE,
    DEFAULT_REFERENCE_DIRECTORY,
    ERROR,
    INTERACTIVE_SIMULATION_SCRIPT,
    MOVEMENT_MODEL,
    MOVEMENT_RECOGNITION_SCRIPT,
    POSE_RECORDING_SCRIPT,
    RECORDER_DETAILS,
    REPOSITORY_ROOT,
    SUCCESS,
    WARNING,
    FirmwareProject,
    display_path,
    find_simulation_python,
    open_path,
    resolve_firmware_tool,
)
from project_support import (
    has_python_dependencies as _has_python_dependencies,
)
from project_tabs import ProjectTabsMixin
from recording_tab import RecordingTabMixin
from simulation_tabs import SimulationTabsMixin

# Application


class ProjectGui(
    AppLayoutMixin,
    BleDashboardMixin,
    RecordingTabMixin,
    SimulationTabsMixin,
    ProjectTabsMixin,
    ProcessManagerMixin,
    ttk.Frame,
):
    """Main AURORA launcher frame shared by all registered project tabs."""

    def __init__(self, window: tk.Tk) -> None:
        """Initialize shared state, build the interface, and start background refreshes."""
        self.window = window
        self.style = ttk.Style(window)
        self._configure_styles()
        super().__init__(window, style="App.TFrame", padding=(16, 10, 16, 12))

        self._initialize_process_manager()
        self.simulation_python = find_simulation_python()
        self.movement_ai_ready = bool(
            self.simulation_python
            and _has_python_dependencies(self.simulation_python, ("numpy", "onnxruntime"))
        )

        self.dynamic_simulation = tk.BooleanVar(value=False)
        self.save_simulation_torque = tk.BooleanVar(value=True)
        self.fusion_camera_weight = tk.StringVar(value="0.25")
        self.fusion_depth = tk.BooleanVar(value=False)
        self.fusion_grip_source = tk.StringVar(value="Camera")
        self.fusion_shoulder_twist_axis = tk.StringVar(value="y")
        self.fusion_wrist_twist_axis = tk.StringVar(value="y")
        self.motion_recording = tk.StringVar()
        self.motion_references = tk.StringVar(value=str(DEFAULT_REFERENCE_DIRECTORY))
        self.loop_pose_playback = tk.BooleanVar(value=False)
        self.motion_ai_status = tk.StringVar()
        self._initialize_recording_state()
        self._initialize_ble_dashboard()
        self.firmware_project = tk.StringVar()
        self.serial_port = tk.StringVar()
        self.status = tk.StringVar(value="Ready")

        self.firmware_projects: dict[str, FirmwareProject] = {}

        self.grid(row=0, column=0, sticky="nsew")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)

        self._build_header()
        self._build_notebook()
        self._build_activity_panel()

        self._refresh_recording_programs()
        self._refresh_recording_ports()
        self._refresh_recordings()
        self._refresh_firmware()
        self._refresh_info()
        self._update_controls()
        if self.simulation_python is None:
            self._set_status("Simulation environment missing", "warning")

        self._bind_recording_traces()
        self.motion_recording.trace_add("write", lambda *_: self._update_controls())
        self.motion_references.trace_add("write", lambda *_: self._update_controls())
        self.firmware_project.trace_add("write", lambda *_: self._firmware_selection_changed())
        self.serial_port.trace_add("write", lambda *_: self._update_controls())
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.after(100, self._drain_log_queue)
        self.after(100, self._drain_serial_sensor_events)
        self.after(500, self._start_ble_dashboard_if_available)
        self.after(100, self._poll_ble_dashboard)
        self.after(1000, self._poll_serial_ports)

    # Window and tabs

    def _build_activity_panel(self) -> None:
        """Build the shared process log and stop controls."""
        panel = ttk.Frame(self, style="Card.TFrame", padding=(10, 8))
        panel.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        panel.columnconfigure(0, weight=1)

        heading = ttk.Frame(panel, style="Card.TFrame")
        heading.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        heading.columnconfigure(0, weight=1)
        ttk.Label(heading, text="Activity", style="CardTitle.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Button(heading, text="Clear", command=self._clear_log).grid(
            row=0, column=1, padx=(8, 0)
        )
        self.stop_button = ttk.Button(
            heading,
            text="Stop active program",
            style="Danger.TButton",
            command=self.stop_program,
            state="disabled",
        )
        self.stop_button.grid(row=0, column=2, padx=(8, 0))

        self.log = tk.Text(
            panel,
            height=4,
            wrap="word",
            state="disabled",
            bg=CONSOLE,
            fg="#d7e0ef",
            insertbackground="white",
            relief="flat",
            font=("Cascadia Mono", 9),
            padx=10,
            pady=8,
        )
        scrollbar = ttk.Scrollbar(panel, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scrollbar.set)
        self.log.grid(row=1, column=0, sticky="ew")
        scrollbar.grid(row=1, column=1, sticky="ns")
        self._append_log("AURORA Control Center ready.\n")

    # Simulation actions

    def _update_controls(self) -> None:
        """Enable actions only when their files, tools, and state are ready."""
        if not hasattr(self, "interactive_button"):
            return
        active_processes = self._active_processes()
        running = bool(active_processes) or self._ble_handoff_pending
        non_preview_running = self._ble_handoff_pending or any(
            process.kind != "preview" for process in active_processes
        )
        interactive_ready = (
            INTERACTIVE_SIMULATION_SCRIPT.is_file() and self.simulation_python is not None
        )
        fusion_ready = interactive_ready
        self.interactive_button.configure(
            state="disabled" if non_preview_running or not interactive_ready else "normal"
        )
        self.fusion_button.configure(state="disabled" if running or not fusion_ready else "normal")
        camera_preview_running = any(
            process.key == "preview:record_oak_pose.py"
            for process in self._active_processes("preview")
        )
        camera_blocked = any(
            process.kind != "preview" and process.name != "Interactive task simulator"
            for process in active_processes
        )
        self.camera_monitor_button.configure(
            state="disabled" if camera_preview_running or camera_blocked else "normal"
        )
        recording_selected = bool(self.motion_recording.get().strip())
        motion_ai_ready = (
            MOVEMENT_RECOGNITION_SCRIPT.is_file()
            and MOVEMENT_MODEL.is_file()
            and self.movement_ai_ready
        )
        self.analyze_motion_button.configure(
            state="disabled"
            if running or not recording_selected or not motion_ai_ready
            else "normal"
        )
        pose_playback_ready = POSE_RECORDING_SCRIPT.is_file() and self.simulation_python is not None
        self.pose_playback_button.configure(
            state="disabled"
            if running or not recording_selected or not pose_playback_ready
            else "normal"
        )
        selected_sources = [
            filename for filename, variable in self.recording_sources.items() if variable.get()
        ]
        available_recorders = {path.name for path in self.recording_programs.values()}
        recording_ready = bool(selected_sources) and all(
            filename in available_recorders for filename in selected_sources
        )
        self.record_button.configure(
            state="disabled" if running or not recording_ready else "normal"
        )
        self.stop_recordings_button.configure(
            state="normal" if self._active_processes("recording") else "disabled"
        )
        selected_program = self.recording_programs.get(self.recording_program.get())
        selected_preview = (
            selected_program is not None and selected_program.name in RECORDER_DETAILS
        )
        active_preview_keys = {process.key for process in self._active_processes("preview")}
        selected_preview_running = bool(
            selected_program and f"preview:{selected_program.name}" in active_preview_keys
        )
        self.recording_preview_button.configure(
            state="disabled"
            if non_preview_running or selected_preview_running or not selected_preview
            else "normal"
        )
        self.single_record_button.configure(
            state="disabled" if running or selected_program is None else "normal"
        )
        firmware = self.firmware_projects.get(self.firmware_project.get())
        firmware_ready = firmware is not None and resolve_firmware_tool(firmware) is not None
        port_ready = bool(self.serial_port.get().strip())
        self.build_button.configure(state="disabled" if running or not firmware_ready else "normal")
        self.flash_button.configure(
            state="disabled" if running or not firmware_ready or not port_ready else "normal"
        )
        self.monitor_button.configure(
            state="disabled" if running or not firmware_ready or not port_ready else "normal"
        )
        ble_monitor_running = self.ble_reader is not None and self.ble_reader.thread.is_alive()
        self.stop_button.configure(state="normal" if running or ble_monitor_running else "disabled")

    # UI helpers

    def _set_status(self, text: str, state: str) -> None:
        """Set the global status message and its severity color."""
        self.status.set(text)
        colors = {
            "ready": ("#e8f5ee", SUCCESS),
            "running": ("#eef2ff", ACCENT),
            "warning": ("#fff4e5", WARNING),
            "error": ("#ffebe9", ERROR),
        }
        background, foreground = colors.get(state, colors["ready"])
        self.status_label.configure(bg=background, fg=foreground)

    def _display_command(self, command: list[str]) -> str:
        """Return a readable command with repository-relative paths."""
        displayed: list[str] = []
        python_paths = {Path(sys.executable)}
        if self.simulation_python is not None:
            python_paths.add(self.simulation_python)
        for argument in command:
            if Path(argument) in python_paths:
                displayed.append("python")
                continue
            try:
                path = Path(argument)
                if path.is_absolute() and path.exists():
                    displayed.append(display_path(path))
                    continue
            except OSError:
                pass
            displayed.append(argument)
        return subprocess.list2cmdline(displayed)

    def _append_log(self, value: str) -> None:
        """Append portable output while keeping the Activity log bounded."""
        cleaned = value.replace(str(REPOSITORY_ROOT), ".")
        self.log.configure(state="normal")
        self.log.insert("end", cleaned)
        line_count = int(self.log.index("end-1c").split(".")[0])
        if line_count > 2000:
            self.log.delete("1.0", f"{line_count - 1800}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self) -> None:
        """Remove all text from the Activity log."""
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _open_existing(self, path: Path) -> None:
        """Open an existing local file or directory with error reporting."""
        if not path.exists():
            messagebox.showerror("Path not found", f"Path not found:\n{path}")
            return
        try:
            open_path(path)
        except OSError as error:
            messagebox.showerror("Could not open path", str(error))

    def close(self) -> None:
        """Stop active work when allowed, then close the application."""
        if not self._close_processes():
            return
        if self.ble_reader is not None:
            self.ble_reader.stop()
        self._stop_ble_dashboard()
        self._stop_serial_dashboard()
        if self.imu_monitor is not None and self.imu_monitor.exists:
            self.imu_monitor.close()
        self.window.destroy()


# Entry point


def main() -> None:
    """Create the top-level window and run the Tk event loop."""
    # IDEs and double-clicks can start this file with a system Python even when
    # the project environment exists. Switch before Tk is created so the
    # in-process sensor readers have the installed USB and BLE dependencies.
    if (
        any(importlib.util.find_spec(module) is None for module in ("serial", "bleak"))
        and os.environ.get("AURORA_GUI_RELAUNCHED") != "1"
    ):
        simulation_python = find_simulation_python()
        current_python = Path(sys.executable).resolve()
        if simulation_python is not None and simulation_python.resolve() != current_python:
            environment = os.environ.copy()
            environment["AURORA_GUI_RELAUNCHED"] = "1"
            # os.execve() does not preserve quoting reliably when a Microsoft
            # Store Python hands a Windows path containing spaces to another
            # interpreter. subprocess receives an argument list and quotes the
            # script path correctly (including the OneDrive university path).
            try:
                return_code = subprocess.call(
                    [
                        str(simulation_python),
                        str(Path(__file__).resolve()),
                        *sys.argv[1:],
                    ],
                    cwd=REPOSITORY_ROOT,
                    env=environment,
                )
            except OSError:
                # Keep the GUI launchable in the current interpreter. The IMU
                # monitor will show a focused dependency message if needed.
                pass
            else:
                raise SystemExit(return_code)

    window = tk.Tk()
    window.title("AURORA Control Center")
    window.geometry("1000x720")
    window.minsize(800, 620)
    ProjectGui(window)
    window.mainloop()


if __name__ == "__main__":
    main()
