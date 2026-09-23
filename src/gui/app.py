"""Portable Tkinter control center for the AURORA project.

The launcher owns shared discovery, process supervision, and activity logging.
Notebook pages are declared in ``TAB_DEFINITIONS`` so tab order and extension
points stay visible in one place. Recording and firmware programs are discovered
from repository conventions instead of machine-specific paths.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
import sys
import tkinter as tk
from tkinter import messagebox, ttk

from process_manager import ProcessManagerMixin
from project_tabs import ProjectTabsMixin
from project_support import (
    ACCENT,
    ACCENT_ACTIVE,
    BACKGROUND,
    CARD,
    CONSOLE,
    DEFAULT_REFERENCE_DIRECTORY,
    DEFAULT_TRAJECTORY,
    ERROR,
    FirmwareProject,
    INK,
    INTERACTIVE_SIMULATION_SCRIPT,
    MANUAL_SIMULATION_SCRIPT,
    MOVEMENT_MODEL,
    MOVEMENT_RECOGNITION_SCRIPT,
    MUTED,
    POSE_RECORDING_SCRIPT,
    RECORDER_DETAILS,
    REPOSITORY_ROOT,
    SIMULATION_SCRIPT,
    SUCCESS,
    TAB_DEFINITIONS,
    WARNING,
    display_path,
    find_simulation_python,
    has_python_dependencies as _has_python_dependencies,
    open_path,
    project_path,
)
from recording_tab import RecordingTabMixin
from simulation_tabs import SimulationTabsMixin


# Application

class ProjectGui(
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

        self.trajectory_path = tk.StringVar(value=str(DEFAULT_TRAJECTORY))
        self.loop_playback = tk.BooleanVar(value=True)
        self.dynamic_simulation = tk.BooleanVar(value=False)
        self.save_simulation_torque = tk.BooleanVar(value=True)
        self.refit_dmp = tk.BooleanVar(value=False)
        self.motion_recording = tk.StringVar()
        self.motion_references = tk.StringVar(value=str(DEFAULT_REFERENCE_DIRECTORY))
        self.loop_pose_playback = tk.BooleanVar(value=False)
        self.motion_ai_status = tk.StringVar()
        self._initialize_recording_state()
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
        self.after(500, self._start_serial_dashboard_if_available)

    # Window and tabs

    def _configure_styles(self) -> None:
        """Configure the common color, typography, and widget styles."""
        try:
            self.style.theme_use("clam")
        except tk.TclError:
            pass
        self.window.configure(background=BACKGROUND)
        self.style.configure("App.TFrame", background=BACKGROUND)
        self.style.configure("Page.TFrame", background=BACKGROUND)
        self.style.configure("Card.TFrame", background=CARD, relief="solid", borderwidth=1)
        self.style.configure(
            "Header.TLabel", background=BACKGROUND, foreground=INK, font=("Segoe UI", 17, "bold")
        )
        self.style.configure(
            "HeaderSub.TLabel", background=BACKGROUND, foreground=MUTED, font=("Segoe UI", 10)
        )
        self.style.configure(
            "PageTitle.TLabel",
            background=BACKGROUND,
            foreground=INK,
            font=("Segoe UI", 13, "bold"),
        )
        self.style.configure(
            "PageSub.TLabel", background=BACKGROUND, foreground=MUTED, font=("Segoe UI", 10)
        )
        self.style.configure(
            "CardTitle.TLabel", background=CARD, foreground=INK, font=("Segoe UI", 11, "bold")
        )
        self.style.configure(
            "CardText.TLabel", background=CARD, foreground=MUTED, font=("Segoe UI", 9)
        )
        self.style.configure("Card.TLabel", background=CARD, foreground=INK)
        self.style.configure("Card.TCheckbutton", background=CARD, foreground=INK)
        self.style.map("Card.TCheckbutton", background=[("active", CARD)])
        self.style.configure(
            "Accent.TButton", foreground="white", background=ACCENT, padding=(12, 6)
        )
        self.style.map(
            "Accent.TButton",
            background=[("active", ACCENT_ACTIVE), ("disabled", "#a8b5df")],
        )
        self.style.configure("Secondary.TButton", padding=(10, 5))
        self.style.configure("Danger.TButton", foreground=ERROR, padding=(10, 5))
        self.style.configure("TNotebook", background=BACKGROUND, borderwidth=0)
        self.style.configure("TNotebook.Tab", padding=(16, 7), font=("Segoe UI", 10))
        self.style.map(
            "TNotebook.Tab",
            background=[("selected", CARD), ("!selected", "#e5e9f2")],
            foreground=[("selected", INK), ("!selected", MUTED)],
        )
        self.style.configure("Treeview", rowheight=27, font=("Segoe UI", 9))
        self.style.configure("Treeview.Heading", font=("Segoe UI", 9, "bold"))

    def _build_header(self) -> None:
        """Build the application title and global status indicator."""
        header = ttk.Frame(self, style="App.TFrame")
        header.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="AURORA Control Center", style="Header.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            header,
            text="Simulation, data capture, recordings, and robot tools",
            style="HeaderSub.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(2, 0))
        self.status_label = tk.Label(
            header,
            textvariable=self.status,
            bg="#e8f5ee",
            fg=SUCCESS,
            font=("Segoe UI", 9, "bold"),
            padx=12,
            pady=6,
        )
        self.status_label.grid(row=0, column=1, rowspan=2, sticky="e")

    def _build_notebook(self) -> None:
        """Create registered tabs in their declared order."""
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=1, column=0, sticky="nsew")
        self.tabs: dict[str, ttk.Frame] = {}
        for definition in TAB_DEFINITIONS:
            if definition.key in self.tabs:
                raise ValueError(f"Duplicate GUI tab key: {definition.key}")
            builder = getattr(self, definition.builder_name, None)
            if not callable(builder):
                raise ValueError(
                    f"GUI tab {definition.key!r} has no builder "
                    f"{definition.builder_name!r}"
                )
            container, page = self._new_tab(definition.title)
            self.tabs[definition.key] = container
            builder(page)

    def _new_tab(self, title: str) -> tuple[ttk.Frame, ttk.Frame]:
        """Create a scrollable notebook page for normal and high-DPI screens."""
        container = ttk.Frame(self.notebook, style="Page.TFrame")
        container.columnconfigure(0, weight=1)
        container.rowconfigure(0, weight=1)
        canvas = tk.Canvas(container, bg=BACKGROUND, highlightthickness=0)
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        page = ttk.Frame(canvas, style="Page.TFrame", padding=(14, 11))
        page_window = canvas.create_window((0, 0), window=page, anchor="nw")
        page.bind(
            "<Configure>",
            lambda _event: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas.bind(
            "<Configure>",
            lambda event: canvas.itemconfigure(page_window, width=event.width),
        )

        def scroll_page(event) -> None:
            direction = -1 if event.delta > 0 else 1
            canvas.yview_scroll(direction, "units")

        container.bind("<Enter>", lambda _event: canvas.bind_all("<MouseWheel>", scroll_page))
        container.bind("<Leave>", lambda _event: canvas.unbind_all("<MouseWheel>"))
        self.notebook.add(container, text=title)
        return container, page

    def _heading(self, tab: ttk.Frame, title: str, description: str) -> None:
        """Add a standard title and explanatory subtitle to a tab."""
        ttk.Label(tab, text=title, style="PageTitle.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(
            tab,
            text=description,
            style="PageSub.TLabel",
            wraplength=850,
            justify="left",
        ).grid(row=1, column=0, sticky="ew", pady=(2, 10))

    def _card(self, parent: ttk.Frame, row: int, title: str, description: str) -> ttk.Frame:
        """Create a standard bordered content card inside a tab."""
        card = ttk.Frame(parent, style="Card.TFrame", padding=12)
        card.grid(row=row, column=0, sticky="ew", pady=(0, 9))
        card.columnconfigure(0, weight=1)
        ttk.Label(card, text=title, style="CardTitle.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(
            card,
            text=description,
            style="CardText.TLabel",
            wraplength=820,
            justify="left",
        ).grid(row=1, column=0, sticky="ew", pady=(3, 12))
        return card

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
        if not hasattr(self, "playback_button"):
            return
        running = bool(self._active_processes())
        interactive_ready = (
            INTERACTIVE_SIMULATION_SCRIPT.is_file() and self.simulation_python is not None
        )
        simulation_ready = SIMULATION_SCRIPT.is_file() and self.simulation_python is not None
        manual_ready = MANUAL_SIMULATION_SCRIPT.is_file() and self.simulation_python is not None
        self.interactive_button.configure(
            state="disabled" if running or not interactive_ready else "normal"
        )
        self.playback_button.configure(
            state="disabled" if running or not simulation_ready else "normal"
        )
        self.manual_button.configure(
            state="disabled" if running or not manual_ready else "normal"
        )
        recording_selected = bool(self.motion_recording.get().strip())
        motion_ai_ready = (
            MOVEMENT_RECOGNITION_SCRIPT.is_file()
            and MOVEMENT_MODEL.is_file()
            and self.movement_ai_ready
        )
        self.analyze_motion_button.configure(
            state="disabled" if running or not recording_selected or not motion_ai_ready else "normal"
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
        selected_preview = selected_program is not None and selected_program.name in RECORDER_DETAILS
        active_preview_keys = {
            process.key for process in self._active_processes("preview")
        }
        non_preview_running = any(
            process.kind != "preview" for process in self._active_processes()
        )
        selected_preview_running = bool(
            selected_program
            and f"preview:{selected_program.name}" in active_preview_keys
        )
        self.recording_preview_button.configure(
            state="disabled"
            if non_preview_running or selected_preview_running or not selected_preview
            else "normal"
        )
        self.single_record_button.configure(
            state="disabled" if running or selected_program is None else "normal"
        )
        for preview_id, button in self.sensor_preview_buttons.items():
            filename = self.sensor_preview_programs[preview_id]
            preview_script = REPOSITORY_ROOT / "src" / "recording" / filename
            is_ble_window = filename == "record_ble_sensors.py"
            button.configure(
                state="disabled"
                if (
                    non_preview_running
                    or (
                        f"preview:{filename}" in active_preview_keys
                        and not is_ble_window
                    )
                    or not preview_script.is_file()
                )
                else "normal"
            )

        firmware = self.firmware_projects.get(self.firmware_project.get())
        firmware_ready = firmware is not None and shutil.which(firmware.executable) is not None
        port_ready = bool(self.serial_port.get().strip())
        self.build_button.configure(
            state="disabled" if running or not firmware_ready else "normal"
        )
        self.flash_button.configure(
            state="disabled" if running or not firmware_ready or not port_ready else "normal"
        )
        self.monitor_button.configure(
            state="disabled" if running or not firmware_ready or not port_ready else "normal"
        )
        self.stop_button.configure(state="normal" if running else "disabled")

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

    def _open_user_path(self, value: str) -> None:
        """Validate and open a path entered by the user."""
        value = value.strip()
        if not value:
            messagebox.showerror("Path required", "Choose a folder first.")
            return
        self._open_existing(project_path(value))

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
        self._stop_serial_dashboard()
        self.window.destroy()


# Entry point

def main() -> None:
    """Create the top-level window and run the Tk event loop."""
    window = tk.Tk()
    window.title("AURORA Control Center")
    window.geometry("1000x720")
    window.minsize(800, 620)
    ProjectGui(window)
    window.mainloop()


if __name__ == "__main__":
    main()
