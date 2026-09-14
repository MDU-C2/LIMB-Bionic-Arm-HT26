"""Portable Tkinter control center for the AURORA project.

The launcher owns shared discovery, process supervision, and activity logging.
Notebook pages are declared in ``TAB_DEFINITIONS`` so tab order and extension
points stay visible in one place. Recording and firmware programs are discovered
from repository conventions instead of machine-specific paths.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import queue
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


# --- 1. Repository paths and supported files -------------------------------

GUI_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = GUI_DIR.parents[1]
INTERACTIVE_SIMULATION_SCRIPT = (
    REPOSITORY_ROOT / "src" / "simulation" / "interactive" / "limb_simulator.py"
)
SIMULATION_SCRIPT = REPOSITORY_ROOT / "src" / "simulation" / "sim" / "limb_sim.py"
MANUAL_SIMULATION_SCRIPT = (
    REPOSITORY_ROOT / "src" / "simulation" / "sim" / "manual_sim.py"
)
DEFAULT_TRAJECTORY = Path("examples/simulation/demo")
DEFAULT_RECORDINGS_DIRECTORY = Path("outputs/recordings")
DOCUMENTATION_FILE = REPOSITORY_ROOT / "docs" / "SIMULATION.md"
GUI_DOCUMENTATION_FILE = GUI_DIR / "README.md"

RECORDING_EXTENSIONS = {
    ".avi",
    ".bag",
    ".csv",
    ".flac",
    ".json",
    ".mcap",
    ".mkv",
    ".mov",
    ".mp3",
    ".mp4",
    ".npy",
    ".npz",
    ".wav",
    ".webm",
}
RECORDING_ENTRYPOINT_PREFIXES = ("record", "capture")

# --- 2. Shared visual theme -------------------------------------------------

BACKGROUND = "#f3f5f9"
CARD = "#ffffff"
INK = "#172033"
MUTED = "#667085"
ACCENT = "#3157d5"
ACCENT_ACTIVE = "#2546b8"
SUCCESS = "#18794e"
WARNING = "#a15c00"
ERROR = "#b42318"
CONSOLE = "#111827"


# --- 3. Small data models and the tab extension registry -------------------

@dataclass(frozen=True)
class FirmwareProject:
    """A firmware directory and the command-line tool that operates it."""

    directory: Path
    system: str
    executable: str


@dataclass(frozen=True)
class TabDefinition:
    """Describe one notebook tab without coupling tab order to UI setup."""

    key: str
    title: str
    builder_name: str


# --- GUI extension registry -------------------------------------------------
# Add a future tab here, then implement the named builder method on ProjectGui.
# Keeping this list declarative makes tab order and available areas easy to scan.
TAB_DEFINITIONS = (
    TabDefinition("simulation", "Simulation", "_build_simulation_tab"),
    TabDefinition("recording", "Recording", "_build_recording_tab"),
    TabDefinition("recordings", "Recordings", "_build_recordings_tab"),
    TabDefinition("robot", "Robot", "_build_robot_tab"),
    TabDefinition("info", "Info", "_build_info_tab"),
)


# --- 4. Portable path and environment discovery ----------------------------

def project_path(value: str | Path) -> Path:
    """Resolve a path entered in the GUI relative to the repository."""
    path = Path(value).expanduser()
    return path if path.is_absolute() else REPOSITORY_ROOT / path


def display_path(path: Path) -> str:
    """Prefer a portable repository-relative path for display."""
    try:
        return str(path.resolve().relative_to(REPOSITORY_ROOT.resolve()))
    except ValueError:
        return str(path)


def open_path(path: Path) -> None:
    """Open a local file or directory with the operating system."""
    if sys.platform == "win32":
        os.startfile(path)  # type: ignore[attr-defined]
    elif sys.platform == "darwin":
        subprocess.Popen(["open", str(path)])
    else:
        subprocess.Popen(["xdg-open", str(path)])


def serial_ports() -> list[str]:
    """Return serial-port names using only the Python standard library."""
    if sys.platform == "win32":
        try:
            import winreg

            key = winreg.OpenKey(
                winreg.HKEY_LOCAL_MACHINE,
                r"HARDWARE\DEVICEMAP\SERIALCOMM",
            )
            ports: list[str] = []
            index = 0
            while True:
                try:
                    _, value, _ = winreg.EnumValue(key, index)
                except OSError:
                    break
                ports.append(str(value))
                index += 1
            winreg.CloseKey(key)
            return sorted(set(ports))
        except OSError:
            return []

    device_root = Path("/dev")
    patterns = ("ttyACM*", "ttyUSB*", "cu.*")
    return sorted(str(path) for pattern in patterns for path in device_root.glob(pattern))


def _console_python(path: Path) -> Path:
    """Prefer python.exe over pythonw.exe so child output can be captured."""
    if path.name.lower() == "pythonw.exe":
        console_python = path.with_name("python.exe")
        if console_python.is_file():
            return console_python
    return path


def _simulation_python_candidates() -> list[Path]:
    """Return likely interpreters without relying on the current clone location."""
    candidates: list[Path] = []

    # Respect an explicit team/user setting before checking common local layouts.
    configured = os.environ.get("AURORA_SIMULATION_PYTHON")
    if configured:
        candidates.append(Path(configured).expanduser())

    candidates.append(_console_python(Path(sys.executable)))
    if os.name == "nt":
        candidates.extend(
            [
                REPOSITORY_ROOT / ".venv" / "Scripts" / "python.exe",
                REPOSITORY_ROOT / "venv" / "Scripts" / "python.exe",
            ]
        )
    else:
        candidates.extend(
            [
                REPOSITORY_ROOT / ".venv" / "bin" / "python",
                REPOSITORY_ROOT / "venv" / "bin" / "python",
            ]
        )

    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        candidates.append(
            Path(conda_prefix) / ("python.exe" if os.name == "nt" else "bin/python")
        )

    environment_roots: list[Path] = []
    mamba_root = os.environ.get("MAMBA_ROOT_PREFIX")
    if mamba_root:
        environment_roots.append(Path(mamba_root))
    local_data = os.environ.get("LOCALAPPDATA")
    if local_data:
        environment_roots.append(Path(local_data) / "micromamba")
    environment_roots.extend(
        [
            Path.home() / ".local" / "share" / "mamba",
            Path.home() / "micromamba",
            Path.home() / "miniconda3",
            Path.home() / "anaconda3",
            Path.home() / ".conda",
        ]
    )
    relative_python = Path("python.exe") if os.name == "nt" else Path("bin/python")
    for root in environment_roots:
        candidates.append(root / "envs" / "aurora-simulation" / relative_python)

    unique: list[Path] = []
    seen: set[str] = set()
    for candidate in candidates:
        normalized = _console_python(candidate).resolve()
        key = os.path.normcase(str(normalized))
        if key not in seen:
            seen.add(key)
            unique.append(normalized)
    return unique


def _has_simulation_dependencies(python: Path) -> bool:
    """Return whether an interpreter imports every simulation dependency."""
    if not python.is_file():
        return False
    creation_flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    try:
        result = subprocess.run(
            [str(python), "-c", "import numpy, scipy, pybullet, pygame"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=12,
            check=False,
            creationflags=creation_flags,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0


def find_simulation_python() -> Path | None:
    """Find a Python interpreter that can run the migrated simulator."""
    for candidate in _simulation_python_candidates():
        if _has_simulation_dependencies(candidate):
            return candidate
    return None


def environment_name(python: Path | None) -> str:
    """Return a concise display name for a Python environment."""
    if python is None:
        return "Not found"
    parent = python.parent
    if parent.name.lower() in {"bin", "scripts"}:
        parent = parent.parent
    return parent.name or python.name


def has_trajectory_data(directory: Path) -> bool:
    """Return whether a folder contains a supported trajectory NPZ."""
    supported_names = (
        "angles.npz",
        "angles_raw.npz",
        "angles_clean.npz",
        "dmp_rollout_raw.npz",
        "dmp_rollout_clean.npz",
    )
    try:
        return any(
            path.is_file() and path.name.endswith(supported_names)
            for path in directory.glob("*.npz")
        )
    except OSError:
        return False


# --- 5. Main application ----------------------------------------------------

class ProjectGui(ttk.Frame):
    """Main AURORA launcher frame shared by all registered project tabs."""

    def __init__(self, window: tk.Tk) -> None:
        """Initialize shared state, build the interface, and start background refreshes."""
        self.window = window
        self.style = ttk.Style(window)
        self._configure_styles()
        super().__init__(window, style="App.TFrame", padding=(16, 10, 16, 12))

        self.process: subprocess.Popen[str] | None = None
        self.process_name = ""
        self.process_kind = ""
        self.stopping = False
        self.log_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self.simulation_python = find_simulation_python()

        self.trajectory_path = tk.StringVar(value=str(DEFAULT_TRAJECTORY))
        self.loop_playback = tk.BooleanVar(value=True)
        self.refit_dmp = tk.BooleanVar(value=False)
        self.recording_program = tk.StringVar()
        self.recording_arguments = tk.StringVar()
        self.recording_output = tk.StringVar(value=str(DEFAULT_RECORDINGS_DIRECTORY))
        self.recordings_root = self.recording_output
        self.recordings_filter = tk.StringVar()
        self.firmware_project = tk.StringVar()
        self.serial_port = tk.StringVar()
        self.status = tk.StringVar(value="Ready")

        self.recording_programs: dict[str, Path] = {}
        self.firmware_projects: dict[str, FirmwareProject] = {}
        self.recording_files: list[Path] = []
        self.recording_metadata: dict[Path, tuple[int, float]] = {}
        self.recording_tree_paths: dict[str, Path] = {}
        self.recordings_scan_error = ""
        self.recordings_scan_truncated = False
        self.recordings_filter_after: str | None = None

        self.grid(row=0, column=0, sticky="nsew")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)

        self._build_header()
        self._build_notebook()
        self._build_activity_panel()

        self._refresh_recording_programs()
        self._refresh_recordings()
        self._refresh_firmware()
        self._refresh_info()
        self._update_controls()
        if self.simulation_python is None:
            self._set_status("Simulation environment missing", "warning")

        self.recording_program.trace_add("write", lambda *_: self._update_controls())
        self.firmware_project.trace_add("write", lambda *_: self._firmware_selection_changed())
        self.serial_port.trace_add("write", lambda *_: self._update_controls())
        self.recordings_filter.trace_add("write", lambda *_: self._schedule_recordings_filter())
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.after(100, self._drain_log_queue)

    # --- Window and tab construction ---------------------------------------

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
            tab = self._new_tab(definition.title)
            self.tabs[definition.key] = tab
            builder(tab)

    def _new_tab(self, title: str) -> ttk.Frame:
        """Create and attach a consistently styled notebook page."""
        tab = ttk.Frame(self.notebook, style="Page.TFrame", padding=(14, 11))
        self.notebook.add(tab, text=title)
        return tab

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
            "Open the LIMB25 table-and-target scene with 7-axis keyboard control, "
            "inverse kinematics, grasping, live joint torque, hand IMU, and simulated EMG.",
        )
        self.interactive_button = ttk.Button(
            interactive,
            text="Open full simulator",
            style="Accent.TButton",
            command=self.start_interactive_simulation,
        )
        self.interactive_button.grid(row=2, column=0, sticky="w")

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

    def _build_recording_tab(self, tab: ttk.Frame) -> None:
        """Populate controls for discovered recording entry points."""
        tab.columnconfigure(0, weight=1)
        self._heading(
            tab,
            "Record a session",
            "Start a recording program from the repository and keep its output "
            "in one session folder.",
        )
        card = self._card(
            tab,
            2,
            "Recording program",
            "Programs named record*.py or capture*.py under src are detected automatically.",
        )

        ttk.Label(card, text="Program", style="Card.TLabel").grid(row=2, column=0, sticky="w")
        program_row = ttk.Frame(card, style="Card.TFrame")
        program_row.grid(row=3, column=0, sticky="ew", pady=(4, 10))
        program_row.columnconfigure(0, weight=1)
        self.recording_program_box = ttk.Combobox(
            program_row,
            textvariable=self.recording_program,
            state="readonly",
        )
        self.recording_program_box.grid(row=0, column=0, sticky="ew")
        ttk.Button(program_row, text="Refresh", command=self._refresh_recording_programs).grid(
            row=0, column=1, padx=(8, 0)
        )

        ttk.Label(card, text="Optional arguments", style="Card.TLabel").grid(
            row=4, column=0, sticky="w"
        )
        ttk.Entry(card, textvariable=self.recording_arguments).grid(
            row=5, column=0, sticky="ew", pady=(4, 10)
        )

        ttk.Label(card, text="Recording output", style="Card.TLabel").grid(
            row=6, column=0, sticky="w"
        )
        output_row = ttk.Frame(card, style="Card.TFrame")
        output_row.grid(row=7, column=0, sticky="ew", pady=(4, 12))
        output_row.columnconfigure(0, weight=1)
        ttk.Entry(output_row, textvariable=self.recording_output).grid(
            row=0, column=0, sticky="ew"
        )
        ttk.Button(output_row, text="Browse", command=self._browse_recording_output).grid(
            row=0, column=1, padx=(8, 0)
        )

        action_row = ttk.Frame(card, style="Card.TFrame")
        action_row.grid(row=8, column=0, sticky="w")
        self.record_button = ttk.Button(
            action_row,
            text="Start recording",
            style="Accent.TButton",
            command=self.start_recording,
        )
        self.record_button.pack(side="left", padx=(0, 8))
        ttk.Button(
            action_row,
            text="Open output folder",
            command=self._open_recordings_root,
        ).pack(side="left")

        self.recording_program_status = ttk.Label(
            tab, text="", style="PageSub.TLabel", wraplength=850, justify="left"
        )
        self.recording_program_status.grid(row=3, column=0, sticky="w")

    def _build_recordings_tab(self, tab: ttk.Frame) -> None:
        """Populate the searchable recording-file browser."""
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(3, weight=1)
        self._heading(
            tab,
            "Browse recordings",
            "Find captured video, audio, sensor data, and saved trajectories "
            "without searching folders.",
        )

        toolbar = ttk.Frame(tab, style="Page.TFrame")
        toolbar.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        toolbar.columnconfigure(1, weight=1)
        ttk.Label(toolbar, text="Folder", style="PageSub.TLabel").grid(
            row=0,
            column=0,
            padx=(0, 7),
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
            tree_card,
            columns=("type", "size", "modified"),
            selectmode="browse",
            height=7,
        )
        self.recordings_tree.heading("#0", text="File")
        self.recordings_tree.heading("type", text="Type")
        self.recordings_tree.heading("size", text="Size")
        self.recordings_tree.heading("modified", text="Modified")
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

    def _build_robot_tab(self, tab: ttk.Frame) -> None:
        """Populate firmware build, flash, and serial-monitor controls."""
        tab.columnconfigure(0, weight=1)
        self._heading(
            tab,
            "Build and flash the robot",
            "Use an ESP-IDF or PlatformIO project under firmware when the robot "
            "software is added.",
        )
        card = self._card(
            tab,
            2,
            "Firmware target",
            "The GUI detects supported projects and serial ports; flashing "
            "always asks for confirmation.",
        )

        ttk.Label(card, text="Project", style="Card.TLabel").grid(row=2, column=0, sticky="w")
        project_row = ttk.Frame(card, style="Card.TFrame")
        project_row.grid(row=3, column=0, sticky="ew", pady=(4, 10))
        project_row.columnconfigure(0, weight=1)
        self.firmware_project_box = ttk.Combobox(
            project_row,
            textvariable=self.firmware_project,
            state="readonly",
        )
        self.firmware_project_box.grid(row=0, column=0, sticky="ew")
        ttk.Button(project_row, text="Refresh", command=self._refresh_firmware).grid(
            row=0, column=1, padx=(8, 0)
        )

        ttk.Label(card, text="Serial port", style="Card.TLabel").grid(
            row=4, column=0, sticky="w"
        )
        self.port_box = ttk.Combobox(card, textvariable=self.serial_port)
        self.port_box.grid(row=5, column=0, sticky="ew", pady=(4, 12))

        robot_actions = ttk.Frame(card, style="Card.TFrame")
        robot_actions.grid(row=6, column=0, sticky="w")
        self.build_button = ttk.Button(
            robot_actions,
            text="Build",
            style="Accent.TButton",
            command=lambda: self._run_firmware("build"),
        )
        self.build_button.pack(side="left", padx=(0, 8))
        self.flash_button = ttk.Button(
            robot_actions, text="Flash", command=lambda: self._run_firmware("flash")
        )
        self.flash_button.pack(side="left", padx=(0, 8))
        self.monitor_button = ttk.Button(
            robot_actions, text="Serial monitor", command=lambda: self._run_firmware("monitor")
        )
        self.monitor_button.pack(side="left")

        self.firmware_status = ttk.Label(
            tab, text="", style="PageSub.TLabel", wraplength=850, justify="left"
        )
        self.firmware_status.grid(row=3, column=0, sticky="w")

    def _build_info_tab(self, tab: ttk.Frame) -> None:
        """Populate project status and documentation shortcuts."""
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(2, weight=1)
        self._heading(
            tab,
            "Project information",
            "Check what is ready on this computer and open the main project documentation.",
        )
        info_card = ttk.Frame(tab, style="Card.TFrame", padding=16)
        info_card.grid(row=2, column=0, sticky="nsew", pady=(0, 12))
        info_card.columnconfigure(0, weight=1)
        info_card.rowconfigure(0, weight=1)
        self.info_text = tk.Text(
            info_card,
            height=16,
            wrap="word",
            state="disabled",
            relief="flat",
            bg=CARD,
            fg=INK,
            font=("Segoe UI", 10),
            padx=4,
            pady=4,
        )
        self.info_text.grid(row=0, column=0, sticky="nsew")

        actions = ttk.Frame(tab, style="Page.TFrame")
        actions.grid(row=3, column=0, sticky="w")
        ttk.Button(actions, text="Refresh status", command=self._refresh_all).pack(
            side="left", padx=(0, 8)
        )
        ttk.Button(
            actions,
            text="Open project README",
            command=lambda: self._open_existing(REPOSITORY_ROOT / "README.md"),
        ).pack(side="left", padx=(0, 8))
        ttk.Button(
            actions,
            text="Open simulation guide",
            command=lambda: self._open_existing(DOCUMENTATION_FILE),
        ).pack(side="left", padx=(0, 8))
        ttk.Button(
            actions,
            text="Open GUI guide",
            command=lambda: self._open_existing(GUI_DOCUMENTATION_FILE),
        ).pack(side="left", padx=(0, 8))
        ttk.Button(
            actions,
            text="Open docs folder",
            command=lambda: self._open_existing(REPOSITORY_ROOT / "docs"),
        ).pack(side="left")

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

    # --- Simulation actions -------------------------------------------------

    def _browse_trajectory(self) -> None:
        """Choose a trajectory directory and store a portable path."""
        current = project_path(self.trajectory_path.get())
        initial = current if current.is_dir() else REPOSITORY_ROOT
        selected = filedialog.askdirectory(title="Select trajectory folder", initialdir=initial)
        if selected:
            self.trajectory_path.set(display_path(Path(selected)))

    def _browse_recording_output(self) -> None:
        """Choose the directory used by recording programs."""
        current = project_path(self.recording_output.get())
        initial = current if current.is_dir() else REPOSITORY_ROOT
        selected = filedialog.askdirectory(title="Select recording output", initialdir=initial)
        if selected:
            self.recording_output.set(display_path(Path(selected)))
            self._refresh_recordings()

    def _browse_recordings_root(self) -> None:
        """Choose and scan the recording browser's root directory."""
        current = project_path(self.recordings_root.get())
        initial = current if current.is_dir() else REPOSITORY_ROOT
        selected = filedialog.askdirectory(title="Select recordings folder", initialdir=initial)
        if selected:
            self.recordings_root.set(display_path(Path(selected)))
            self._refresh_recordings()

    def start_interactive_simulation(self) -> None:
        """Launch the full LIMB task simulator in the managed environment."""
        if self.simulation_python is None:
            messagebox.showerror(
                "Simulation environment missing",
                "Create aurora-simulation from src/simulation/environment.yml, then refresh Info.",
            )
            return
        self._start_program(
            "Interactive task simulator",
            "simulation",
            [str(self.simulation_python), "-u", str(INTERACTIVE_SIMULATION_SCRIPT)],
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

    # --- Recording program discovery and launch ----------------------------

    def _refresh_recording_programs(self) -> None:
        """Discover runnable record or capture entry points below src."""
        previous = self.recording_program.get()
        discovered: dict[str, Path] = {}
        source_root = REPOSITORY_ROOT / "src"
        if source_root.is_dir():
            for path in source_root.rglob("*.py"):
                lower_name = path.name.lower()
                if lower_name.startswith(RECORDING_ENTRYPOINT_PREFIXES):
                    # Only offer executable entry points, not helper modules with
                    # names such as recording_format.py.
                    try:
                        content = path.read_text(encoding="utf-8", errors="ignore")
                    except OSError:
                        continue
                    if "__main__" in content:
                        label = display_path(path)
                        discovered[label] = path
        self.recording_programs = dict(sorted(discovered.items()))
        values = list(self.recording_programs)
        self.recording_program_box.configure(values=values)
        if previous in self.recording_programs:
            self.recording_program.set(previous)
        elif values:
            self.recording_program.set(values[0])
        else:
            self.recording_program.set("")

        if values:
            self.recording_program_status.configure(
                text=(
                    f"{len(values)} recording program(s) available. "
                    "Output is provided as AURORA_OUTPUT_DIR."
                )
            )
        else:
            self.recording_program_status.configure(
                text=(
                    "No recording program is present yet. Add a record*.py or "
                    "capture*.py entry point under src."
                )
            )
        self._update_controls()

    def start_recording(self) -> None:
        """Launch the selected recorder with its output-directory contract."""
        program = self.recording_programs.get(self.recording_program.get())
        if program is None:
            messagebox.showinfo("Recording unavailable", "No recording program was found in src.")
            return
        output_value = self.recording_output.get().strip()
        if not output_value:
            messagebox.showerror("Output folder required", "Choose a recording output folder.")
            return
        output = project_path(output_value)
        try:
            output.mkdir(parents=True, exist_ok=True)
            arguments = self._split_arguments(self.recording_arguments.get())
        except (OSError, ValueError) as error:
            messagebox.showerror("Could not prepare recording", str(error))
            return

        environment = {"AURORA_OUTPUT_DIR": str(output.resolve())}
        python = self.simulation_python or _console_python(Path(sys.executable))
        command = [str(python), "-u", str(program), *arguments]
        self._start_program("Recording", "recording", command, environment=environment)

    @staticmethod
    def _split_arguments(value: str) -> list[str]:
        """Parse optional command arguments using platform-appropriate quoting."""
        if not value.strip():
            return []
        parts = shlex.split(value, posix=os.name != "nt")
        if os.name == "nt":
            parts = [
                part[1:-1] if len(part) >= 2 and part[0] == part[-1] and part[0] in "\"'" else part
                for part in parts
            ]
        return parts

    # --- Saved-recording browser -------------------------------------------

    def _refresh_recordings(self) -> None:
        """Scan supported recording files and cache their metadata."""
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
        """Debounce filter updates so typing stays responsive."""
        if self.recordings_filter_after is not None:
            self.after_cancel(self.recordings_filter_after)
        self.recordings_filter_after = self.after(180, self._render_scheduled_recordings)

    def _render_scheduled_recordings(self) -> None:
        """Run the pending recording-filter update."""
        self.recordings_filter_after = None
        self._render_recordings()

    def _render_recordings(self) -> None:
        """Render cached recording metadata using the current filter."""
        if not hasattr(self, "recordings_tree"):
            return
        self.recordings_tree.delete(*self.recordings_tree.get_children())
        self.recording_tree_paths.clear()
        root_value = self.recordings_root.get().strip()
        root = project_path(root_value) if root_value else None
        filter_text = self.recordings_filter.get().strip().casefold()
        visible = [
            path
            for path in self.recording_files
            if not filter_text or filter_text in display_path(path).casefold()
        ]
        for path in visible:
            try:
                relative = str(path.relative_to(root)) if root is not None else path.name
                size_bytes, modified_timestamp = self.recording_metadata[path]
                size = self._format_size(size_bytes)
                modified = datetime.fromtimestamp(modified_timestamp).strftime("%Y-%m-%d %H:%M")
            except (KeyError, OSError, ValueError):
                continue
            item = self.recordings_tree.insert(
                "",
                "end",
                text=relative,
                values=(path.suffix.lstrip(".").upper(), size, modified),
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
        """Format a byte count for the recordings table."""
        value = float(size)
        for unit in ("B", "KB", "MB", "GB"):
            if value < 1024 or unit == "GB":
                return f"{value:.0f} {unit}" if unit == "B" else f"{value:.1f} {unit}"
            value /= 1024
        return f"{size} B"

    def _open_selected_recording(self) -> None:
        """Open the file selected in the recordings table."""
        selected = self.recordings_tree.selection()
        if not selected:
            messagebox.showinfo("Select a recording", "Choose a file from the list first.")
            return
        path = self.recording_tree_paths.get(selected[0])
        if path is not None:
            self._open_existing(path)

    def _open_recordings_root(self) -> None:
        """Create when requested, then open the recording output folder."""
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

    # --- Firmware discovery and actions ------------------------------------

    def _discover_firmware_projects(self) -> dict[str, FirmwareProject]:
        """Find supported ESP-IDF and PlatformIO projects below firmware."""
        firmware_root = REPOSITORY_ROOT / "firmware"
        projects: dict[str, FirmwareProject] = {}
        if not firmware_root.is_dir():
            return projects

        for manifest in firmware_root.rglob("platformio.ini"):
            if ".pio" in manifest.parts:
                continue
            label = f"PlatformIO - {display_path(manifest.parent)}"
            projects[label] = FirmwareProject(manifest.parent, "PlatformIO", "pio")

        for manifest in firmware_root.rglob("CMakeLists.txt"):
            try:
                content = manifest.read_text(encoding="utf-8", errors="ignore").lower()
            except OSError:
                continue
            if "project.cmake" not in content or "project(" not in content:
                continue
            label = f"ESP-IDF - {display_path(manifest.parent)}"
            projects[label] = FirmwareProject(manifest.parent, "ESP-IDF", "idf.py")
        return dict(sorted(projects.items()))

    def _refresh_firmware(self) -> None:
        """Refresh firmware projects and currently visible serial ports."""
        previous = self.firmware_project.get()
        self.firmware_projects = self._discover_firmware_projects()
        values = list(self.firmware_projects)
        self.firmware_project_box.configure(values=values)
        if previous in self.firmware_projects:
            self.firmware_project.set(previous)
        elif values:
            self.firmware_project.set(values[0])
        else:
            self.firmware_project.set("")

        ports = serial_ports()
        current_port = self.serial_port.get()
        self.port_box.configure(values=ports)
        if not current_port and ports:
            self.serial_port.set(ports[0])
        self._firmware_selection_changed()

    def _firmware_selection_changed(self) -> None:
        """Update firmware details after the project selection changes."""
        project = self.firmware_projects.get(self.firmware_project.get())
        if project is None:
            self.firmware_status.configure(
                text="No ESP-IDF or PlatformIO firmware project is present under firmware yet."
            )
        elif shutil.which(project.executable):
            self.firmware_status.configure(
                text=f"{project.system} tools ready. Project: {display_path(project.directory)}"
            )
        else:
            self.firmware_status.configure(
                text=(
                    f"{project.system} project found, but {project.executable} "
                    "is not available on PATH."
                )
            )
        self._update_controls()

    def _run_firmware(self, action: str) -> None:
        """Build, flash, or monitor the selected firmware project."""
        project = self.firmware_projects.get(self.firmware_project.get())
        if project is None:
            messagebox.showinfo("Firmware unavailable", "No supported firmware project was found.")
            return
        if shutil.which(project.executable) is None:
            messagebox.showerror(
                "Tool unavailable",
                f"{project.executable} is required for this {project.system} project.",
            )
            return

        port = self.serial_port.get().strip()
        if action in {"flash", "monitor"} and not port:
            messagebox.showerror("Serial port required", "Select or enter the robot serial port.")
            return
        if action == "flash" and not messagebox.askyesno(
            "Flash robot firmware",
            f"Flash {display_path(project.directory)} to {port}?",
        ):
            return

        if project.system == "ESP-IDF":
            command = ["idf.py"]
            if action in {"flash", "monitor"}:
                command.extend(["-p", port])
            command.append(action)
        else:
            if action == "build":
                command = ["pio", "run"]
            elif action == "flash":
                command = ["pio", "run", "-t", "upload", "--upload-port", port]
            else:
                command = ["pio", "device", "monitor", "-p", port]

        self._start_program(
            f"Firmware {action}",
            "firmware",
            command,
            working_directory=project.directory,
        )

    # --- Project status -----------------------------------------------------

    def _refresh_info(self) -> None:
        """Render current dependency, program, and tool availability."""
        simulation_ready = (
            INTERACTIVE_SIMULATION_SCRIPT.is_file()
            and SIMULATION_SCRIPT.is_file()
            and MANUAL_SIMULATION_SCRIPT.is_file()
            and self.simulation_python is not None
        )
        active = (
            self.process_name
            if self.process is not None and self.process.poll() is None
            else "None"
        )
        lines = [
            "PROJECT STATUS\n",
            f"Simulation: {'Ready' if simulation_ready else 'Missing files or environment'}",
            f"Simulation environment: {environment_name(self.simulation_python)}",
            "NumPy, SciPy, PyBullet, and Pygame: "
            f"{'Ready' if self.simulation_python else 'Missing'}",
            f"Recording programs: {len(self.recording_programs)} found",
            f"Firmware projects: {len(self.firmware_projects)} found",
            f"ESP-IDF command: {'Available' if shutil.which('idf.py') else 'Not found'}",
            f"PlatformIO command: {'Available' if shutil.which('pio') else 'Not found'}",
            f"Active program: {active}",
            "",
            "PORTABILITY\n",
            "Paths inside the repository are stored relative to the project root. "
            "The GUI can run from any clone location without path edits.",
            "",
            "SUPPORTED ENTRY POINTS\n",
            "Simulation: full interactive task scene, trajectory playback, and joint sliders",
            "Recording: record*.py or capture*.py anywhere under src",
            "Firmware: ESP-IDF CMake projects or PlatformIO projects under firmware",
            "Recording output contract: AURORA_OUTPUT_DIR is passed to recording programs",
        ]
        self.info_text.configure(state="normal")
        self.info_text.delete("1.0", "end")
        self.info_text.insert("1.0", "\n".join(lines))
        self.info_text.configure(state="disabled")

    def _refresh_all(self) -> None:
        """Refresh every discovery-backed area of the launcher."""
        self.simulation_python = find_simulation_python()
        self._refresh_recording_programs()
        self._refresh_recordings()
        self._refresh_firmware()
        self._refresh_info()
        if self.simulation_python is None and self.process is None:
            self._set_status("Simulation environment missing", "warning")
        elif self.process is None:
            self._set_status("Ready", "ready")
        self._append_log("Project status refreshed.\n")

    # --- Shared child-process supervision ----------------------------------

    def _start_program(
        self,
        name: str,
        kind: str,
        command: list[str],
        *,
        working_directory: Path = REPOSITORY_ROOT,
        environment: dict[str, str] | None = None,
    ) -> None:
        """Start one managed child process and stream its output to Activity."""
        if self.process is not None and self.process.poll() is None:
            messagebox.showinfo("Program running", f"{self.process_name} is already running.")
            return
        merged_environment = os.environ.copy()
        if environment:
            merged_environment.update(environment)
        merged_environment["PYTHONUTF8"] = "1"
        merged_environment["PYTHONIOENCODING"] = "utf-8"
        try:
            creation_flags = 0
            start_new_session = os.name != "nt"
            if os.name == "nt":
                # Suppress extra console windows. Tkinter, Pygame, and PyBullet
                # windows created by the child remain visible.
                creation_flags = subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
            process = subprocess.Popen(
                command,
                cwd=working_directory,
                env=merged_environment,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                creationflags=creation_flags,
                start_new_session=start_new_session,
            )
        except OSError as error:
            messagebox.showerror(f"Could not start {name}", str(error))
            self._set_status("Start failed", "error")
            return

        self.process = process
        self.process_name = name
        self.process_kind = kind
        self.stopping = False
        self._set_status(f"{name} running", "running")
        self._append_log(f"\n> {self._display_command(command)}\n")
        self._update_controls()
        self._refresh_info()
        threading.Thread(
            target=self._watch_process,
            args=(process, name),
            daemon=True,
        ).start()

    def _watch_process(self, process: subprocess.Popen[str], name: str) -> None:
        """Wait for a child process without blocking Tk's event loop."""
        # The worker only writes to a queue because Tk widgets must stay on the
        # main thread.
        if process.stdout is not None:
            for line in process.stdout:
                self.log_queue.put(("line", line))
        self.log_queue.put(("exit", (process, name, process.wait())))

    def _drain_log_queue(self) -> None:
        """Move worker-thread output and completion events into Tk widgets."""
        try:
            while True:
                event, value = self.log_queue.get_nowait()
                if event == "line":
                    self._append_log(str(value))
                    continue
                process, name, return_code = value  # type: ignore[misc]
                if process is not self.process:
                    continue
                stopped = self.stopping
                self.process = None
                self.process_name = ""
                self.process_kind = ""
                self.stopping = False
                if stopped:
                    self._set_status(f"{name} stopped", "ready")
                elif return_code == 0:
                    self._set_status(f"{name} finished", "ready")
                else:
                    self._set_status(f"{name} failed (code {return_code})", "error")
                self._append_log(f"[{name} exited with code {return_code}]\n")
                self._update_controls()
                self._refresh_recordings()
                self._refresh_info()
        except queue.Empty:
            pass
        self.after(100, self._drain_log_queue)

    def stop_program(self) -> None:
        """Request a graceful stop and escalate to a process-tree kill if needed."""
        process = self.process
        if process is None or process.poll() is not None:
            return
        if self.stopping or not self._confirm_sensitive_stop():
            return
        self.stopping = True
        self._set_status(f"Stopping {self.process_name}", "running")
        if not self._request_process_stop(process):
            error = "the process did not accept a stop signal"
            self._append_log(f"Could not stop program: {error}\n")
            return
        # Give recorders and monitors time to flush files before forcing exit.
        self.after(4000, lambda: self._kill_process_tree_if_running(process))

    def _confirm_sensitive_stop(self) -> bool:
        """Confirm stopping work that can leave hardware or files incomplete."""
        if self.process_kind == "recording":
            return messagebox.askyesno(
                "Stop recording",
                "Stop the current recording? The program will have a few seconds "
                "to finish writing its files.",
            )
        if self.process_name == "Firmware flash":
            return messagebox.askyesno(
                "Stop firmware flash",
                "Interrupting a flash can leave the controller without working "
                "firmware. Stop anyway?",
            )
        return True

    @staticmethod
    def _request_process_stop(process: subprocess.Popen[str]) -> bool:
        """Send the platform's graceful termination signal to a child."""
        try:
            if os.name == "nt":
                process.send_signal(signal.CTRL_BREAK_EVENT)
            else:
                os.killpg(process.pid, signal.SIGINT)
            return True
        except (OSError, ValueError):
            try:
                process.terminate()
                return True
            except OSError:
                return False

    @staticmethod
    def _kill_process_tree_if_running(process: subprocess.Popen[str]) -> None:
        """Force-stop a child and its descendants after the grace period."""
        if process.poll() is not None:
            return
        try:
            if os.name == "nt":
                subprocess.run(
                    ["taskkill", "/PID", str(process.pid), "/T", "/F"],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=6,
                    check=False,
                    creationflags=subprocess.CREATE_NO_WINDOW,
                )
            else:
                os.killpg(process.pid, signal.SIGKILL)
        except (OSError, subprocess.TimeoutExpired):
            try:
                process.kill()
            except OSError:
                pass

    def _update_controls(self) -> None:
        """Enable actions only when their files, tools, and state are ready."""
        if not hasattr(self, "playback_button"):
            return
        running = self.process is not None and self.process.poll() is None
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
        recording_ready = self.recording_program.get() in self.recording_programs
        self.record_button.configure(
            state="disabled" if running or not recording_ready else "normal"
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

    # --- Shared UI helpers --------------------------------------------------

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
        process = self.process
        if process is not None and process.poll() is None:
            if not self._confirm_sensitive_stop():
                return
            self.stopping = True
            self._request_process_stop(process)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self._kill_process_tree_if_running(process)
        self.window.destroy()


# --- 6. Application entry point --------------------------------------------

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
