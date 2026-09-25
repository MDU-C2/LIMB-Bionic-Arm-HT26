"""Repository configuration and discovery helpers for the AURORA GUI.

This module deliberately has no Tkinter dependency.  Keeping path, environment,
and tool discovery here makes the launcher easier to test and keeps ``app.py``
focused on composing the interface.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import shutil
import subprocess
import sys


GUI_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = GUI_DIR.parents[1]
INTERACTIVE_SIMULATION_SCRIPT = (
    REPOSITORY_ROOT / "src" / "simulation" / "interactive" / "limb_simulator.py"
)
POSE_RECORDING_SCRIPT = (
    REPOSITORY_ROOT / "src" / "simulation" / "ai" / "pose_recording_sim.py"
)
MOVEMENT_RECOGNITION_SCRIPT = (
    REPOSITORY_ROOT / "src" / "simulation" / "ai" / "movement_recognition.py"
)
MOVEMENT_MODEL = (
    REPOSITORY_ROOT / "src" / "simulation" / "ai" / "models" / "movement_gru.onnx"
)
DEFAULT_RECORDINGS_DIRECTORY = Path("outputs/recordings")
DEFAULT_REFERENCE_DIRECTORY = Path("outputs/recordings/references")
DEFAULT_SIMULATION_OUTPUT_DIRECTORY = Path("outputs/simulation")
DOCUMENTATION_FILE = REPOSITORY_ROOT / "docs" / "SIMULATION.md"
GUI_DOCUMENTATION_FILE = GUI_DIR / "README.md"

RECORDING_EXTENSIONS = {
    ".avi", ".bag", ".csv", ".flac", ".json", ".mcap", ".mkv", ".mov",
    ".mp3", ".mp4", ".npy", ".npz", ".wav", ".webm",
}
RECORDING_ENTRYPOINT_PREFIXES = ("record", "capture")
RECORDER_DETAILS = {
    "record_ble_sensors.py": (
        "Legacy LIMB25 BLE cuff recorder for raw EMG and IMU data."
    ),
    "record_serial_sensors.py": (
        "Serial recorder for newline-delimited JSON and other sensor messages."
    ),
    "record_oak_pose.py": (
        "OAK-D video, selected-arm pose, and hand landmarks. Stereo depth is optional."
    ),
}

# Shared visual language.  Keeping the palette beside other application
# configuration prevents individual tabs from inventing one-off styles.
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


TAB_DEFINITIONS = (
    TabDefinition("simulation", "Simulation", "_build_simulation_tab"),
    TabDefinition("recording", "Recording", "_build_recording_tab"),
    TabDefinition("recordings", "Recordings", "_build_recordings_tab"),
    TabDefinition("motion_ai", "Motion AI", "_build_motion_ai_tab"),
    TabDefinition("robot", "Firmware", "_build_robot_tab"),
    TabDefinition("info", "Info", "_build_info_tab"),
)


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


def console_python(path: Path) -> Path:
    """Prefer python.exe over pythonw.exe so child output can be captured."""
    if path.name.lower() == "pythonw.exe":
        candidate = path.with_name("python.exe")
        if candidate.is_file():
            return candidate
    return path


def simulation_python_candidates() -> list[Path]:
    """Return likely interpreters without relying on the clone location."""
    candidates: list[Path] = []
    configured = os.environ.get("AURORA_SIMULATION_PYTHON")
    if configured:
        candidates.append(Path(configured).expanduser())

    candidates.append(console_python(Path(sys.executable)))
    relative_python = Path("python.exe") if os.name == "nt" else Path("bin/python")
    if os.name == "nt":
        candidates.extend([
            REPOSITORY_ROOT / ".venv" / "Scripts" / "python.exe",
            REPOSITORY_ROOT / "venv" / "Scripts" / "python.exe",
        ])
    else:
        candidates.extend([
            REPOSITORY_ROOT / ".venv" / "bin" / "python",
            REPOSITORY_ROOT / "venv" / "bin" / "python",
        ])

    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        candidates.append(Path(conda_prefix) / relative_python)

    environment_roots: list[Path] = []
    mamba_root = os.environ.get("MAMBA_ROOT_PREFIX")
    if mamba_root:
        environment_roots.append(Path(mamba_root))
    local_data = os.environ.get("LOCALAPPDATA")
    if local_data:
        environment_roots.append(Path(local_data) / "micromamba")
    environment_roots.extend([
        Path.home() / ".local" / "share" / "mamba",
        Path.home() / "micromamba",
        Path.home() / "miniconda3",
        Path.home() / "anaconda3",
        Path.home() / ".conda",
    ])
    for root in environment_roots:
        candidates.append(root / "envs" / "aurora-simulation" / relative_python)

    unique: list[Path] = []
    seen: set[str] = set()
    for candidate in candidates:
        normalized = console_python(candidate).resolve()
        key = os.path.normcase(str(normalized))
        if key not in seen:
            seen.add(key)
            unique.append(normalized)
    return unique


def has_python_dependencies(python: Path, modules: tuple[str, ...]) -> bool:
    """Return whether an interpreter imports the requested modules."""
    if not python.is_file():
        return False
    creation_flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    imports = "; ".join(f"import {module}" for module in modules)
    try:
        result = subprocess.run(
            [str(python), "-c", imports],
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
    """Find an interpreter that can run the migrated simulator."""
    for candidate in simulation_python_candidates():
        if has_python_dependencies(candidate, ("numpy", "scipy", "pybullet", "pygame")):
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


def discover_recording_programs() -> dict[str, Path]:
    """Find executable record/capture entry points below ``src``."""
    discovered: dict[str, Path] = {}
    source_root = REPOSITORY_ROOT / "src"
    if not source_root.is_dir():
        return discovered
    for path in source_root.rglob("*.py"):
        if not path.name.lower().startswith(RECORDING_ENTRYPOINT_PREFIXES):
            continue
        try:
            content = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if "__main__" in content:
            discovered[display_path(path)] = path
    return dict(sorted(discovered.items()))


def discover_firmware_projects() -> dict[str, FirmwareProject]:
    """Find ESP-IDF projects below ``firmware``."""
    projects: dict[str, FirmwareProject] = {}
    root = REPOSITORY_ROOT / "firmware"
    if not root.is_dir():
        return projects
    for path in root.rglob("CMakeLists.txt"):
        directory = path.parent
        try:
            content = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if "project(" in content and "IDF_PATH" in content:
            projects.setdefault(
                display_path(directory), FirmwareProject(directory, "ESP-IDF", "idf.py")
            )
    return dict(sorted(projects.items()))


def executable_available(name: str) -> bool:
    """Return whether a command-line tool is available on PATH."""
    return shutil.which(name) is not None
