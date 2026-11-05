# Majd
"""Install the app or launch it with the selected project Python."""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULES = (
    "tkinter",
    "numpy",
    "scipy",
    "pybullet",
    "pygame",
    "onnxruntime",
    "bleak",
    "serial",
    "depthai",
    "cv2",
    "mediapipe",
    "matplotlib",
    "torch",
    "onnx",
    "platformio",
)


def run_python(*arguments: str, cwd: Path = ROOT) -> None:
    """Use argument lists so spaces and non-ASCII clone paths remain intact."""
    subprocess.run([sys.executable, *arguments], cwd=cwd, check=True)


def verify(*, install_models: bool = False) -> None:
    """Actually load native libraries and check package compatibility."""
    print("Checking all Python dependencies...", flush=True)
    compatibility = subprocess.run(
        [sys.executable, "-m", "pip", "check"],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    # DepthAI wheels publish tags after a blank line, which pip treats
    # as a missing platform tag. Its native import below is the actual check.
    camera_metadata_only = (
        compatibility.returncode == 1
        and re.fullmatch(
            r"depthai \S+ is not supported on this platform", compatibility.stdout.strip()
        )
        is not None
    )
    if camera_metadata_only:
        print(
            "Checking DepthAI through its native module (wheel metadata is incomplete)...",
            flush=True,
        )
    else:
        print(compatibility.stdout, end="", flush=True)
    if compatibility.returncode and not camera_metadata_only:
        raise subprocess.CalledProcessError(compatibility.returncode, compatibility.args)
    # Isolate imports: native packages can conflict when loaded in one process,
    # while the application runs camera, simulation, and training separately.
    for module in MODULES:
        run_python("-c", f"import {module}")
    run_python(
        "-c",
        "import mediapipe as mp; assert hasattr(mp, 'solutions'), "
        "'MediaPipe legacy solutions required by camera tracking are missing'",
    )
    run_python(
        str(ROOT / "scripts/setup_pose_models.py"),
        *(("--install",) if install_models else ()),
    )
    run_python("-m", "compileall", "-q", "src")
    run_python("-m", "aurora.simulation.trajectory_playback", "--headless")
    run_python("-m", "platformio", "--version")


def prepare_firmware() -> None:
    """Download the pinned toolchain and check a build without opening the board."""
    print("Preparing the ESP32 firmware toolchain and checking the build...", flush=True)
    run_python(
        str(ROOT / "src/aurora/gui/firmware_workspace.py"),
        "--project",
        str(ROOT / "firmware/high_rate_emg_imu"),
        "--",
        sys.executable,
        "-m",
        "platformio",
        "run",
    )


def install() -> None:
    """Install and check all app features and the firmware toolchain."""
    print("Installing AURORA...", flush=True)
    run_python("-m", "pip", "install", "-r", "requirements-setup.txt")
    verify(install_models=True)
    prepare_firmware()
    print(
        "Setup complete. Use Start.bat (Windows) or Start.command (macOS/Linux) next time.",
        flush=True,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", action="store_true")
    parser.add_argument("--launch", action="store_true")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    if not (args.install or args.launch or args.check):
        parser.error("Choose --install, --launch, or --check")
    # Use this interpreter in every GUI child, regardless of other local envs.
    os.environ["AURORA_SIMULATION_PYTHON"] = sys.executable
    os.environ["PYTHONUTF8"] = "1"
    os.environ["PYTHONIOENCODING"] = "utf-8"
    try:
        if args.install:
            install()
        if args.check:
            verify()
        if args.launch:
            return subprocess.call([sys.executable, "-m", "aurora.gui.app"], cwd=ROOT)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(
            f"Setup failed: {error}\nRetry Setup after resolving the error above.", file=sys.stderr
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
