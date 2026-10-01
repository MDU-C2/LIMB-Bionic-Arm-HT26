"""Finish one-click setup, verify it, and launch from the managed environment."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
STATE = Path(os.environ.get("AURORA_SETUP_DIRECTORY", str(ROOT / ".aurora")))
READY = STATE / "ready.json"
# A separate exit status lets Start distinguish stale setup from an app failure.
NEEDS_SETUP = 10
INPUTS = (
    "src/simulation/environment.yml",
    "src/ml/requirements.txt",
    "requirements-setup.txt",
    "firmware/dual_imu_serial/platformio.ini",
    "scripts/setup_project.py",
    "scripts/setup.ps1",
    "Setup.command",
)
MODULES = (
    "tkinter", "numpy", "scipy", "pybullet", "pygame", "onnxruntime",
    "bleak", "serial", "depthai", "cv2", "mediapipe", "torch",
    "matplotlib", "onnx", "platformio",
)


def fingerprint() -> str:
    """Tie readiness to this clone, platform, and all dependency definitions."""
    digest = hashlib.sha256()
    for value in (str(ROOT), sys.platform, platform.machine(), sys.version):
        digest.update(value.encode("utf-8"))
        digest.update(b"\0")
    for name in INPUTS:
        digest.update(name.encode("utf-8"))
        digest.update((ROOT / name).read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def is_ready() -> bool:
    try:
        return json.loads(READY.read_text(encoding="utf-8"))["fingerprint"] == fingerprint()
    except (OSError, ValueError, KeyError, TypeError):
        return False


def run_python(*arguments: str, cwd: Path = ROOT) -> None:
    """Use argument lists so spaces and non-ASCII clone paths remain intact."""
    subprocess.run([sys.executable, *arguments], cwd=cwd, check=True)


def verify() -> None:
    """Actually load native libraries and check package compatibility."""
    print("Checking all Python dependencies...", flush=True)
    compatibility = subprocess.run(
        [sys.executable, "-m", "pip", "check"], cwd=ROOT,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    # DepthAI 3.10.0 publishes WHEEL tags after a blank line, which pip treats
    # as a missing platform tag. Its native import below is the actual check.
    camera_metadata_only = compatibility.returncode == 1 and compatibility.stdout.strip() == (
        "depthai 3.10.0 is not supported on this platform"
    )
    if camera_metadata_only:
        print("Checking DepthAI through its native module (wheel metadata is incomplete)...", flush=True)
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
    run_python("-m", "compileall", "-q", "src")
    run_python("src/simulation/sim/limb_sim.py", "--headless")


def install() -> None:
    """Install training/tools, then build firmware to fetch its full toolchain."""
    READY.unlink(missing_ok=True)
    print("Installing ML training and firmware tools (this can take several minutes)...", flush=True)
    if sys.platform in {"win32", "linux"}:
        # CPU training works without an NVIDIA GPU or multi-GB CUDA dependencies.
        run_python(
            "-m", "pip", "install", "torch>=2.2,<2.6",
            "--index-url", "https://download.pytorch.org/whl/cpu",
        )
    run_python("-m", "pip", "install", "-r", "requirements-setup.txt")
    verify()
    print("Preparing and building the ESP32-C3 firmware toolchain...", flush=True)
    # Reuse the GUI's mirror to support OneDrive paths containing spaces.
    # Always build via the installed PlatformIO rather than a machine's old IDF.
    run_python(
        "src/gui/firmware_workspace.py", "--project",
        str(ROOT / "firmware" / "dual_imu_serial"), "--",
        sys.executable, "-m", "platformio", "run",
    )
    STATE.mkdir(parents=True, exist_ok=True)
    READY.write_text(json.dumps({"fingerprint": fingerprint()}, indent=2) + "\n", encoding="utf-8")
    print("Setup complete. Use Start.bat (Windows) or Start.command (macOS/Linux) next time.", flush=True)


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
            if not is_ready():
                print("First-time or updated dependencies detected. Running setup...", flush=True)
                return NEEDS_SETUP
            return subprocess.call([sys.executable, str(ROOT / "src/gui/app.py")], cwd=ROOT)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"Setup failed: {error}\nRetry Setup after resolving the error above.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
