"""Compatibility launcher for interactive camera and dual-IMU control.

The live controller now uses the full interactive task simulator. This small
entry point keeps older commands working while avoiding a second arm scene and
a duplicate sensor loop.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


INTERACTIVE_SIMULATOR = (
    Path(__file__).resolve().parents[1] / "interactive" / "limb_simulator.py"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="ESP32 serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--camera-weight", type=float, default=0.25)
    parser.add_argument("--depth", action="store_true")
    parser.add_argument("--mode", choices=("kinematic", "dynamic"), default="kinematic")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    command = [
        sys.executable,
        "-u",
        str(INTERACTIVE_SIMULATOR),
        "--control",
        "camera-imu",
        "--mode",
        args.mode,
        "--port",
        args.port,
        "--baud",
        str(args.baud),
        "--side",
        args.side,
        "--camera-weight",
        str(args.camera_weight),
    ]
    if args.depth:
        command.append("--depth")
    return subprocess.call(command)


if __name__ == "__main__":
    raise SystemExit(main())
