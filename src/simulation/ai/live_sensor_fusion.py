# Majd
"""Start live control using the interactive simulator."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from imu_twist import DEFAULT_TWIST_AXIS, TWIST_AXES

INTERACTIVE_SIMULATOR = Path(__file__).resolve().parents[1] / "interactive" / "limb_simulator.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="ESP32 serial port (optional USB diagnostics)")
    parser.add_argument("--transport", choices=("ble", "serial"), help="BLE by default")
    parser.add_argument("--device", default="LIMBServer")
    parser.add_argument("--address", default="")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--camera-weight", type=float, default=0.25)
    parser.add_argument("--shoulder-twist-axis", choices=TWIST_AXES, default=DEFAULT_TWIST_AXIS)
    parser.add_argument("--wrist-twist-axis", choices=TWIST_AXES, default=DEFAULT_TWIST_AXIS)
    parser.add_argument("--grip-source", choices=("camera", "emg", "auto"), default="camera")
    parser.add_argument("--depth", action="store_true")
    parser.add_argument("--table-width-cm", type=float, default=60)
    parser.add_argument("--table-depth-cm", type=float, default=60)
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
        "--transport",
        args.transport or ("serial" if args.port else "ble"),
        "--device",
        args.device,
        "--address",
        args.address,
        "--baud",
        str(args.baud),
        "--side",
        args.side,
        "--camera-weight",
        str(args.camera_weight),
        "--table-width-cm",
        str(args.table_width_cm),
        "--table-depth-cm",
        str(args.table_depth_cm),
        "--grip-source",
        args.grip_source,
        "--shoulder-twist-axis",
        args.shoulder_twist_axis,
        "--wrist-twist-axis",
        args.wrist_twist_axis,
    ]
    if args.port:
        command += ["--port", args.port]
    if args.depth:
        command.append("--depth")
    return subprocess.call(command)


if __name__ == "__main__":
    raise SystemExit(main())
