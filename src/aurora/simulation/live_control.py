# Majd
"""Start live control using the interactive simulator."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from aurora.simulation.interactive.settings import parse_args as simulator_args

INTERACTIVE_SIMULATOR = Path(__file__).resolve().parent / "interactive" / "__main__.py"


def parse_args() -> argparse.Namespace:
    return simulator_args([*sys.argv[1:], "--control", "camera-imu"])


def main() -> int:
    args = parse_args()
    command = [
        sys.executable,
        "-u",
        str(INTERACTIVE_SIMULATOR),
        *sys.argv[1:],
        "--control",
        "camera-imu",
        "--transport",
        args.transport,
    ]
    return subprocess.call(command)


if __name__ == "__main__":
    raise SystemExit(main())
