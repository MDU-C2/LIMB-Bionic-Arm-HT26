# Majd, Christian
"""Run the interactive LIMB task simulator migrated from LIMB-HT25."""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
# Conda's dynamically loaded BLAS DLLs need PATH as well as Python's DLL directory.
if os.name == "nt":
    native_bin = Path(sys.prefix) / "Library" / "bin"
    if native_bin.is_dir():
        os.environ["PATH"] = str(native_bin) + os.pathsep + os.environ.get("PATH", "")
SIMULATION_ROOT = Path(__file__).resolve().parents[1]
if str(SIMULATION_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATION_ROOT))

from interactive.settings import parse_args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    from interactive.runtime import InteractiveSimulator

    InteractiveSimulator(args).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
