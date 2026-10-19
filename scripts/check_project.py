# Majd
"""Compile the source and run each test module in a separate process."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    environment = os.environ.copy()
    if os.name == "nt":
        native = Path(sys.executable).parent / "Library" / "bin"
        if native.is_dir():
            environment["PATH"] = str(native) + os.pathsep + environment.get("PATH", "")
    environment["PYTHONUTF8"] = "1"
    environment["PYTHONIOENCODING"] = "utf-8"

    compiled = subprocess.run(
        [sys.executable, "-m", "compileall", "-q", "src", "scripts", "tests"],
        cwd=ROOT,
        env=environment,
        check=False,
    )
    if compiled.returncode:
        return compiled.returncode

    failed = []
    modules = sorted((ROOT / "tests").glob("test_*.py"))
    for module in modules:
        print(f"Running {module.name}", flush=True)
        result = subprocess.run(
            [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", module.name],
            cwd=ROOT,
            env=environment,
            check=False,
        )
        if result.returncode:
            failed.append(module.name)
    print(f"{len(modules)} test modules checked; {len(failed)} failed.", flush=True)
    return int(bool(failed))


if __name__ == "__main__":
    raise SystemExit(main())
