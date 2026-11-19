# Majd
"""Compile the source and run each test module in a separate process."""

from __future__ import annotations

import argparse
import importlib.util
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=("runtime", "training", "all"), default="runtime")
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("Use a positive per-module timeout")
    training_modules = {"test_emg_neural.py", "test_movement_training.py"}
    if args.suite in ("training", "all"):
        missing = [
            name
            for name in ("torch", "onnx", "onnxruntime")
            if importlib.util.find_spec(name) is None
        ]
        if missing:
            print("Training prerequisites missing: " + ", ".join(missing))
            return 2
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
    if args.suite != "all":
        modules = [
            module
            for module in modules
            if (module.name in training_modules) == (args.suite == "training")
        ]
    print(f"Suite: {args.suite}. Hardware checks are separate scripts.", flush=True)
    for module in modules:
        print(f"Running {module.name}", flush=True)
        try:
            result = subprocess.run(
                [sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", module.name],
                cwd=ROOT,
                env=environment,
                check=False,
                timeout=args.timeout,
            )
        except subprocess.TimeoutExpired:
            print(f"Timed out: {module.name}", flush=True)
            failed.append(module.name)
        else:
            if result.returncode:
                failed.append(module.name)
    print(f"{len(modules)} test modules checked; {len(failed)} failed.", flush=True)
    return int(bool(failed))


if __name__ == "__main__":
    raise SystemExit(main())
