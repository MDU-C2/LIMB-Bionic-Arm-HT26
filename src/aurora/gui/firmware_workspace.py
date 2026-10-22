# Majd
"""Run a firmware command from a no-space mirror of an ESP-IDF project."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

PROJECT_FILES = (
    "CMakeLists.txt",
    "platformio.ini",
    "sdkconfig.defaults",
)


def source_files(project: Path) -> list[Path]:
    """Return the checked-in inputs that determine a firmware build."""
    files = [project / name for name in PROJECT_FILES if (project / name).is_file()]
    for folder in ("main", "src", "include"):
        source_dir = project / folder
        if source_dir.is_dir():
            files.extend(path for path in source_dir.rglob("*") if path.is_file())
    return sorted(files, key=lambda path: path.relative_to(project).as_posix())


def staged_project(project: Path) -> Path:
    """Copy source inputs into a stable no-space directory for ESP-IDF."""
    project = project.resolve()
    files = source_files(project)
    if not files:
        raise RuntimeError(f"No firmware source files found in {project}")

    digest = hashlib.sha256()
    for source in files:
        relative = source.relative_to(project)
        digest.update(relative.as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(source.read_bytes())
        digest.update(b"\0")

    runtime = os.environ.get("AURORA_SETUP_DIRECTORY", "").strip()
    if not runtime:
        try:
            state = json.loads(
                (Path(__file__).resolve().parents[3] / ".aurora/runtime.json").read_text(
                    encoding="utf-8-sig"
                )
            )
            runtime = str(Path(state["prefix"]).parent)
        except (OSError, ValueError, KeyError, TypeError):
            pass
    staging_root = Path(runtime).expanduser() if runtime else Path(tempfile.gettempdir())
    staging_root = staging_root.resolve() / "aurora_firmware"
    name = re.sub(r"[^A-Za-z0-9_-]", "_", project.name)
    destination = staging_root / f"{name}-{digest.hexdigest()[:12]}"
    if any(character.isspace() for character in str(destination)):
        raise RuntimeError(f"Temporary firmware path still contains whitespace: {destination}")
    destination.mkdir(parents=True, exist_ok=True)
    for source in files:
        target = destination / source.relative_to(project)
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.is_file() or target.read_bytes() != source.read_bytes():
            shutil.copy2(source, target)
    return destination


def parse_args(argv: list[str]) -> tuple[Path, list[str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, required=True)
    if "--help" in argv or "-h" in argv:
        parser.parse_args(["--help"])
    try:
        separator = argv.index("--")
    except ValueError as error:
        raise ValueError("Separate the firmware command with --") from error
    args = parser.parse_args(argv[:separator])
    command = argv[separator + 1 :]
    if not command:
        raise ValueError("No firmware command was supplied")
    return args.project, command


def main(argv: list[str] | None = None) -> int:
    project, command = parse_args(sys.argv[1:] if argv is None else argv)
    staging = staged_project(project)
    print(f"Firmware workspace: {staging}", flush=True)
    return subprocess.call(command, cwd=staging)


if __name__ == "__main__":
    raise SystemExit(main())
