"""Run a firmware command from a no-space mirror of an ESP-IDF project."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


PROJECT_FILES = (
    "CMakeLists.txt",
    "platformio.ini",
    "sdkconfig.defaults",
)


def source_files(project: Path) -> list[Path]:
    """Return the checked-in inputs that determine a firmware build."""
    files = [project / name for name in PROJECT_FILES if (project / name).is_file()]
    main = project / "main"
    if main.is_dir():
        files.extend(path for path in main.rglob("*") if path.is_file())
    return sorted(files, key=lambda path: path.relative_to(project).as_posix())


def staged_project(project: Path) -> Path:
    """Copy source inputs into a stable no-space directory for old ESP-IDF."""
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

    staging_root = Path(tempfile.gettempdir()) / "aurora_firmware"
    destination = staging_root / f"{project.name}-{digest.hexdigest()[:12]}"
    if any(character.isspace() for character in str(destination)):
        raise RuntimeError(f"Temporary firmware path still contains whitespace: {destination}")
    if not destination.is_dir():
        destination.mkdir(parents=True)
        for source in files:
            target = destination / source.relative_to(project)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    return destination


def parse_args(argv: list[str]) -> tuple[Path, list[str]]:
    try:
        separator = argv.index("--")
    except ValueError as error:
        raise ValueError("Separate the firmware command with --") from error
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, required=True)
    args = parser.parse_args(argv[:separator])
    command = argv[separator + 1:]
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
