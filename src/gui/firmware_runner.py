"""Run PlatformIO firmware actions from a whitespace-safe staging directory."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def platformio_arguments(action: str, port: str) -> list[str]:
    """Return PlatformIO arguments for one GUI firmware action."""
    if action == "build":
        return ["run"]
    if action == "flash":
        return ["run", "--target", "upload", "--upload-port", port]
    if action == "monitor":
        return ["device", "monitor", "--port", port, "--baud", "115200"]
    raise ValueError(f"Unsupported firmware action: {action}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("build", "flash", "monitor"))
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--port", default="")
    parser.add_argument("--tool", nargs=argparse.REMAINDER, required=True)
    args = parser.parse_args()

    project = args.project.resolve()
    if not project.is_dir():
        parser.error(f"Firmware project does not exist: {project}")
    tool = list(args.tool)
    if not tool:
        parser.error("PlatformIO command is missing")

    arguments = platformio_arguments(args.action, args.port)
    if args.action == "monitor":
        return subprocess.call([*tool, *arguments], cwd=project)

    # ESP-IDF rejects whitespace anywhere in its project path. The repository
    # commonly lives below a OneDrive university folder, so compile/upload a
    # fresh source copy in a whitespace-safe system cache directory.
    project_key = hashlib.sha256(str(project).encode("utf-8")).hexdigest()[:12]
    staging_root = Path(tempfile.gettempdir()) / "aurora_firmware_cache" / project_key
    staged_project = staging_root / "project"
    staged_project.mkdir(parents=True, exist_ok=True)
    shutil.copytree(
        project,
        staged_project,
        dirs_exist_ok=True,
        ignore=shutil.ignore_patterns(".pio", "build", "sdkconfig", "sdkconfig.old"),
    )
    # Regenerate configuration from sdkconfig.defaults while retaining the
    # expensive object cache in .pio for quick later Build/Flash actions.
    for generated_config in ("sdkconfig", "sdkconfig.old"):
        (staged_project / generated_config).unlink(missing_ok=True)
    print(f"Firmware staging path: {staged_project}", flush=True)
    return subprocess.call([*tool, *arguments], cwd=staged_project)


if __name__ == "__main__":
    raise SystemExit(main())
