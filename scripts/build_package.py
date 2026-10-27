# Majd
"""Include runtime resources without copying them into the source package."""

from __future__ import annotations

import os
import re
from pathlib import Path
from urllib.parse import unquote, urlsplit

from setuptools.command.build_py import build_py

ROOT = Path(__file__).resolve().parents[1]
RESOURCE_PATTERNS = (
    "README.md",
    "data/README.md",
    "examples/README.md",
    "firmware/README.md",
    "hardware/README.md",
    "scripts/README.md",
    "scripts/check_ble.py",
    "scripts/check_dual_imu.py",
    "scripts/check_oak_camera.py",
    "scripts/check_emg_calibration.py",
    "scripts/check_live_replay.py",
    "scripts/diagnose_live_control.py",
    "scripts/benchmark_runtime.py",
    "src/README.md",
    "tests/README.md",
    "docs/*.md",
    "research/*.md",
    "artifacts/models/README.md",
    "artifacts/models/ULTRALYTICS_LICENSE.txt",
    "artifacts/models/movement_gru.onnx",
    "artifacts/models/movement_gru.json",
    "artifacts/models/cup_yolo11n.onnx",
    "examples/simulation/demo/*.npz",
    "firmware/high_rate_emg_imu/README.md",
    "firmware/high_rate_emg_imu/CMakeLists.txt",
    "firmware/high_rate_emg_imu/platformio.ini",
    "firmware/high_rate_emg_imu/sdkconfig.defaults",
    "firmware/high_rate_emg_imu/main/*.c",
    "firmware/high_rate_emg_imu/main/CMakeLists.txt",
    "firmware/high_rate_emg_imu/include/*.h",
)


def resource_files() -> list[Path]:
    return sorted({p for pattern in RESOURCE_PATTERNS for p in ROOT.glob(pattern) if p.is_file()})


class ResourceBuild(build_py):
    """Put models, guides, examples and firmware inside installed wheels."""

    def destination(self, source: Path) -> Path:
        relative = source.relative_to(ROOT)
        package = Path(self.build_lib) / "aurora"
        if relative.parts[:2] == ("src", "aurora"):
            return package.joinpath(*relative.parts[2:])
        return package / "resources" / relative

    def rewrite_links(self, source: Path, target: Path) -> None:
        def replace(match: re.Match) -> str:
            value = match.group(1)
            parsed = urlsplit(value.strip("<>"))
            if parsed.scheme or parsed.netloc or not parsed.path:
                return match.group(0)
            resolved = (source.parent / unquote(parsed.path)).resolve()
            if not resolved.is_relative_to(ROOT):
                return match.group(0)
            relative = os.path.relpath(self.destination(resolved), target.parent).replace("\\", "/")
            if parsed.fragment:
                relative += "#" + parsed.fragment
            return match.group(0).replace(value, "<" + relative + ">", 1)

        text = source.read_text(encoding="utf-8-sig")
        target.write_text(re.sub(r"\]\((<[^>]+>|[^\s)]+)\)", replace, text), encoding="utf-8")

    def run(self) -> None:
        super().run()
        if self.editable_mode:
            return
        for source in resource_files():
            target = self.destination(source)
            self.mkpath(str(target.parent))
            self.copy_file(str(source), str(target))
            if source.suffix == ".md":
                self.rewrite_links(source, target)
        for source in (ROOT / "src" / "aurora").rglob("README.md"):
            target = self.destination(source)
            if target.is_file():
                self.rewrite_links(source, target)

    def get_source_files(self) -> list[str]:
        sources = [*resource_files(), ROOT / "scripts" / "build_package.py"]
        return super().get_source_files() + [p.relative_to(ROOT).as_posix() for p in sources]

    def get_outputs(self, include_bytecode: bool = True) -> list[str]:
        outputs = super().get_outputs(include_bytecode)
        if not self.editable_mode:
            outputs.extend(str(self.destination(p)) for p in resource_files())
        return outputs
