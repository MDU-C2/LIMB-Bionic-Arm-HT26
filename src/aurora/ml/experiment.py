# Majd
"""Save the files and settings used for a camera profile training run."""

from __future__ import annotations

import importlib.metadata
import platform
import subprocess
from pathlib import Path

from aurora.file_utils import file_digest as sha256
from aurora.json_utils import read_json_object, write_json
from aurora.ml.features import COORDINATE_SYSTEM, PREPROCESSING, load_movement
from aurora.paths import CHECKOUT_DIRECTORY, REPOSITORY_ROOT
from aurora.time_utils import utc_now


def create_manifest(data: Path, splits: dict[str, list[str]], settings: dict) -> dict:
    files = []
    seen = set()
    for split, names in splits.items():
        for name in names:
            path = (data / name).resolve()
            if not path.is_relative_to(data.resolve()) or path in seen:
                raise ValueError("Training and validation files must be separate and inside --data")
            seen.add(path)
            _frames, subject_id = load_movement(path)
            files.append(
                {
                    "file": path.relative_to(data.resolve()).as_posix(),
                    "subject_id": subject_id,
                    "split": split,
                    "sha256": sha256(path),
                }
            )
    versions = {"python": platform.python_version()}
    for name in ("torch", "numpy", "onnx", "matplotlib"):
        versions[name] = importlib.metadata.version(name)
    revision = None
    dirty = None
    try:
        if CHECKOUT_DIRECTORY is None:
            raise FileNotFoundError("No source checkout for this installation")
        revision = (
            subprocess.check_output(
                ["git", "rev-parse", "HEAD"],
                cwd=REPOSITORY_ROOT,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
            )
            .decode()
            .strip()
        )
        dirty = bool(
            subprocess.check_output(
                ["git", "status", "--porcelain"],
                cwd=REPOSITORY_ROOT,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
            ).strip()
        )
    except (OSError, subprocess.CalledProcessError):
        pass
    source_files = {
        path.name: sha256(path)
        for path in (
            Path(__file__),
            Path(__file__).with_name("features.py"),
            Path(__file__).with_name("train_gru.py"),
            Path(__file__).with_name("training_plot.py"),
        )
    }
    return {
        "format_version": 1,
        "status": "running",
        "created_utc": utc_now(),
        "task": "camera_movement_profile_identification",
        "split_method": "separate_trials_within_each_subject",
        "settings": settings,
        "preprocessing": dict(PREPROCESSING),
        "versions": versions,
        "platform": platform.platform(),
        "code": {"revision": revision, "uncommitted_changes": dirty, "source_sha256": source_files},
        "files": files,
    }


def write_manifest(directory: Path, manifest: dict) -> None:
    write_json(directory / "training.json", manifest)


def verify_model_manifest(
    model: Path, *, require_manifest: bool = False
) -> tuple[dict, Path] | None:
    """Validate a sidecar or training run, allowing earlier models without either."""
    sidecar = model.with_suffix(".json")
    if not sidecar.is_file():
        sidecar = model.parent / "training.json"
    if not sidecar.is_file():
        if require_manifest:
            raise ValueError(
                "Select a model with a completed training manifest or inherited sidecar"
            )
        return
    try:
        manifest = read_json_object(sidecar)
        expected_hash = manifest["artifacts"][model.name]
        coordinates = manifest["preprocessing"]["coordinate_system"]
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise ValueError(f"Invalid model manifest: {sidecar.name}") from error
    if expected_hash != sha256(model) or coordinates != COORDINATE_SYSTEM:
        raise ValueError("Model weights or coordinate system do not match the model manifest")
    if "format_version" in manifest and manifest["format_version"] != 1:
        raise ValueError("Unsupported model manifest format_version")
    if require_manifest or "format_version" in manifest:
        if manifest.get("status") not in {"complete", "inherited"}:
            raise ValueError("Select a completed training run")
        if any(manifest["preprocessing"].get(key) != value for key, value in PREPROCESSING.items()):
            raise ValueError("Model preprocessing does not match movement recognition")
    return manifest, sidecar
