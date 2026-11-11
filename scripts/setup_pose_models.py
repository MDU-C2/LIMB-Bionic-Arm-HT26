# Christian och Majd
"""Install and check the MediaPipe pose models used by the app."""

from __future__ import annotations

import argparse
import shutil
import sys
import tempfile
import urllib.request
from pathlib import Path

POSE_MODELS = ((0, "lite"), (1, "full"), (2, "heavy"))
MODEL_URL = "https://storage.googleapis.com/mediapipe-assets/"


def valid_model_file(path: Path) -> bool:
    """Check the TFLite header before trying to load a model."""
    try:
        with path.open("rb") as model:
            return model.read(8)[4:] == b"TFL3"
    except FileNotFoundError:
        return False


def download_model(path: Path) -> None:
    """Publish a complete download without leaving a partial model on failure."""
    print(f"Downloading MediaPipe model: {path.name}", flush=True)
    pending = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=path.parent, prefix=path.name + ".", suffix=".download", delete=False
        ) as target:
            pending = Path(target.name)
            with urllib.request.urlopen(MODEL_URL + path.name, timeout=60) as source:
                shutil.copyfileobj(source, target)
        if not valid_model_file(pending):
            raise ValueError(f"Downloaded file is not a TFLite model: {path.name}")
        pending.replace(path)
    finally:
        if pending is not None:
            pending.unlink(missing_ok=True)


def check_models(*, install: bool = False) -> None:
    """Load every pose option and process a blank image without opening a camera."""
    import mediapipe as mp
    import numpy as np

    directory = Path(mp.__file__).resolve().parent / "modules/pose_landmark"
    image = np.zeros((64, 64, 3), dtype=np.uint8)
    for complexity, name in POSE_MODELS:
        path = directory / f"pose_landmark_{name}.tflite"
        if install and not valid_model_file(path):
            directory.mkdir(parents=True, exist_ok=True)
            download_model(path)
        if not valid_model_file(path):
            raise RuntimeError(f"MediaPipe {name} pose model is missing or invalid. Run Setup.")
        print(f"Checking MediaPipe {name} pose model...", flush=True)
        with mp.solutions.pose.Pose(static_image_mode=True, model_complexity=complexity) as pose:
            pose.process(image)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", action="store_true", help="Download missing models first.")
    args = parser.parse_args(argv)
    try:
        check_models(install=args.install)
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        print(f"Pose model setup failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
