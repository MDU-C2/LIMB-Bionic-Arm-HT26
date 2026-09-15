"""Recognize a recorded movement profile with the migrated GRU model."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Iterable

import numpy as np


AI_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL = AI_DIR / "models" / "movement_gru.onnx"
SEQUENCE_LENGTH = 60
FEATURE_COUNT = 69
UNKNOWN_THRESHOLD = 0.51


@dataclass(frozen=True)
class ReferenceMatch:
    """One reference recording and its embedding distance."""

    user_id: str
    path: Path
    distance: float


def _point3(value: object, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain x, y, and depth")
    point = np.asarray(value, dtype=np.float32)
    if not np.all(np.isfinite(point)):
        raise ValueError(f"{name} contains a missing or invalid value")
    return point.tolist()


def frame_to_vector(frame: object) -> np.ndarray:
    """Convert one shoulder, elbow, and hand frame to 69 model features."""
    if not isinstance(frame, dict):
        raise ValueError("Each frame must be a JSON object")

    values = _point3(frame.get("shoulder"), "shoulder")
    values.extend(_point3(frame.get("elbow"), "elbow"))

    hand = frame.get("hand", [])
    if not isinstance(hand, list):
        raise ValueError("hand must be a list")
    hand_by_id: dict[int, dict[str, object]] = {}
    for point in hand:
        if isinstance(point, dict) and "id" in point:
            hand_by_id[int(point["id"])] = point

    for landmark_id in range(21):
        point = hand_by_id.get(landmark_id)
        if point is None:
            values.extend((0.0, 0.0, 0.0))
            continue
        values.extend(
            _point3(
                [point.get("x"), point.get("y"), point.get("depth_m")],
                f"hand landmark {landmark_id}",
            )
        )
    return np.asarray(values, dtype=np.float32)


def load_sequence(path: Path) -> tuple[np.ndarray, str]:
    """Load and preprocess one recording exactly as the GRU was trained."""
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"Could not read {path}: {error}") from error
    if not isinstance(raw, dict) or not isinstance(raw.get("data"), list):
        raise ValueError(f"{path} does not contain a recording data list")
    if not raw["data"]:
        raise ValueError(f"{path} contains no frames")

    sequence = np.stack([frame_to_vector(frame) for frame in raw["data"]])
    source_time = np.linspace(0.0, len(sequence) - 1, len(sequence))
    target_time = np.linspace(0.0, len(sequence) - 1, SEQUENCE_LENGTH)
    resampled = np.stack(
        [
            np.interp(target_time, source_time, sequence[:, feature])
            for feature in range(FEATURE_COUNT)
        ],
        axis=1,
    ).astype(np.float32)

    # Keep the original centering so new embeddings match the trained model.
    points = resampled.reshape(SEQUENCE_LENGTH, 23, 3)
    points -= points[:, :1, :]
    user_id = str(raw.get("user_id", path.stem))
    return points.reshape(SEQUENCE_LENGTH, FEATURE_COUNT), user_id


class MovementRecognizer:
    """Run the portable ONNX model and compare recording embeddings."""

    def __init__(self, model_path: Path = DEFAULT_MODEL) -> None:
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise RuntimeError(
                "ONNX Runtime is missing. Recreate aurora-simulation from environment.yml."
            ) from error

        if not model_path.is_file():
            raise FileNotFoundError(f"Movement model not found: {model_path}")
        self.session = ort.InferenceSession(
            str(model_path),
            providers=["CPUExecutionProvider"],
        )
        model_input = self.session.get_inputs()[0]
        model_output = self.session.get_outputs()[0]
        if model_input.shape[1:] != [SEQUENCE_LENGTH, FEATURE_COUNT]:
            raise ValueError(f"Unexpected model input shape: {model_input.shape}")
        if model_output.shape[-1] != 128:
            raise ValueError(f"Unexpected model output shape: {model_output.shape}")
        self.input_name = model_input.name

    def embed(self, sequences: np.ndarray) -> np.ndarray:
        """Return one normalized embedding per preprocessed sequence."""
        batch = np.asarray(sequences, dtype=np.float32)
        if batch.ndim == 2:
            batch = batch[None, :, :]
        expected = (SEQUENCE_LENGTH, FEATURE_COUNT)
        if batch.ndim != 3 or batch.shape[1:] != expected:
            raise ValueError(f"Expected a batch shaped (N, {expected[0]}, {expected[1]})")
        return np.asarray(self.session.run(None, {self.input_name: batch})[0])


def _reference_files(directory: Path, excluded: Path) -> Iterable[Path]:
    for path in sorted(directory.rglob("*.json")):
        try:
            if path.resolve() != excluded.resolve():
                yield path
        except OSError:
            continue


def recognize_recording(
    recording: Path,
    references: Path,
    model_path: Path = DEFAULT_MODEL,
    threshold: float = UNKNOWN_THRESHOLD,
) -> tuple[str, list[ReferenceMatch], int]:
    """Return a profile prediction, ranked matches, and skipped file count."""
    if not recording.is_file():
        raise FileNotFoundError(f"Recording not found: {recording}")
    if not references.is_dir():
        raise FileNotFoundError(f"Reference folder not found: {references}")

    recognizer = MovementRecognizer(model_path)
    target, _ = load_sequence(recording)
    valid: list[tuple[Path, str, np.ndarray]] = []
    skipped = 0
    for path in _reference_files(references, recording):
        try:
            sequence, user_id = load_sequence(path)
        except ValueError:
            skipped += 1
            continue
        valid.append((path, user_id, sequence))
    if not valid:
        raise ValueError("The reference folder contains no valid pose recordings")

    reference_embeddings = recognizer.embed(np.stack([item[2] for item in valid]))
    target_embedding = recognizer.embed(target)[0]
    distances = np.linalg.norm(reference_embeddings - target_embedding, axis=1)
    order = np.argsort(distances)
    matches = [
        ReferenceMatch(valid[index][1], valid[index][0], float(distances[index]))
        for index in order
    ]
    prediction = matches[0].user_id if matches[0].distance <= threshold else "Unknown"
    return prediction, matches, skipped


def main() -> None:
    """Run movement-profile recognition from the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--references", type=Path, required=True)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--threshold", type=float, default=UNKNOWN_THRESHOLD)
    args = parser.parse_args()
    if args.threshold <= 0:
        parser.error("--threshold must be positive")

    prediction, matches, skipped = recognize_recording(
        args.recording,
        args.references,
        args.model,
        args.threshold,
    )
    print(f"Prediction: {prediction}")
    print("Closest references:")
    for match in matches[:5]:
        print(f"  {match.user_id}: {match.path.name} ({match.distance:.4f})")
    if skipped:
        print(f"Skipped {skipped} invalid JSON file(s)")


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        raise SystemExit(f"Error: {error}") from error
