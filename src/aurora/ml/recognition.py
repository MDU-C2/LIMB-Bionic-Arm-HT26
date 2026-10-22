# Majd
"""Compare recorded movements using the GRU model."""

from __future__ import annotations

import argparse
import math
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from aurora.ml.experiment import verify_model_manifest
from aurora.ml.features import FEATURE_COUNT, SEQUENCE_LENGTH, load_sequence
from aurora.paths import MODEL_DIRECTORY

DEFAULT_MODEL = MODEL_DIRECTORY / "movement_gru.onnx"
UNKNOWN_THRESHOLD = 0.51


@dataclass(frozen=True)
class ReferenceMatch:
    """One reference recording and its embedding distance."""

    subject_id: str
    path: Path
    distance: float


class MovementRecognizer:
    """Run the portable ONNX model and compare recording embeddings."""

    def __init__(self, model_path: Path = DEFAULT_MODEL) -> None:
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise RuntimeError(
                "ONNX Runtime is missing. Run Setup.bat (Windows) or Setup.command (macOS/Linux)."
            ) from error

        if not model_path.is_file():
            raise FileNotFoundError(f"Movement model not found: {model_path}")
        verify_model_manifest(model_path)
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
        if not len(batch) or not np.all(np.isfinite(batch)):
            raise ValueError("Movement inputs must contain finite samples in a non-empty batch")
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
    if not math.isfinite(threshold) or threshold <= 0:
        raise ValueError("Recognition threshold must be finite and positive")
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
            sequence, subject_id = load_sequence(path)
        except ValueError:
            skipped += 1
            continue
        valid.append((path, subject_id, sequence))
    if not valid:
        raise ValueError("The reference folder contains no valid pose recordings")

    reference_embeddings = recognizer.embed(np.stack([item[2] for item in valid]))
    target_embedding = recognizer.embed(target)[0]
    distances = np.linalg.norm(reference_embeddings - target_embedding, axis=1)
    order = np.argsort(distances)
    matches = [
        ReferenceMatch(valid[index][1], valid[index][0], float(distances[index])) for index in order
    ]
    prediction = matches[0].subject_id if matches[0].distance <= threshold else "Unknown"
    return prediction, matches, skipped


def main() -> None:
    """Run movement-profile recognition from the command line."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--references", type=Path, required=True)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--threshold", type=float, default=UNKNOWN_THRESHOLD)
    args = parser.parse_args()
    if not math.isfinite(args.threshold) or args.threshold <= 0:
        parser.error("--threshold must be finite and positive")

    prediction, matches, skipped = recognize_recording(
        args.recording,
        args.references,
        args.model,
        args.threshold,
    )
    print(f"Prediction: {prediction}")
    print("Closest references:")
    for match in matches[:5]:
        print(f"  {match.subject_id}: {match.path.name} ({match.distance:.4f})")
    if skipped:
        print(f"Skipped {skipped} invalid JSON file(s)")


if __name__ == "__main__":
    try:
        main()
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        raise SystemExit(f"Error: {error}") from error
