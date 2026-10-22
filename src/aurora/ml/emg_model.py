# Majd
"""Portable EMG classifiers. Live use needs no training framework."""

from __future__ import annotations

import json
from collections import deque
from pathlib import Path

import numpy as np

from aurora.recording.emg_features import PHASE_NAMES
from aurora.recording.emg_labels import digest
from aurora.sensors.emg_window import EmgWindowSettings, EmgWindowStream, preprocessing


def fit_lda(x: np.ndarray, y: np.ndarray, classes: int = 4) -> tuple[np.ndarray, np.ndarray]:
    means = np.asarray([x[y == label].mean(axis=0) for label in range(classes)])
    residuals = x - means[y]
    covariance = residuals.T @ residuals / max(1, len(x) - classes)
    # Fixed shrinkage is recorded; it is never selected on the test participant.
    covariance = 0.9 * covariance + 0.1 * np.eye(x.shape[1]) * max(
        float(np.trace(covariance) / x.shape[1]), 1e-6
    )
    weights = np.linalg.solve(covariance, means.T).T
    # Equal priors prevent the much longer rest/hold intervals dominating by count.
    bias = -0.5 * np.sum(means * weights, axis=1)
    return weights, bias


def probabilities(logits: np.ndarray) -> np.ndarray:
    values = np.exp(logits - np.max(logits, axis=-1, keepdims=True))
    return values / values.sum(axis=-1, keepdims=True)


class EmgModel:
    def __init__(self, directory: Path, *, require_review: bool = True):
        self.directory = Path(directory)
        self.manifest = json.loads((self.directory / "model.json").read_text(encoding="utf-8"))
        info = self.manifest
        if (
            info.get("format_version") != 1
            or info.get("task") != "emg_phase_classification"
            or info.get("classes") != list(PHASE_NAMES)
            or (require_review and info.get("reviewed_for_live") is not True)
        ):
            raise ValueError("Select a reviewed EMG model bundle with the four phase labels.")
        self.settings = EmgWindowSettings(**info["preprocessing"]["settings"])
        if info["preprocessing"] != preprocessing(self.settings):
            raise ValueError("Model preprocessing is incompatible with this version.")
        for name, expected in info["artifacts"].items():
            path = self.directory / name
            if path.parent.resolve() != self.directory.resolve() or digest(path) != expected:
                raise ValueError("Model artifact hash does not match model.json.")
        self.sequence_length = info["sequence_length"]
        self.feature_indices = np.asarray(info["feature_indices"], dtype=int)
        if (
            not isinstance(self.sequence_length, int)
            or not 1 <= self.sequence_length <= 20
            or self.feature_indices.tolist() not in [[0, 1], list(range(6))]
        ):
            raise ValueError("Invalid model feature or sequence settings.")
        if "weights.npz" not in info["artifacts"]:
            raise ValueError("Model bundle needs frozen scaling weights.")
        with np.load(self.directory / "weights.npz", allow_pickle=False) as weights:
            self.mean, self.scale = weights["mean"], weights["scale"]
            if (
                self.mean.shape != (len(self.feature_indices),)
                or self.scale.shape != self.mean.shape
                or not np.isfinite(self.mean).all()
                or not np.isfinite(self.scale).all()
                or np.any(self.scale <= 0)
            ):
                raise ValueError("Invalid model scaling weights.")
            self.kind = info["kind"]
            if self.kind == "lda":
                self.weights, self.bias = weights["weights"], weights["bias"]
                if (
                    self.weights.shape != (4, len(self.mean))
                    or self.bias.shape != (4,)
                    or not np.isfinite(self.weights).all()
                    or not np.isfinite(self.bias).all()
                ):
                    raise ValueError("Invalid LDA weights.")
            elif self.kind in ("ann", "lstm") and "model.onnx" in info["artifacts"]:
                import onnxruntime as ort

                options = ort.SessionOptions()
                options.intra_op_num_threads = 1
                options.inter_op_num_threads = 1
                self.session = ort.InferenceSession(
                    str(self.directory / "model.onnx"),
                    sess_options=options,
                    providers=["CPUExecutionProvider"],
                )
            else:
                raise ValueError("Unknown EMG model kind or missing model.onnx.")

    def predict(self, windows: np.ndarray) -> np.ndarray:
        x = np.asarray(windows, dtype=np.float64)
        if x.ndim != 3 or x.shape[1:] != (self.sequence_length, 6) or not np.isfinite(x).all():
            raise ValueError("Prediction needs complete finite causal feature sequences.")
        scaled = (x[:, :, self.feature_indices] - self.mean) / self.scale
        if self.kind == "lda":
            logits = scaled[:, -1] @ self.weights.T + self.bias
        else:
            logits = self.session.run(None, {"features": scaled.astype(np.float32)})[0]
        return probabilities(logits)


class LiveEmgClassifier:
    def __init__(self, directory: Path):
        self.model = EmgModel(directory)
        self.reset()

    def reset(self) -> None:
        self.stream = EmgWindowStream(self.model.settings)
        self.history: deque = deque(maxlen=self.model.sequence_length)
        self.phase: str | None = None
        self.confidence: float | None = None

    def update(self, raw: float) -> None:
        if not np.isfinite(raw) or not 0 <= raw <= 4095:
            self.reset()
            return
        emitted = self.stream.update(raw)
        if emitted is None:
            return
        _start, vector = emitted
        if vector is None:
            self.history.clear()
            self.phase = self.confidence = None
            return
        self.history.append(vector)
        if len(self.history) == self.model.sequence_length:
            scores = self.model.predict(np.asarray([self.history]))[0]
            self.phase = PHASE_NAMES[int(np.argmax(scores))]
            self.confidence = float(np.max(scores))

    def grip(self, activation: float | None) -> float | None:
        # Phase describes intent; the calibrated amplitude still sets grip strength.
        if self.phase is None or activation is None:
            return None
        return activation if self.phase in ("grip", "hold") else 0.0
