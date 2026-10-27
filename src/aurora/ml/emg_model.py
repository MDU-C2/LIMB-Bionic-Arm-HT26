# Majd
"""Portable EMG classifiers. Live use needs no training framework."""

from __future__ import annotations

import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

from aurora.json_utils import read_json_object
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
        self.manifest = read_json_object(self.directory / "model.json")
        info = self.manifest
        if (
            info.get("format_version") != 1
            or info.get("task") != "emg_phase_classification"
            or info.get("classes") != list(PHASE_NAMES)
            or (require_review and info.get("reviewed_for_live") is not True)
        ):
            raise ValueError("Select a reviewed EMG model bundle with the four phase labels.")
        try:
            self.settings = EmgWindowSettings(**info["preprocessing"]["settings"])
        except (KeyError, TypeError) as error:
            raise ValueError("Model bundle needs valid preprocessing settings.") from error
        if info["preprocessing"] != preprocessing(self.settings):
            raise ValueError("Model preprocessing is incompatible with this version.")
        artifacts = info.get("artifacts")
        if not isinstance(artifacts, dict) or "weights.npz" not in artifacts:
            raise ValueError("Model bundle needs hashed scaling weights.")
        for name, expected in artifacts.items():
            if not isinstance(name, str) or Path(name).name != name or "/" in name or "\\" in name:
                raise ValueError("Model artifacts must be files inside the bundle.")
            path = self.directory / name
            if path.parent.resolve() != self.directory.resolve() or digest(path) != expected:
                raise ValueError("Model artifact hash does not match model.json.")
        self.sequence_length = info.get("sequence_length")
        feature_indices = info.get("feature_indices")
        if (
            type(self.sequence_length) is not int
            or not 1 <= self.sequence_length <= 20
            or not isinstance(feature_indices, list)
            or not all(type(index) is int for index in feature_indices)
            or feature_indices not in [[0, 1], list(range(6))]
        ):
            raise ValueError("Invalid model feature or sequence settings.")
        self.feature_indices = np.asarray(feature_indices, dtype=int)
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
            self.kind = info.get("kind")
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
                try:
                    self.session = ort.InferenceSession(
                        str(self.directory / "model.onnx"),
                        sess_options=options,
                        providers=["CPUExecutionProvider"],
                    )
                except Exception as error:
                    raise ValueError(f"Could not load the EMG ONNX model: {error}") from error
                inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
                if (
                    len(inputs) != 1
                    or inputs[0].name != "features"
                    or inputs[0].type != "tensor(float)"
                    or inputs[0].shape[1:] != [self.sequence_length, len(self.mean)]
                    or len(outputs) != 1
                    or outputs[0].shape[1:] != [4]
                ):
                    raise ValueError("ONNX input/output shapes do not match the EMG bundle.")
            else:
                raise ValueError("Unknown EMG model kind or missing model.onnx.")

    def predict(self, windows: np.ndarray) -> np.ndarray:
        x = np.asarray(windows, dtype=np.float64)
        if (
            x.ndim != 3
            or not len(x)
            or x.shape[1:] != (self.sequence_length, 6)
            or not np.isfinite(x).all()
        ):
            raise ValueError("Prediction needs complete finite causal feature sequences.")
        scaled = (x[:, :, self.feature_indices] - self.mean) / self.scale
        if self.kind == "lda":
            logits = scaled[:, -1] @ self.weights.T + self.bias
        else:
            logits = self.session.run(None, {"features": scaled.astype(np.float32)})[0]
        if logits.shape != (len(x), 4) or not np.isfinite(logits).all():
            raise ValueError("EMG model returned invalid phase scores.")
        return probabilities(logits)


class LiveEmgClassifier:
    def __init__(self, directory: Path, *, background=False):
        self.model = EmgModel(directory)
        self.background = background
        self.condition = threading.Condition()
        self.worker = None
        self.stopping = False
        self.generation = 0
        self.pending = None
        self.skipped_windows = 0
        self.error = ""
        self.reset()

    def reset(self) -> None:
        self.stream = EmgWindowStream(self.model.settings)
        self.history: deque = deque(maxlen=self.model.sequence_length)
        with self.condition:
            self.generation += 1
            self.pending = None
            self._prediction = (None, None, 0.0)

    @property
    def phase(self):
        phase, _confidence, stamp = self._prediction
        return phase if not self.background or time.monotonic() - stamp <= 0.5 else None

    @property
    def confidence(self):
        return self._prediction[1] if self.phase is not None else None

    def _predict(self, features, generation, stamp):
        try:
            scores = self.model.predict(features)[0]
            prediction = (PHASE_NAMES[int(np.argmax(scores))], float(np.max(scores)), stamp)
            error = ""
        except Exception as failure:
            prediction, error = (None, None, stamp), f"EMG model inference failed: {failure}"
        with self.condition:
            if generation == self.generation and not self.stopping:
                self._prediction, self.error = prediction, error

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.stopping or self.pending is not None)
                if self.stopping:
                    return
                features, generation, stamp = self.pending
                self.pending = None
            self._predict(features, generation, stamp)

    def close(self, timeout=3.0):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify_all()
        if self.worker is not None:
            self.worker.join(timeout)
            if self.worker.is_alive():
                return False
            self.worker = None
        return True

    def reopen(self):
        if self.worker is not None and self.worker.is_alive():
            raise RuntimeError("Previous EMG inference worker is still stopping")
        self.stopping = False
        self.reset()

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
            with self.condition:
                self.generation += 1
                self.pending = None
                self._prediction = (None, None, 0.0)
            return
        self.history.append(vector)
        if len(self.history) == self.model.sequence_length:
            features, stamp = np.asarray([self.history]), time.monotonic()
            if not self.background:
                self._predict(features, self.generation, stamp)
                return
            with self.condition:
                if self.stopping:
                    return
                if self.pending is not None:
                    self.skipped_windows += 1
                self.pending = features, self.generation, stamp
                if self.worker is None:
                    self.worker = threading.Thread(target=self._run, daemon=True)
                    self.worker.start()
                self.condition.notify()

    def grip(self, activation: float | None) -> float | None:
        # Phase describes intent; the calibrated amplitude still sets grip strength.
        if self.phase is None or activation is None:
            return None
        return activation if self.phase in ("grip", "hold") else 0.0
