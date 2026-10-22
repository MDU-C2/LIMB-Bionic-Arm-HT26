# Majd
"""The same causal EMG windows for training and live prediction."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass

import numpy as np

from aurora.sensors.emg_filter import RawEmgFilter

FEATURE_NAMES = ("rms", "mav", "waveform_length", "zero_crossings", "slope_changes", "variance")


@dataclass(frozen=True)
class EmgWindowSettings:
    rate_hz: float = 4000.0
    window_ms: float = 200.0
    overlap_ms: float = 50.0
    settling_ms: float = 250.0
    noise_threshold_adc: float = 5.0

    def __post_init__(self):
        values = np.asarray(list(asdict(self).values()), dtype=float)
        if not np.all(np.isfinite(values)) or not 1000 <= self.rate_hz <= 10000:
            raise ValueError("Causal EMG needs a finite rate between 1000 and 10000 Hz.")
        if not 0 <= self.overlap_ms < self.window_ms or self.window_ms <= 0:
            raise ValueError("Need 0 <= overlap < window duration.")
        if self.settling_ms < 250 or self.noise_threshold_adc < 0:
            raise ValueError("Allow at least 250 ms settling and a non-negative noise threshold.")
        if self.size < 3 or self.step < 1:
            raise ValueError("EMG windows need at least three samples and a positive step.")

    @property
    def size(self) -> int:
        return round(self.rate_hz * self.window_ms / 1000)

    @property
    def step(self) -> int:
        return self.size - round(self.rate_hz * self.overlap_ms / 1000)


def features(values: np.ndarray, threshold: float) -> np.ndarray:
    """HT25 time-domain features, with explicit ADC noise thresholds."""
    values = np.asarray(values, dtype=float)
    differences = np.diff(values)
    crossing = (values[:-1] * values[1:] < 0) & (np.abs(differences) >= threshold)
    slopes = (differences[:-1] * differences[1:] < 0) & (
        np.minimum(np.abs(differences[:-1]), np.abs(differences[1:])) >= threshold
    )
    return np.asarray(
        [
            np.sqrt(np.mean(values**2)),
            np.mean(np.abs(values)),
            np.sum(np.abs(differences)),
            np.count_nonzero(crossing),
            np.count_nonzero(slopes),
            np.var(values, ddof=1),
        ],
        dtype=np.float64,
    )


class EmgWindowStream:
    def __init__(self, settings: EmgWindowSettings):
        self.settings = settings
        self.reset()

    def reset(self) -> None:
        self.filter = RawEmgFilter(self.settings.rate_hz)
        self.values: deque = deque(maxlen=self.settings.size)
        self.clipped: deque = deque(maxlen=self.settings.size)
        self.count = 0

    def update(self, raw: float) -> tuple[int, np.ndarray | None] | None:
        if not np.isfinite(raw) or not 0 <= raw <= 4095:
            self.reset()
            return None
        value = self.filter.update(float(raw))
        self.count += 1
        settling = round(self.settings.rate_hz * self.settings.settling_ms / 1000)
        if self.count <= settling:
            return None
        self.values.append(value)
        self.clipped.append(raw <= 2 or raw >= 4093)
        elapsed = self.count - settling - self.settings.size
        if elapsed < 0 or elapsed % self.settings.step:
            return None
        result = None
        if np.mean(self.clipped) <= 0.05:
            result = features(np.asarray(self.values), self.settings.noise_threshold_adc)
        return self.count - self.settings.size, result


def preprocessing(settings: EmgWindowSettings) -> dict:
    return {
        "version": 1,
        "settings": asdict(settings),
        "features": list(FEATURE_NAMES),
        "filter": "causal_hp4_20_lp4_450_notch50_q30",
        "initial_state": "constant_first_sample",
        "clipping_fraction_limit": 0.05,
    }
