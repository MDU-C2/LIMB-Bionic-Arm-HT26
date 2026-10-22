# Majd
"""Participant splits and causal windows from reviewed EMG recordings."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from aurora.recording.emg_features import PHASE_NAMES
from aurora.recording.emg_labels import digest, load_labels
from aurora.sensors.emg_window import EmgWindowSettings, EmgWindowStream


@dataclass
class EmgTrial:
    label_file: Path
    source: Path
    labels: dict
    times: np.ndarray
    raw: np.ndarray

    @property
    def subject(self) -> str:
        return self.labels["subject_id"]

    @property
    def key(self) -> tuple[str, str]:
        return self.subject, self.labels["trial_id"]


def load_trials(root: Path) -> list[EmgTrial]:
    files = [root] if root.is_file() else sorted(root.rglob("*.labels.json"))
    trials = []
    keys, hashes = set(), set()
    for path in files:
        labels, source, times, raw = load_labels(path)
        trial = EmgTrial(path, source, labels, times, raw)
        source_hash = digest(source)
        if trial.key in keys or source_hash in hashes:
            raise ValueError("Each trial and raw recording may occur only once in the dataset.")
        keys.add(trial.key)
        hashes.add(source_hash)
        trials.append(trial)
    if not trials:
        raise ValueError("No reviewed .labels.json files found. Review recordings first.")
    return trials


def participant_splits(trials: list[EmgTrial], calibration_trials: int = 0) -> list[dict]:
    subjects = sorted({trial.subject for trial in trials})
    if len(subjects) < 2:
        raise ValueError("Participant evaluation needs at least two different subjects.")
    if calibration_trials < 0:
        raise ValueError("Calibration trial count cannot be negative.")
    folds = []
    for subject in subjects:
        # IDs are explicit; no windows from one trial can enter two partitions.
        held = sorted((t for t in trials if t.subject == subject), key=lambda t: t.key)
        calibration = held[:calibration_trials]
        testing = held[calibration_trials:]
        if not testing:
            raise ValueError(f"{subject} needs a separate test trial after personal calibration.")
        folds.append(
            {
                "subject": subject,
                "train": [t for t in trials if t.subject != subject] + calibration,
                "calibration": calibration,
                "test": testing,
            }
        )
    return folds


def trial_windows(
    trial: EmgTrial, settings: EmgWindowSettings, sequence_length: int
) -> tuple[np.ndarray, np.ndarray, list[dict]]:
    if not 1 <= sequence_length <= 20:
        raise ValueError("Sequence length must be between 1 and 20 windows.")
    stream = EmgWindowStream(settings)
    expected_us = 1e6 / settings.rate_hz
    intervals = np.diff(trial.times)
    if np.any(intervals <= 0) or not np.isclose(
        np.median(intervals), expected_us, rtol=0.01, atol=1
    ):
        raise ValueError(f"{trial.source.name}: sample times do not match {settings.rate_hz:g} Hz.")
    history = []
    x, y, records = [], [], []
    origin = trial.times[0]
    run_start = 0
    for index, (timestamp, raw) in enumerate(zip(trial.times, trial.raw, strict=True)):
        if index and abs(timestamp - trial.times[index - 1] - expected_us) > 1 + 0.01 * expected_us:
            stream.reset()
            history.clear()
            run_start = index
        emitted = stream.update(raw)
        if emitted is None:
            continue
        local_start, vector = emitted
        if vector is None:
            history.clear()
            continue
        history.append(vector)
        history = history[-sequence_length:]
        if len(history) != sequence_length:
            continue
        start_s = (trial.times[run_start + local_start] - origin) / 1e6
        end_s = (timestamp - origin) / 1e6
        segment = next(
            (s for s in trial.labels["segments"] if start_s >= s["start_s"] and end_s < s["end_s"]),
            None,
        )
        if segment is None:
            continue
        x.append(np.asarray(history))
        y.append(PHASE_NAMES.index(segment["phase"]))
        records.append(
            {
                "subject_id": trial.subject,
                "trial_id": trial.key[1],
                "start_s": start_s,
                "end_s": end_s,
                "segment_start_s": segment["start_s"],
                "sample_index": index,
            }
        )
    return (
        np.asarray(x, dtype=np.float64).reshape(-1, sequence_length, 6),
        np.asarray(y, dtype=np.int64),
        records,
    )


def make_dataset(
    trials: list[EmgTrial], settings: EmgWindowSettings, sequence_length: int
) -> tuple[np.ndarray, np.ndarray, list[dict]]:
    parts = [trial_windows(trial, settings, sequence_length) for trial in trials]
    x = np.concatenate([p[0] for p in parts])
    y = np.concatenate([p[1] for p in parts])
    if not len(y):
        raise ValueError("No complete, continuous labelled windows in this partition.")
    return x, y, [record for part in parts for record in part[2]]
