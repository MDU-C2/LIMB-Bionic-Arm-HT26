# Majd
"""Reviewed intervals beside a raw recording. Cue files stay unchanged."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from aurora.json_utils import write_json
from aurora.recording.emg_features import PHASE_NAMES, load_emg
from aurora.time_utils import utc_now


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_segments(segments: list, duration: float) -> list[dict]:
    if not isinstance(segments, list) or not segments:
        raise ValueError("Add at least one reviewed interval.")
    result = []
    previous = 0.0
    for item in segments:
        try:
            start, end = float(item["start_s"]), float(item["end_s"])
            phase = item["phase"]
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("Each interval needs start_s, end_s and phase.") from error
        if (
            not np.isfinite([start, end]).all()
            or start < previous
            or end <= start
            or end > duration + 1e-6
            or phase not in PHASE_NAMES
        ):
            raise ValueError("Intervals must be ordered, within the recording and non-overlapping.")
        result.append({"start_s": start, "end_s": end, "phase": phase})
        previous = end
    return result


def save_labels(
    source: Path,
    subject: str,
    trial: str,
    segments: list,
    *,
    reviewed: bool,
    destination: Path | None = None,
) -> Path:
    times, _raw = load_emg(source)
    if not subject.strip() or not trial.strip():
        raise ValueError("Subject and trial IDs are required.")
    duration = (times[-1] - times[0]) / 1e6
    document = {
        "format_version": 1,
        "task": "emg_phase_labels",
        "reviewed": bool(reviewed),
        "label_source": "manual_review",
        "subject_id": subject.strip(),
        "trial_id": trial.strip(),
        "source": source.name,
        "source_sha256": digest(source),
        "origin_device_us": float(times[0]),
        "time_reference": "seconds_from_first_emg_sample",
        "reviewed_utc": utc_now() if reviewed else None,
        "segments": validate_segments(segments, duration),
    }
    path = destination or source.with_name(source.stem + ".labels.json")
    if path.resolve() == source.resolve() or path.name in {"emg_trial.json", "trial.json"}:
        raise ValueError("Save reviewed labels in a separate .labels.json file.")
    if path.parent.resolve() != source.parent.resolve() or not path.name.endswith(".labels.json"):
        raise ValueError("Keep the label file beside its raw source with a .labels.json suffix.")
    write_json(path, document)
    return path


def load_labels(path: Path) -> tuple[dict, Path, np.ndarray, np.ndarray]:
    labels = json.loads(path.read_text(encoding="utf-8"))
    if (
        labels.get("format_version") != 1
        or labels.get("task") != "emg_phase_labels"
        or labels.get("reviewed") is not True
        or labels.get("time_reference") != "seconds_from_first_emg_sample"
    ):
        raise ValueError(f"Review and save the EMG labels first: {path.name}")
    source = path.parent / labels["source"]
    if (
        source.parent.resolve() != path.parent.resolve()
        or digest(source) != labels["source_sha256"]
    ):
        raise ValueError("The reviewed labels do not match the raw source.")
    times, raw = load_emg(source)
    if labels["origin_device_us"] != times[0]:
        raise ValueError("The label clock does not match the raw source.")
    labels["segments"] = validate_segments(labels["segments"], (times[-1] - times[0]) / 1e6)
    for key in ("subject_id", "trial_id"):
        if not isinstance(labels.get(key), str) or not labels[key].strip():
            raise ValueError(f"Labels need a {key}.")
    return labels, source, times, raw
