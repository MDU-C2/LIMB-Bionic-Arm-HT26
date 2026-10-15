# Majd
"""Guide a grip trial and save labels beside the raw recording."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

PHASES = (("rest", 3.0), ("grip", 0.5), ("hold", 3.0), ("release", 0.5))
PROMPTS = {
    "rest": "Relax your hand",
    "grip": "Close your hand",
    "hold": "Keep holding",
    "release": "Open your hand",
}


class GripTrial:
    """Keep cue times on the same clock as the recorded samples."""

    def __init__(self, repeats: int = 5) -> None:
        if not 1 <= repeats <= 20:
            raise ValueError("Choose 1 to 20 grip repeats.")
        self.repeats = repeats
        self.steps = [(trial, phase, seconds)
                      for trial in range(1, repeats + 1) for phase, seconds in PHASES]
        self.steps.append((repeats, "rest", 3.0))
        self.segments: list[dict] = []
        self.start_ns: int | None = None
        self.complete = False

    @property
    def duration(self) -> float:
        return sum(seconds for _trial, _phase, seconds in self.steps)

    def advance(self, now_ns: int) -> str | None:
        if self.complete:
            return None
        if self.start_ns is None:
            self.start_ns = now_ns
        elapsed = (now_ns - self.start_ns) / 1e9
        if self.segments:
            current = self.segments[-1]
            if elapsed - current["start_s"] < self.steps[len(self.segments) - 1][2]:
                return None
            current["end_s"] = elapsed
        if len(self.segments) == len(self.steps):
            self.complete = True
            return "Grip trial finished"
        trial, phase, seconds = self.steps[len(self.segments)]
        self.segments.append({"trial": trial, "phase": phase, "start_s": elapsed})
        # Use the time the cue was shown, even if the event loop was delayed.
        return f"Trial {trial}/{self.repeats}: {PROMPTS[phase]} ({seconds:g} s)"

    def save(self, session: Path, now_ns: int) -> dict:
        if self.segments and "end_s" not in self.segments[-1]:
            self.segments[-1]["end_s"] = (now_ns - self.start_ns) / 1e9
        labels = {
            "start_host_monotonic_ns": self.start_ns,
            "label_source": "cue",
            "complete": self.complete,
            "repeats": self.repeats,
            "segments": self.segments,
        }
        path = session / "emg_trial.json"
        path.write_text(json.dumps(labels, indent=2) + "\n", encoding="utf-8")
        result = {"complete": self.complete, "labels": path.name, "features_saved": False}
        if self.segments:
            try:
                result.update(export_features(session / "emg.csv", path))
                rest = result["median_rms_adc"]["rest"]
                hold = result["median_rms_adc"]["hold"]
                if rest is not None and hold is not None:
                    print(f"[CAPTURE] Rest {rest:.1f} | hold {hold:.1f} RMS ADC")
                if result["adc_clipped_fraction"] > 0.05:
                    print("[CAPTURE] EMG is clipping. Check the gain and repeat.")
            except (ImportError, OSError, ValueError) as error:
                result["feature_error"] = str(error)
                print(f"[CAPTURE] Raw trial saved; features unavailable: {error}")
        return result


def export_features(emg_file: Path, labels_file: Path) -> dict:
    """Keep complete windows inside one phase and preserve their amplitude."""
    import numpy as np
    from emg_processing import load_emg, preprocess_emg, sample_rate, window_features

    labels = json.loads(labels_file.read_text(encoding="utf-8"))
    if not isinstance(labels, dict) or not isinstance(labels.get("segments"), list):
        raise ValueError("Trial labels need a list of phase segments.")
    origin = labels.get("start_host_monotonic_ns")
    if not isinstance(origin, int) or origin <= 0:
        raise ValueError("The trial has no cue start time.")
    previous_end = 0.0
    for segment in labels["segments"]:
        try:
            start, end = float(segment["start_s"]), float(segment["end_s"])
            if segment["trial"] < 1 or segment["phase"] not in PROMPTS:
                raise ValueError("Invalid phase or trial number.")
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("Each phase needs trial, phase, start_s and end_s values.") from error
        if (segment["phase"] not in PROMPTS or not math.isfinite(start)
                or not math.isfinite(end) or start < previous_end or end <= start):
            raise ValueError("Trial labels must have ordered, non-overlapping phase times.")
        previous_end = end

    device_times, raw = load_emg(emg_file)
    rate = sample_rate(device_times)
    with emg_file.open(newline="", encoding="utf-8") as source:
        synced = [row.get("synced_host_monotonic_ns", "") for row in csv.DictReader(source)]
    if len(synced) != len(raw) or any(not value for value in synced):
        raise ValueError("Grip labels need clock-synchronized EMG samples.")
    host_times = np.asarray(synced, dtype=np.int64)
    if np.any(host_times <= 0) or np.any(np.diff(host_times) <= 0):
        raise ValueError("Synced sample times must increase.")
    filtered = preprocess_emg(raw, rate)
    starts, rms, mav = window_features(filtered, rate)
    size = int(rate * 0.2)
    output = emg_file.with_name("emg_trial_features.csv")
    counts = dict.fromkeys(PROMPTS, 0)
    levels: dict[str, list[float]] = {phase: [] for phase in PROMPTS}
    rows = []
    for segment in labels["segments"]:
        # Leave a small margin around each cue for reaction time.
        begin = origin + round((segment["start_s"] + 0.05) * 1e9)
        end = origin + round((segment["end_s"] - 0.05) * 1e9)
        selected = np.flatnonzero(
            (host_times[starts] >= begin) & (host_times[starts + size - 1] < end)
        )
        for index in selected:
            phase = segment["phase"]
            rows.append((emg_file.parent.name, segment["trial"], phase,
                         device_times[starts[index]], rms[index], mav[index]))
            counts[phase] += 1
            levels[phase].append(float(rms[index]))
    if not rows or (labels.get("complete") and not all(counts.values())):
        raise ValueError("Not enough recorded samples inside the labelled phases.")
    with output.open("w", newline="", encoding="utf-8") as destination:
        writer = csv.writer(destination)
        writer.writerow(("session", "trial", "phase", "timestamp_us", "rms", "mav"))
        writer.writerows(rows)
    medians = {phase: float(np.median(values)) if values else None
               for phase, values in levels.items()}
    rest, hold = medians["rest"], medians["hold"]
    return {
        "features_saved": True,
        "features": output.name,
        "label_source": labels.get("label_source", "cue"),
        "sample_rate_hz": rate,
        "adc_clipped_fraction": float(np.mean((raw <= 2) | (raw >= 4093))),
        "windows_by_phase": counts,
        "median_rms_adc": medians,
        "hold_rest_ratio": hold / rest if hold is not None and rest else None,
    }
