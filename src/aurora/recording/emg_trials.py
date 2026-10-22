# Majd
"""Guide a grip trial and save labels beside the raw recording."""

from __future__ import annotations

from pathlib import Path

from aurora.json_utils import write_json

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
        self.steps = [
            (trial, phase, seconds) for trial in range(1, repeats + 1) for phase, seconds in PHASES
        ]
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
        write_json(path, labels)
        result = {"complete": self.complete, "labels": path.name, "features_saved": False}
        if self.segments:
            try:
                from aurora.recording.emg_features import export_features

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
