# Johan Sandred, Majd
"""Process saved EMG and export window features with reviewed cue labels."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt, iirnotch

from aurora.recording.sensor_csv import iter_recording

PHASE_NAMES = ("rest", "grip", "hold", "release")


def load_emg(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read the single EMG channel from USB or timestamped BLE recordings."""
    times, values = [], []
    for row in iter_recording(path):
        value = row.get("emg_raw", row.get("value"))
        if value is None:
            continue  # The aligned USB CSV also has rows containing only IMU data.
        if row.get("channel", 0) != 0:
            raise ValueError("Expected a single EMG channel (channel 0)")
        timestamp = row.get("timestamp_us", row.get("device_time"))
        if timestamp is None:
            raise ValueError("EMG needs a device timestamp for every sample")
        if not np.isfinite(value) or not 0 <= value <= 4095:
            raise ValueError("EMG samples must be finite 12-bit ADC values")
        times.append(timestamp)
        values.append(value)
    if not values:
        raise ValueError("No EMG samples found in the recording")
    return np.asarray(times, dtype=np.float64), np.asarray(values, dtype=np.float64)


def sample_rate(timestamps_us: np.ndarray) -> float:
    """Get the rate from device times and reject gaps or repeated timestamps."""
    times = np.asarray(timestamps_us, dtype=np.float64)
    if times.ndim != 1 or len(times) < 2 or not np.all(np.isfinite(times)):
        raise ValueError("At least two finite device timestamps are needed")
    intervals = np.diff(times)
    period_us = float(np.median(intervals))
    if np.any(intervals <= 0) or not np.allclose(intervals, period_us, rtol=0.01, atol=1.0):
        raise ValueError(
            "EMG timestamps must be increasing and regularly spaced; select a continuous recording"
        )
    return 1_000_000.0 / period_us


def preprocess_emg(
    samples: np.ndarray,
    rate_hz: float,
    lowcut_hz: float = 20.0,
    highcut_hz: float = 450.0,
    notch_hz: float = 50.0,
) -> np.ndarray:
    """Remove the offset, then apply a band-pass and mains notch."""
    values = np.asarray(samples, dtype=np.float64)
    if values.ndim != 1 or not np.all(np.isfinite(values)):
        raise ValueError("EMG must be a finite, one-dimensional signal")
    if not np.isfinite(rate_hz) or not 0 < lowcut_hz < highcut_hz < rate_hz / 2:
        raise ValueError(
            f"Need 0 < lowcut < highcut < Nyquist ({rate_hz / 2:g} Hz). "
            "Use full-rate BLE data for the 20–450 Hz band, or choose --highcut explicitly."
        )
    if not 0 < notch_hz < rate_hz / 2:
        raise ValueError("The notch frequency must be below Nyquist")
    band_b, band_a = butter(4, [lowcut_hz, highcut_hz], btype="bandpass", fs=rate_hz)
    notch_b, notch_a = iirnotch(notch_hz, 30, fs=rate_hz)
    # filtfilt needs both ends of the recording; this processing is offline.
    padding = 3 * max(len(band_a), len(band_b), len(notch_a), len(notch_b))
    if len(values) <= padding:
        raise ValueError(f"Need more than {padding} samples for zero-phase filtering")
    centered = values - values.mean()
    filtered = filtfilt(band_b, band_a, centered)
    return filtfilt(notch_b, notch_a, filtered)


def window_features(
    filtered: np.ndarray,
    rate_hz: float,
    window_ms: float = 200.0,
    overlap_ms: float = 50.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Calculate RMS and mean absolute value for complete windows."""
    values = np.asarray(filtered, dtype=np.float64)
    if values.ndim != 1 or not np.all(np.isfinite(values)):
        raise ValueError("Features need a finite, one-dimensional signal")
    if not np.isfinite(rate_hz) or rate_hz <= 0 or not 0 <= overlap_ms < window_ms:
        raise ValueError("Need a positive rate and 0 <= overlap < window duration")
    if not np.isfinite(window_ms) or not np.isfinite(overlap_ms):
        raise ValueError("Window durations must be finite")
    size = int(rate_hz * window_ms / 1000)
    step = size - int(rate_hz * overlap_ms / 1000)
    if size < 1 or step < 1 or len(values) < size:
        raise ValueError("Recording is shorter than one feature window")
    # Use a view instead of copying every overlapping window.
    windows = np.lib.stride_tricks.sliding_window_view(values, size)[::step]
    starts = np.arange(len(windows)) * step
    rms = np.sqrt(np.mean(windows**2, axis=1))
    mav = np.mean(np.abs(windows), axis=1)
    return starts, rms, mav


def export_features(
    emg_file: Path,
    labels_file: Path,
    *,
    lowcut_hz: float = 20.0,
    highcut_hz: float = 450.0,
    notch_hz: float = 50.0,
    window_ms: float = 200.0,
    overlap_ms: float = 50.0,
) -> dict:
    """Keep complete windows inside one phase and preserve their amplitude."""
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
            if segment["trial"] < 1 or segment["phase"] not in PHASE_NAMES:
                raise ValueError("Invalid phase or trial number.")
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("Each phase needs trial, phase, start_s and end_s values.") from error
        if (
            segment["phase"] not in PHASE_NAMES
            or not math.isfinite(start)
            or not math.isfinite(end)
            or start < previous_end
            or end <= start
        ):
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
    filtered = preprocess_emg(raw, rate, lowcut_hz, highcut_hz, notch_hz)
    starts, rms, mav = window_features(filtered, rate, window_ms, overlap_ms)
    size = int(rate * window_ms / 1000)
    output = emg_file.with_name("emg_trial_features.csv")
    counts = dict.fromkeys(PHASE_NAMES, 0)
    levels: dict[str, list[float]] = {phase: [] for phase in PHASE_NAMES}
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
            rows.append(
                (
                    emg_file.parent.name,
                    segment["trial"],
                    phase,
                    device_times[starts[index]],
                    rms[index],
                    mav[index],
                )
            )
            counts[phase] += 1
            levels[phase].append(float(rms[index]))
    if not rows or (labels.get("complete") and not all(counts.values())):
        raise ValueError("Not enough recorded samples inside the labelled phases.")
    with output.open("w", newline="", encoding="utf-8") as destination:
        writer = csv.writer(destination)
        writer.writerow(("session", "trial", "phase", "timestamp_us", "rms", "mav"))
        writer.writerows(rows)
    medians = {
        phase: float(np.median(values)) if values else None for phase, values in levels.items()
    }
    rest, hold = medians["rest"], medians["hold"]
    return {
        "features_saved": True,
        "features": output.name,
        "label_source": labels.get("label_source", "cue"),
        "sample_rate_hz": rate,
        "samples": len(raw),
        "processing": {
            "lowcut_hz": lowcut_hz,
            "highcut_hz": highcut_hz,
            "notch_hz": notch_hz,
            "filter_order": 4,
            "filter_application": "forward_backward",
            "notch_quality": 30,
            "window_ms": window_ms,
            "overlap_ms": overlap_ms,
        },
        "adc_clipped_fraction": float(np.mean((raw <= 2) | (raw >= 4093))),
        "windows_by_phase": counts,
        "median_rms_adc": medians,
        "hold_rest_ratio": hold / rest if hold is not None and rest else None,
    }
