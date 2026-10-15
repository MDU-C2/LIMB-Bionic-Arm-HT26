# Johan Sandred, Majd
"""Filter recorded EMG and calculate RMS and mean absolute value."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from plot_sensor_recording import iter_recording
from scipy.signal import butter, filtfilt, iirnotch


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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, required=True, help="USB recording.csv or BLE emg.csv")
    parser.add_argument("--lowcut", type=float, default=20.0)
    parser.add_argument("--highcut", type=float, default=450.0)
    parser.add_argument("--notch", type=float, default=50.0)
    parser.add_argument("--window-ms", type=float, default=200.0)
    parser.add_argument("--overlap-ms", type=float, default=50.0)
    parser.add_argument("--labels", type=Path, help="Rebuild grip features from edited trial labels.")
    args = parser.parse_args()
    try:
        if args.labels is not None:
            from emg_trials import export_features

            summary = export_features(args.file, args.labels)
            metadata_file = args.file.with_name("meta.json")
            if metadata_file.is_file():
                metadata = json.loads(metadata_file.read_text(encoding="utf-8"))
                if isinstance(metadata.get("grip_trial"), dict):
                    metadata["grip_trial"].update(summary)
                    metadata_file.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
            print(f"Saved {summary['features']}: {summary['windows_by_phase']}")
            return 0
        times, raw = load_emg(args.file)
        rate_hz = sample_rate(times)
        filtered = preprocess_emg(raw, rate_hz, args.lowcut, args.highcut, args.notch)
        starts, rms, mav = window_features(filtered, rate_hz, args.window_ms, args.overlap_ms)
        filtered_path = args.file.with_suffix(".filtered.csv")
        features_path = args.file.with_suffix(".features.csv")
        metadata_path = args.file.with_suffix(".processing.json")
        # Keep the source recording untouched so the filters can be changed later.
        with filtered_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(("timestamp_us", "emg_raw", "emg_filtered"))
            writer.writerows(zip(times, raw, filtered, strict=True))
        with features_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(("timestamp_us", "rms", "mav"))
            writer.writerows(zip(times[starts], rms, mav, strict=True))
        metadata_path.write_text(
            json.dumps(
                {
                    "source": str(args.file),
                    "sample_rate_hz": rate_hz,
                    "lowcut_hz": args.lowcut,
                    "highcut_hz": args.highcut,
                    "notch_hz": args.notch,
                    "filter_order": 4,
                    "notch_quality": 30,
                    "window_ms": args.window_ms,
                    "overlap_ms": args.overlap_ms,
                    "samples": len(raw),
                    "windows": len(starts),
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError) as error:
        print(f"EMG processing failed: {error}")
        return 1
    print(f"Processed {len(raw)} samples at {rate_hz:g} Hz")
    print(f"Saved {filtered_path}, {features_path} and {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
