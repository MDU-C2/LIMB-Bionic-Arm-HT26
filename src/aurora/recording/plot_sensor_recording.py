# Jonas Gerne, Majd
"""Inspect saved raw signals, camera angles and processed EMG."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from aurora.recording.common import DEFAULT_OUTPUT
from aurora.recording.saved_signals import load_signals, with_gaps
from aurora.recording.sensor_csv import load_recording as load_recording


def latest_recording(root: Path = DEFAULT_OUTPUT) -> Path | None:
    candidates = (
        [path for name in ("trial.json", "emg.csv", "recording.csv") for path in root.rglob(name)]
        if root.is_dir()
        else []
    )
    return max(candidates, key=lambda path: path.stat().st_mtime, default=None)


def make_figure(path: Path, sensor: str = "all", spectrum: bool = False):
    import matplotlib.pyplot as plt

    data = load_signals(path, sensor)
    spectral = []
    if spectrum:
        from scipy.signal import welch

        from aurora.recording.emg_features import sample_rate

        for panel in data.panels:
            if panel.title != "Raw EMG":
                continue
            for trace in panel.traces:
                if len(trace.values) < 2:
                    continue
                if trace.sample_time_s is None:
                    raise ValueError("The raw EMG spectrum needs device sample timestamps.")
                rate = sample_rate(trace.sample_time_s * 1e6)
                size = min(4096, len(trace.values))
                frequency, power = welch(
                    trace.values,
                    fs=rate,
                    window="hann",
                    nperseg=size,
                    noverlap=size // 2,
                    detrend="constant",
                    scaling="density",
                )
                spectral.append((trace.label, frequency, power))
        if not spectral:
            raise ValueError("The spectrum needs a regularly sampled raw EMG recording.")
    count = len(data.panels) + bool(spectral)
    figure, axes = plt.subplots(count, 1, figsize=(13, min(9.5, 2.4 * count + 1)), squeeze=False)
    shared = {}
    for axis, panel in zip(axes.flat, data.panels, strict=False):
        if panel.clock in shared:
            axis.sharex(shared[panel.clock])
        else:
            shared[panel.clock] = axis
        for trace in panel.traces:
            axis.plot(*with_gaps(trace), label=trace.label, linewidth=0.8)
            print(f"{panel.title} / {trace.label}: {len(trace.values)} samples")
        axis.set_title(panel.title, fontsize=10)
        axis.set_ylabel(panel.unit)
        axis.set_xlabel(panel.clock)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right", fontsize=8)
        if panel.clock == "Time from trial start [s]":
            for time_s, label in data.events:
                axis.axvline(time_s, color="grey", linestyle="--", linewidth=0.8)
                axis.text(
                    time_s, 1, label, transform=axis.get_xaxis_transform(), fontsize=7, va="top"
                )
    if spectral:
        axis = axes[-1, 0]
        for label, frequency, power in spectral:
            axis.plot(
                frequency, 10 * np.log10(np.maximum(power, np.finfo(float).tiny)), label=label
            )
        axis.set_title("Raw EMG power spectrum (Welch)", fontsize=10)
        axis.set_xlabel("Frequency [Hz]")
        axis.set_ylabel("dB (ADC counts²/Hz)")
        axis.grid(True, alpha=0.3)
        axis.legend()
    figure.suptitle(data.title, fontsize=11)
    figure.tight_layout()
    return figure


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--file", type=Path, help="Trial folder, trial.json, pose.json or sensor CSV"
    )
    parser.add_argument("--sensor", choices=("all", "emg", "imu1", "imu2", "camera"), default="all")
    parser.add_argument(
        "--spectrum",
        action="store_true",
        help="Add raw EMG Welch spectrum; requires regular samples",
    )
    parser.add_argument("--save", type=Path, help="Save the figure instead of opening a window")
    args = parser.parse_args(argv)
    path = args.file or latest_recording()
    if path is None:
        print("No saved recording was found. Pass --file PATH.")
        return 2
    import matplotlib.pyplot as plt

    figure = None
    try:
        figure = make_figure(path, args.sensor, args.spectrum)
        if args.save:
            args.save.parent.mkdir(parents=True, exist_ok=True)
            figure.savefig(args.save, dpi=150)
            print(f"Saved {args.save}")
        else:
            plt.show()
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"Could not plot recording: {error}")
        return 1
    finally:
        if figure is not None:
            plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
