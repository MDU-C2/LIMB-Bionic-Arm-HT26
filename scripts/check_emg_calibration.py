# Majd
"""Capture bounded BLE-only rest/grip trials with signal and packet diagnostics."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if os.name == "nt":
    os.environ["PATH"] = (
        str(Path(sys.prefix) / "Library/bin") + os.pathsep + os.environ.get("PATH", "")
    )


def describe(samples, period, np):
    values = np.asarray(samples, dtype=float)
    if len(values) < 2:
        return {"samples": len(values)}
    centered = values - values.mean()
    spectrum = abs(np.fft.rfft(centered)) ** 2
    frequencies = np.fft.rfftfreq(len(values), period)
    total = float(spectrum[1:].sum())
    return {
        "samples": len(values),
        "sample_rate_hz": 1 / period,
        "adc_min": float(values.min()),
        "adc_max": float(values.max()),
        "adc_mean": float(values.mean()),
        "raw_rms": float(np.sqrt(np.mean(centered**2))),
        "clipped_fraction": float(np.mean((values <= 2) | (values >= 4093))),
        "dominant_hz": float(frequencies[1 + spectrum[1:].argmax()]),
        "mains_power_fraction": float(spectrum[(frequencies >= 48) & (frequencies <= 52)].sum())
        / max(total, 1e-12),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("rest", "grip"), default="rest")
    parser.add_argument("--seconds", type=float, default=6)
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/emg_calibration_check.json")
    args = parser.parse_args()
    if args.seconds < 4:
        parser.error("--seconds must be at least 4")
    import numpy as np

    from aurora.sensors.ble_input import BleSensorReader

    reader = BleSensorReader()
    raw = []
    captured = []
    envelopes = []
    active = False
    original_sink = reader.decoder.sample_sink

    def sink(sensor, sequence, channels):
        original_sink(sensor, sequence, channels)
        if sensor == "emg" and active:
            raw.extend(channels[0])
            envelopes.append(reader.emg_estimator.envelope)

    reader.decoder.sample_sink = sink
    report = {"phase": args.phase, "error": None}
    try:
        reader.start()
        deadline = time.monotonic() + 15
        while reader.emg_time == 0 and time.monotonic() < deadline:
            time.sleep(0.1)
        if reader.emg_time == 0:
            raise RuntimeError(f"No EMG samples: {reader.status}: {reader.error}")
        print("BLE connected; both IMU and EMG notifications subscribed.", flush=True)
        print("RELAX for baseline now; capture starts in 3 seconds.", flush=True)
        time.sleep(3)
        reader.calibrate_emg()
        time.sleep(1)
        with reader._lock:
            report["baseline_raw_samples"] = list(reader.emg_estimator._calibration)
        if args.phase == "grip":
            if args.interactive:
                input("READY: relaxed baseline captured. Press Enter to start grip countdown.\n")
                time.sleep(2)
            print("GRIP now and hold for the capture.", flush=True)
            reader.calibrate_emg_strength()
        else:
            print("REST capture: keep relaxed.", flush=True)
        active = True
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            data = reader.latest()
            data["emg_status"] = reader.graph_data()["emg_calibration"]
            captured.append(data)
            time.sleep(0.25)
        active = False
        with reader._lock:
            estimator = reader.emg_estimator
            report.update(
                {
                    "rest_rms": estimator.rest_envelope,
                    "reference": estimator.reference,
                    "strong_rms": estimator.strong_envelope,
                    "minimum_grip_rms": estimator.minimum_grip_envelope,
                    "status": estimator.status,
                    "filtered_rms_median": float(np.median(envelopes)) if envelopes else None,
                    "filtered_rms_max": max(envelopes) if envelopes else None,
                    "packet_health": reader.decoder.packet_health(),
                    "signal": describe(raw, reader._emg_period, np),
                    "snapshots": captured,
                    "raw_samples": raw,
                }
            )
    except Exception as error:
        report["error"] = str(error)
    finally:
        reader.stop()
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                key: value
                for key, value in report.items()
                if key not in ("snapshots", "raw_samples", "baseline_raw_samples")
            }
        ),
        flush=True,
    )
    return 1 if report["error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
