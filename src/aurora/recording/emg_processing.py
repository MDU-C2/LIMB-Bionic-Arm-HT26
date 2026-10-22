# Johan Sandred, Majd
"""Filter saved EMG and export six time-domain features."""

from __future__ import annotations

import argparse
import csv
import hashlib
from pathlib import Path

from aurora.json_utils import write_json
from aurora.recording.emg_features import (
    export_features,
    full_window_features,
    load_emg,
    preprocess_emg,
    sample_rate,
)
from aurora.recording.emg_features import window_features as window_features
from aurora.sensors.emg_window import FEATURE_NAMES
from aurora.time_utils import utc_now


def source_metadata(source: Path) -> dict:
    return {
        "format_version": 1,
        "source": source.name,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "processed_utc": utc_now(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, required=True, help="USB recording.csv or BLE emg.csv")
    parser.add_argument("--lowcut", type=float, default=20.0)
    parser.add_argument("--highcut", type=float, default=450.0)
    parser.add_argument("--notch", type=float, default=50.0)
    parser.add_argument("--window-ms", type=float, default=200.0)
    parser.add_argument("--overlap-ms", type=float, default=50.0)
    parser.add_argument(
        "--noise-threshold", type=float, default=5.0, help="ZC/SSC threshold in ADC counts"
    )
    parser.add_argument(
        "--labels",
        type=Path,
        help="Guided emg_trial.json cues; reviewed .labels.json files are for training.",
    )
    args = parser.parse_args()
    try:
        if args.labels is not None:
            summary = export_features(
                args.file,
                args.labels,
                lowcut_hz=args.lowcut,
                highcut_hz=args.highcut,
                notch_hz=args.notch,
                window_ms=args.window_ms,
                overlap_ms=args.overlap_ms,
                noise_threshold_adc=args.noise_threshold,
            )
            write_json(
                args.file.with_suffix(".processing.json"),
                {
                    **source_metadata(args.file),
                    "labels": args.labels.name,
                    "labels_sha256": hashlib.sha256(args.labels.read_bytes()).hexdigest(),
                    **summary,
                },
            )
            print(f"Saved {summary['features']}: {summary['windows_by_phase']}")
            return 0
        times, raw = load_emg(args.file)
        rate_hz = sample_rate(times)
        filtered = preprocess_emg(raw, rate_hz, args.lowcut, args.highcut, args.notch)
        starts, vectors = full_window_features(
            filtered, rate_hz, args.window_ms, args.overlap_ms, args.noise_threshold
        )
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
            writer.writerow(("timestamp_us", *FEATURE_NAMES))
            writer.writerows(
                (time, *vector) for time, vector in zip(times[starts], vectors, strict=True)
            )
        write_json(
            metadata_path,
            {
                **source_metadata(args.file),
                "sample_rate_hz": rate_hz,
                "lowcut_hz": args.lowcut,
                "highcut_hz": args.highcut,
                "notch_hz": args.notch,
                "filter_order": 4,
                "filter_application": "forward_backward",
                "notch_quality": 30,
                "window_ms": args.window_ms,
                "overlap_ms": args.overlap_ms,
                "noise_threshold_adc": args.noise_threshold,
                "samples": len(raw),
                "windows": len(starts),
            },
        )
    except (OSError, ValueError) as error:
        print(f"EMG processing failed: {error}")
        return 1
    print(f"Processed {len(raw)} samples at {rate_hz:g} Hz")
    print(f"Saved {filtered_path}, {features_path} and {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
