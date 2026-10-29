# Johan Sandred, Majd
"""Filter saved EMG and export six time-domain features."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from aurora.file_utils import file_digest
from aurora.json_utils import write_json
from aurora.recording.common import enable_ctrl_break_as_interrupt
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
        "source_sha256": file_digest(source),
        "processed_utc": utc_now(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, required=True, help="USB recording.csv or BLE emg.csv")
    parser.add_argument(
        "--output", type=Path, help="New derived-data folder; source stays unchanged"
    )
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
    enable_ctrl_break_as_interrupt(stdin_stop=True)
    destination = args.output or args.file.parent
    owned_output = False
    run = {"status": "running", "source": str(args.file.resolve()), "started_utc": utc_now()}
    try:
        if args.output is not None:
            destination.mkdir(parents=True, exist_ok=False)
            owned_output = True
            write_json(destination / "processing.run.json", run)
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
                output_directory=destination,
            )
            write_json(
                destination / (args.file.stem + ".processing.json"),
                {
                    **source_metadata(args.file),
                    "labels": args.labels.name,
                    "labels_sha256": file_digest(args.labels),
                    **summary,
                },
            )
            print(f"Saved {summary['features']}: {summary['windows_by_phase']}")
            run["status"] = "complete"
            return 0
        times, raw = load_emg(args.file)
        rate_hz = sample_rate(times)
        filtered = preprocess_emg(raw, rate_hz, args.lowcut, args.highcut, args.notch)
        starts, vectors = full_window_features(
            filtered, rate_hz, args.window_ms, args.overlap_ms, args.noise_threshold
        )
        filtered_path = destination / (args.file.stem + ".filtered.csv")
        features_path = destination / (args.file.stem + ".features.csv")
        metadata_path = destination / (args.file.stem + ".processing.json")
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
        run["status"] = "complete"
    except KeyboardInterrupt:
        run["status"] = "interrupted"
        print("Processing cancelled; raw data is unchanged.")
        return 130
    except (OSError, ValueError) as error:
        run.update(status="failed", error=str(error))
        print(f"EMG processing failed: {error}")
        return 1
    finally:
        if owned_output:
            run["finished_utc"] = utc_now()
            write_json(destination / "processing.run.json", run)
    print(f"Processed {len(raw)} samples at {rate_hz:g} Hz")
    print(f"Saved {filtered_path}, {features_path} and {metadata_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
