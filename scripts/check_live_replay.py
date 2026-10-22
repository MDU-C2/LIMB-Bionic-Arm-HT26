# Majd
"""Check a recorded cup trial through the live controller without hardware."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

from aurora.sensors.fusion import anatomical_reference_angles, interactive_control_targets
from aurora.sensors.replay_input import RecordedSensorInput
from aurora.simulation.interactive.settings import LIVE_ALIGNMENT_POSE_DEG


def check(trial: Path, output: Path) -> dict:
    live = RecordedSensorInput(trial, show_monitor=False, grip_source="emg")
    rows = []
    try:
        live.start()
        live.calibrate(anatomical_reference_angles(LIVE_ALIGNMENT_POSE_DEG))
        stamp = live.elapsed
        while stamp <= live.duration:
            snapshot = live.advance_to(stamp)
            angles = snapshot.angles or {}
            targets = interactive_control_targets(angles) or {}
            if not all(math.isfinite(value) for value in (*angles.values(), *targets.values())):
                raise ValueError("Non-finite angle in replay")
            rows.append(
                {
                    "time_s": round(stamp, 5),
                    "camera_ready": snapshot.camera_ready,
                    "imu_ready": snapshot.imu_ready,
                    "emg_activation": snapshot.emg_activation,
                    **{f"fused_{name}": value for name, value in angles.items()},
                    **{f"target_{name}": value for name, value in targets.items()},
                }
            )
            stamp += 1 / 60
        live.advance_to(live.duration)
        report = {
            "trial": live.directory.name,
            "pace_condition": live.manifest.get("pace_condition", "unspecified"),
            "duration_seconds": live.duration,
            "camera_observations": sum(event[1] == 1 for event in live.events),
            "control_snapshots": len(rows),
            "events_decoded": live.index,
            "camera_ready_snapshots": sum(row["camera_ready"] for row in rows),
            "imu_ready_snapshots": sum(row["imu_ready"] for row in rows),
            "note": "Offline consistency check; camera estimates are not ground truth.",
        }
        output.mkdir(parents=True, exist_ok=True)
        fields = list(dict.fromkeys(key for row in rows for key in row))
        with (output / "control.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        (output / "check.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        return report
    finally:
        live.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path, help="Trial folder or trial.json")
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/replay_check")
    args = parser.parse_args()
    print(json.dumps(check(args.trial, args.out), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
