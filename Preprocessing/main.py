"""AURORA offline preprocessing integration and preview.

Requires the agreed sensor_state.py, filters.py, vision.py alongside this file.
Does not implement final BLE/camera synchronization, sensor-ID mapping or
validated per-subject calibration. No camera tracking is recomputed from video.
"""
from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict
from pathlib import Path

from filters import EMGConfig, EMGProcessor, IMUConfig, IMUProcessor
from reconstruct_timestamps import get_trial_start, relative_seconds
from vision import VisionProcessor

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RECORDINGS_DIR = PROJECT_ROOT / "outputs" / "recordings"
PROCESSED_DIR = PROJECT_ROOT / "outputs" / "processed"

# Offline PREVIEW ONLY: not validated resting/immobile calibration intervals.
PREVIEW_CALIBRATION_S = 0.5

IMU_SENSOR_LABELS = {
    0: "Forearm/Wrist (IMU0)",
    1: "Upper Arm/Shoulder (IMU1)",
}
IMU_FOREARM_SENSOR_ID = 0
IMU_UPPER_ARM_SENSOR_ID = 1

def load_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def find_trials() -> list[Path]:
    if not RECORDINGS_DIR.exists():
        return []
    return sorted((p.parent.parent for p in RECORDINGS_DIR.rglob("ble/emg.csv")
                   if (p.parent / "imu.csv").is_file()), key=str)


def _select_trial() -> Path | None:
    trials = find_trials()
    if not trials:
        print(f"No recordings found under {RECORDINGS_DIR}")
        return None
    subjects = sorted({p.parent.name for p in trials})
    print("\nSubjects:")
    for i, subject in enumerate(subjects, 1):
        print(f"{i}. {subject}")
    choice = input("Subject number (Enter to cancel): ").strip()
    if not choice:
        return None
    if not choice.isdigit() or not 1 <= int(choice) <= len(subjects):
        print("Invalid selection")
        return None
    subject_trials = [p for p in trials if p.parent.name == subjects[int(choice)-1]]
    print("\nTrials:")
    for i, trial in enumerate(subject_trials, 1):
        print(f"{i}. {trial.name}")
    choice = input("Trial number (Enter to cancel): ").strip()
    if not choice:
        return None
    if not choice.isdigit() or not 1 <= int(choice) <= len(subject_trials):
        print("Invalid selection")
        return None
    return subject_trials[int(choice)-1]


def _first_window(rows: list[dict], seconds: float) -> list[dict]:
    if not rows:
        return []
    t0 = int(rows[0]["synced_host_monotonic_ns"])
    end = t0 + int(seconds * 1e9)
    return [r for r in rows if int(r["synced_host_monotonic_ns"]) < end]


def process_emg(rows: list[dict]):
    """Use a provisional first-0.5s baseline only for preview; not for scientific labels."""
    processor = EMGProcessor(EMGConfig())
    if not rows:
        return []
    calibration = _first_window(rows, PREVIEW_CALIBRATION_S)
    if calibration:
        processor.calibrate([float(r["value"]) for r in calibration])
    states = []
    for row in rows:
        states.append(processor.update(
            timestamp_ns=int(row["synced_host_monotonic_ns"]),
            raw_adc=float(row["value"]),
        ))
    return states


def process_imus(rows: list[dict]):
    """Process the two sensor IDs separately (0=forearm, 1=upper arm)."""
    by_sensor: dict[int, list[dict]] = {}
    for row in rows:
        by_sensor.setdefault(int(row["sensor"]), []).append(row)
    results = {}
    for sensor_id, sensor_rows in sorted(by_sensor.items()):
        processor = IMUProcessor(IMUConfig())
        calibration = _first_window(sensor_rows, PREVIEW_CALIBRATION_S)
        if calibration:
            processor.calibrate([
                (float(r["gyro_x"]), float(r["gyro_y"]), float(r["gyro_z"]))
                for r in calibration
            ])
        results[sensor_id] = [
            processor.update(
                timestamp_ns=int(r["synced_host_monotonic_ns"]),
                accel=(float(r["accel_x"]), float(r["accel_y"]), float(r["accel_z"])),
                gyro=(float(r["gyro_x"]), float(r["gyro_y"]), float(r["gyro_z"])),
                gyro_saturated=r.get("gyro_saturated", "0") == "1",
            ) for r in sensor_rows
        ]
    return by_sensor, results


def process_camera(path: Path):
    if not path.exists():
        return None, []
    recording = json.loads(path.read_text(encoding="utf-8"))
    processor = VisionProcessor()
    # No fake open/closed grip reference: grip_closure stays None.
    return recording, processor.process_recording(recording)


def process_trial(trial: Path, save: bool = False):
    ble = trial / "ble"
    print(f"\nProcessing {trial.parent.name}/{trial.name}")
    trial_start_ns = get_trial_start(trial)
    emg_rows = load_csv(ble / "emg.csv")
    imu_rows = load_csv(ble / "imu.csv")
    for row in emg_rows + imu_rows:
        row["aligned_time_s"] = relative_seconds(int(row["synced_host_monotonic_ns"]), trial_start_ns)
    emg_states = process_emg(emg_rows)
    imu_raw, imu_states = process_imus(imu_rows)
    cam_recording, cam_states = process_camera(trial / "camera" / "pose.json")

    results = {
        "trial": trial, "trial_start_ns": trial_start_ns, "emg_raw": emg_rows, "emg": emg_states,
        "imu_raw": imu_raw, "imu": imu_states,
        "camera_raw": cam_recording, "camera": cam_states,
    }
    print(f"EMG: {len(emg_states)} samples")
    for sid, states in imu_states.items():
        print(f"IMU sensor={sid}: {len(states)} samples (0=forearm, 1=upper arm)")
    print(f"Vision: {len(cam_states)} observations")
    if cam_states:
        print("Vision arm valid:", sum(x.arm.valid for x in cam_states),
              "| hand tracked:", sum(x.hand.tracking_valid for x in cam_states))
    print("PREVIEW CALIBRATION: first 0.5s, unverified; no valid grip reference")
    if save:
        output_dir = PROCESSED_DIR / trial.relative_to(RECORDINGS_DIR)
        save_results(results, output_dir)
        print(f"Saved: {output_dir}")
    return results


def _json_value(value):
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")) if value is not None else ""


def save_states_csv(path: Path, states: list):
    if not states:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = [asdict(s) for s in states]
    columns = list(rows[0])
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: _json_value(v) if isinstance(v, (dict, list, tuple))
                             else ("" if v is None else v) for k, v in row.items()})


def save_results(results: dict, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    save_states_csv(out_dir / "emg_processed.csv", results["emg"])
    for sid, states in results["imu"].items():
        save_states_csv(out_dir / f"imu_sensor_{sid}_processed.csv", states)
    save_states_csv(out_dir / "camera_processed.csv", results["camera"])
    with (out_dir / "preprocessing_events.jsonl").open("w", encoding="utf-8") as stream:
        events = [(s.timestamp_ns, "emg", s) for s in results["emg"]]
        for sid, states in results["imu"].items():
            events += [(s.timestamp_ns, f"imu_sensor_{sid}", s) for s in states]
        events += [(s.timestamp_ns, "camera", s) for s in results["camera"]]
        for t, source, state in sorted(events, key=lambda x: x[0]):
            stream.write(json.dumps({"timestamp_ns": t, "source": source,
                                     "state": asdict(state)}, allow_nan=False) + "\n")
    summary = {
        "subject": results["trial"].parent.name,
        "trial": results["trial"].name,
        "emg_samples": len(results["emg"]),
        "imu_samples_by_sensor_id": {str(k): len(v) for k, v in results["imu"].items()},
        "camera_observations": len(results["camera"]),
        "camera_valid_arm": sum(s.arm.valid for s in results["camera"]),
        "camera_valid_hand": sum(s.hand.tracking_valid for s in results["camera"]),
        "calibration_note": "First 0.5s preview baseline/gyro bias, not verified stationary/resting",
        "synchronization_note": "Common host time origin; physical sensor alignment and timing accuracy not validated",
        "imu_mapping_note": "Sensor 0=forearm, sensor 1=upper arm; orientation frame alignment still uncalibrated",
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if plt is not None:
        plot_emg_imu(results, show=False, save_to=out_dir / "emg_imu_preview.png")
        plot_camera(results, show=False, save_to=out_dir / "camera_preview.png")
        plot_imu_orientation(results, show=False, save_to=out_dir / "imu_orientation_preview.png")


def _sec(times: list[int], origin: int) -> list[float]:
    return [relative_seconds(t, origin) for t in times]


def plot_emg_imu(results: dict, show: bool = True, save_to: Path | None = None):
    if plt is None:
        print("Install matplotlib to plot: pip install matplotlib")
        return
    raw = results["emg_raw"]
    imu_raw = results["imu_raw"]
    ids = sorted(imu_raw)
    if not raw and not ids:
        return
    origin = results["trial_start_ns"]
    # Keep old raw vs processed preview, for BOTH IMUs.
    nrows = 2 + 4 * len(ids)
    fig, axes = plt.subplots(nrows, 1, figsize=(14, max(6, 2.0 * nrows)), sharex=True, squeeze=False)
    axes = [a[0] for a in axes]
    if raw:
        t = _sec([int(r["synced_host_monotonic_ns"]) for r in raw], origin)
        axes[0].plot(t, [float(r["value"]) for r in raw], linewidth=0.6)
        axes[0].set_title("EMG raw ADC")
        valid = [(s.timestamp_ns, s.activity) for s in results["emg"] if s.activity is not None]
        if valid:
            axes[1].plot(_sec([x[0] for x in valid], origin), [x[1] for x in valid])
        axes[1].set_title("EMG processed activity (provisional baseline)")
    axisnames = 'xyz'
    for i, sid in enumerate(ids):
        rows = imu_raw[sid]
        states = results["imu"][sid]
        t = _sec([int(r["synced_host_monotonic_ns"]) for r in rows], origin)
        tf = _sec([s.timestamp_ns for s in states], origin)
        base = 2 + i * 4
        for j, axis in enumerate(axisnames):
            axes[base].plot(t, [float(r[f"accel_{axis}"]) for r in rows], label=axis)
            axes[base+1].plot(tf, [s.accel[j] if s.accel is not None else math.nan for s in states], label=axis)
            axes[base+2].plot(t, [float(r[f"gyro_{axis}"]) for r in rows], label=axis)
            axes[base+3].plot(tf, [s.gyro[j] if s.gyro is not None else math.nan for s in states], label=axis)
            label = IMU_SENSOR_LABELS.get(sid, f"IMU sensor={sid}")
            for j, suffix in enumerate((
                "Accel raw [g]",
                "Accel filtered [g]",
                "Gyro raw [deg/s]",
                "Gyro filtered [deg/s]",
            )):
                axes[base+j].set_title(f"{label} — {suffix}")
                axes[base+j].legend(loc="upper right", ncol=3)
    for ax in axes:
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Time from trial start [s]")
    fig.tight_layout()
    if save_to:
        fig.savefig(save_to, dpi=140)
    if show:
        plt.show()
    else:
        plt.close(fig)


def match_ylim(ax_raw, ax_filtered):
    raw_min, raw_max = ax_raw.get_ylim()
    filtered_min, filtered_max = ax_filtered.get_ylim()

    y_min = min(raw_min, filtered_min)
    y_max = max(raw_max, filtered_max)

    ax_raw.set_ylim(y_min, y_max)
    ax_filtered.set_ylim(y_min, y_max)

def plot_camera(results: dict, show: bool = True, save_to: Path | None = None):
    if plt is None:
        print("Install matplotlib to plot: pip install matplotlib")
        return

    source = results["camera_raw"]
    states = results["camera"]

    if not source or not states:
        print("No camera data to preview")
        return

    # Match raw pose and observations to processed timestamps
    poses_by_time = {
        int(p["host_monotonic_ns"]): p
        for p in source.get("data", [])
    }

    observations_by_time = {
        int(obs["host_monotonic_ns"]): obs
        for obs in source.get("observations", [])
        if obs.get("host_monotonic_ns") is not None
    }

    poses = [
        poses_by_time.get(state.timestamp_ns, {})
        for state in states
    ]

    observations = [
        observations_by_time.get(state.timestamp_ns, {})
        for state in states
    ]

    side = source.get("side", "left")
    origin = results["trial_start_ns"]
    t = _sec([s.timestamp_ns for s in states], origin)

    fig, axes = plt.subplots(
        9, 1,
        figsize=(14, 19),
        sharex=True,
    )

    # ========================================================
    # ARM POSITION - RAW VS FILTERED
    # ========================================================

    for k, joint in enumerate(("shoulder", "elbow", "wrist")):

        vals_raw = [
            p.get("keypoints_camera_m", {}).get(f"{side}_{joint}")
            for p in poses
        ]

        vals_filtered = [
            getattr(state.arm, joint)
            for state in states
        ]

        reference = next(
            (point for point in vals_raw if point is not None),
            None
        )

        if reference is None:
            continue

        for a, axis_name in enumerate(("X", "Y", "Z")):

            raw_values = [
                (point[a] - reference[a]) * 100
                if point is not None else math.nan
                for point in vals_raw
            ]

            filtered_values = [
                (point[a] - reference[a]) * 100
                if point is not None else math.nan
                for point in vals_filtered
            ]

            axes[2*k].plot(
                t, raw_values, label=axis_name
            )

            axes[2*k+1].plot(
                t, filtered_values, label=axis_name
            )

        axes[2*k].set_title(
            f"{joint.title()} XYZ - Raw (relative)"
        )

        axes[2*k+1].set_title(
            f"{joint.title()} XYZ - Filtered (relative)"
        )

        axes[2*k].set_ylabel("cm")
        axes[2*k+1].set_ylabel("cm")

        axes[2*k].legend()
        axes[2*k+1].legend()

        match_ylim(axes[2*k], axes[2*k+1])

    # ========================================================
    # FINGER CURL - RAW VS FILTERED
    # ========================================================

    fingers = ("index", "middle", "ring", "pinky")

    for finger in fingers:

        raw_values = [
            (
                obs.get("finger_curl_deg", {}).get(finger, math.nan)
                if obs.get("hand_visible", False)
                else math.nan
            )
            for obs in observations
        ]

        filtered_values = [
            state.hand.finger_curl.get(finger, math.nan)
            if state.hand.tracking_valid else math.nan
            for state in states
        ]

        # Same finger color in both plots
        line, = axes[6].plot(
            t, raw_values, label=finger
        )

        axes[7].plot(
            t,
            filtered_values,
            label=finger,
            color=line.get_color(),
        )

    axes[6].set_title("Finger Curl - Raw")
    axes[7].set_title("Finger Curl - Filtered (EMA)")

    for ax in (axes[6], axes[7]):
        ax.set_ylabel("deg")
        ax.legend(loc="upper right", ncol=4)

    match_ylim(axes[6], axes[7])

    # ========================================================
    # CAMERA VALIDITY FLAGS
    # ========================================================

    axes[8].step(
        t,
        [int(s.hand.tracking_valid) for s in states],
        where="post",
        label="hand tracked",
    )

    axes[8].step(
        t,
        [int(s.arm.valid) for s in states],
        where="post",
        label="arm valid",
    )

    axes[8].set_title("Camera Validity Flags")
    axes[8].set_ylim(-0.1, 1.1)
    axes[8].legend(loc="upper right")

    # ========================================================
    # PLOT SETTINGS
    # ========================================================

    for ax in axes:
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel(
        "Time from trial start [s]"
    )

    fig.tight_layout()

    if save_to:
        fig.savefig(save_to, dpi=140)

    if show:
        plt.show()
    else:
        plt.close(fig)

def quat_to_euler(q: tuple[float, float, float, float]):
    """Quaternion (w,x,y,z) to roll/pitch/yaw in degrees, for preview only."""
    w, x, y, z = q
    roll = math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
    sinp = 2.0*(w*y - z*x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def plot_imu_orientation(results: dict, show: bool = True, save_to: Path | None = None):
    """Preview RPY for each IMU; the saved estimate remains a quaternion."""
    if plt is None:
        print("Install matplotlib to plot: pip install matplotlib")
        return
    imu_states = results["imu"]
    if not imu_states:
        print("No IMU data to preview")
        return

    ids = sorted(imu_states)
    fig, axes = plt.subplots(len(ids), 1, figsize=(14, 4*len(ids)),
                             sharex=True, squeeze=False)
    for ax, sid in zip((row[0] for row in axes), ids):
        states = imu_states[sid]
        t = _sec([s.timestamp_ns for s in states], results["trial_start_ns"])
        angles = [quat_to_euler(s.orientation_quat)
                  if s.orientation_valid and s.orientation_quat is not None
                  else (math.nan, math.nan, math.nan) for s in states]
        for index, name in enumerate(("Roll (X)", "Pitch (Y)", "Yaw (Z)")):
            ax.plot(t, [r[index] for r in angles], label=name)
        ax.set_title(f"{IMU_SENSOR_LABELS.get(sid, f'IMU sensor={sid}')} - Orientation")
        ax.set_ylabel("Angle [deg]")
        ax.legend(loc="upper right", ncol=3)
        ax.grid(True, alpha=0.3)
        print(f"IMU{sid} orientation valid: {sum(s.orientation_valid for s in states)}/{len(states)}")
    axes[-1][0].set_xlabel("Time from trial start [s]")
    fig.suptitle("IMU Orientation Estimation")
    fig.tight_layout()
    if save_to:
        fig.savefig(save_to, dpi=140)
    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> int:
    while True:
        print("\n======================================")
        print(" AURORA PREPROCESSING - OFFLINE TEST")
        print("======================================")
        print("1. Preview EMG + both IMUs")
        print("2. Preview camera + finger tracking")
        print("3. Preview IMU Orientation")
        print("4. Process one recording + save CSV/JSONL/graphs")
        print("5. Process all recordings")
        print("6. Exit")
        choice = input("Select mode: ").strip()
        if choice == "6":
            return 0
        if choice in ("1", "2", "3", "4"):
            trial = _select_trial()
            if trial is None:
                continue
            try:
                results = process_trial(trial, save=(choice == "4"))
                if choice == "1":
                    plot_emg_imu(results)
                elif choice == "2":
                    plot_camera(results)
                elif choice == "3":
                    plot_imu_orientation(results)
            except (OSError, ValueError, KeyError, TypeError) as exc:
                print(f"Cannot process trial: {exc}")
        elif choice == "5":
            trials = find_trials()
            print(f"Found {len(trials)} trials")
            for trial in trials:
                try:
                    process_trial(trial, save=True)
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    print(f"Skipped {trial}: {exc}")
        else:
            print("Invalid selection")


if __name__ == "__main__":
    raise SystemExit(main())
