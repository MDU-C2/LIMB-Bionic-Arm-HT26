"""Standalone diagnostics for AURORA camera 3-D landmarks.

Requires metric camera-frame 3-D keypoints in pose.json, preferably
sample.keypoints_camera_m. Does not modify input files.

Usage:
    python test.py path/to/pose.json
    python test.py path/to/pose.json --side right --save
"""
import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_records(path: Path):
    raw = path.read_text(encoding="utf-8-sig").strip()
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        data = [json.loads(line) for line in raw.splitlines() if line.strip()]
    if isinstance(data, list):
        return data
    if isinstance(data, dict):
        for key in ("data", "frames", "observations", "records", "samples", "pose"):
            if isinstance(data.get(key), list):
                return data[key]
        return [data]
    raise ValueError("Unexpected pose.json structure")


def descend_records(item):
    yield item
    if isinstance(item, dict):
        for key in ("sample", "observation", "pose", "result", "data"):
            nested = item.get(key)
            if isinstance(nested, dict):
                yield from descend_records(nested)


def as_xyz(value):
    if isinstance(value, dict):
        if all(k in value for k in ("x", "y", "z")):
            xyz = [value["x"], value["y"], value["z"]]
        elif all(k in value for k in ("x_m", "y_m", "z_m")):
            xyz = [value["x_m"], value["y_m"], value["z_m"]]
        else:
            return None
    elif isinstance(value, (list, tuple)) and len(value) >= 3:
        xyz = value[:3]
    else:
        return None
    try:
        xyz = np.asarray(xyz, dtype=float)
        return xyz if np.isfinite(xyz).all() else None
    except (ValueError, TypeError):
        return None


def find_metric_points(record):
    for obj in descend_records(record):
        if not isinstance(obj, dict):
            continue
        for key in ("keypoints_camera_m", "camera_points_m", "camera_xyz_m", "keypoints_3d_m"):
            points = obj.get(key)
            if isinstance(points, dict):
                return points
    return None


def get_point(points, side, part):
    for name in (f"{side}_{part}", f"{part}_{side}", part):
        if name in points:
            return as_xyz(points[name])
    return None


def timestamp_ns(record):
    for obj in descend_records(record):
        if not isinstance(obj, dict):
            continue
        for key in ("host_monotonic_ns", "timestamp_ns", "frame_timestamp_ns", "synced_host_monotonic_ns"):
            val = obj.get(key)
            if val is not None:
                try:
                    return int(val)
                except (ValueError, TypeError):
                    pass
    return None


def robust_median(values):
    return np.nanmedian(values, axis=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pose", type=Path, help="Camera pose.json file")
    parser.add_argument("--side", choices=("left", "right"), default=None,
                        help="Arm side (otherwise auto-select most valid samples)")
    parser.add_argument("--threshold-cm", type=float, default=5.0,
                        help="Threshold for frame-to-frame shoulder jumps")
    parser.add_argument("--save", action="store_true", help="Save PNG beside pose.json instead of displaying")
    args = parser.parse_args()

    records = load_records(args.pose)
    candidates = []
    for i, record in enumerate(records):
        if not isinstance(record, dict):
            continue
        points = find_metric_points(record)
        if not points:
            continue
        candidates.append((i, record, points))

    if not candidates:
        raise SystemExit(
            "No metric 3D points found. Expected 'keypoints_camera_m' (or equivalent) "
            "in pose.json. Pixel [x,y,depth_m] is NOT suitable for segment lengths. "
            "Share one anonymized pose.json frame to adapt the parser."
        )

    scores = {}
    for side in ("left", "right"):
        scores[side] = sum(all(get_point(p, side, part) is not None for part in
                               ("shoulder", "elbow", "wrist")) for _, _, p in candidates)
    side = args.side or max(scores, key=scores.get)

    valid_rows = []
    for i, record, points in candidates:
        xyz = [get_point(points, side, part) for part in ("shoulder", "elbow", "wrist")]
        if all(p is not None for p in xyz):
            valid_rows.append((i, timestamp_ns(record), *xyz))

    if len(valid_rows) < 2:
        raise SystemExit(f"Too few complete 3D arm frames on side={side}. Counts: {scores}")

    indices = np.array([row[0] for row in valid_rows])
    timestamps = [row[1] for row in valid_rows]
    if all(t is not None for t in timestamps) and all(b > a for a, b in zip(timestamps, timestamps[1:])):
        t = (np.array(timestamps, dtype=np.float64) - timestamps[0]) * 1e-9
        x_label = "Time from first valid frame [s]"
    else:
        t = indices.astype(float)
        x_label = "Frame index (timestamps missing or invalid)"

    shoulder = np.stack([row[2] for row in valid_rows])
    elbow = np.stack([row[3] for row in valid_rows])
    wrist = np.stack([row[4] for row in valid_rows])
    shoulder_ref = robust_median(shoulder)
    shoulder_delta_cm = (shoulder - shoulder_ref) * 100
    upper_cm = np.linalg.norm(elbow - shoulder, axis=1) * 100
    fore_cm = np.linalg.norm(wrist - elbow, axis=1) * 100
    shoulder_step_cm = np.linalg.norm(np.diff(shoulder, axis=0), axis=1) * 100
    upper_step_cm = np.abs(np.diff(upper_cm))
    fore_step_cm = np.abs(np.diff(fore_cm))
    # Do not confuse nonconsecutive valid frames with adjacent-frame jumps.
    contiguous = (np.diff(indices) == 1)
    jumps = np.flatnonzero((shoulder_step_cm > args.threshold_cm) & contiguous) + 1

    print(f"Input: {args.pose}")
    print(f"Arm: {side} | complete metric frames: {len(valid_rows)} / {len(records)}")
    print(f"Shoulder reference median [m]: {shoulder_ref.round(3).tolist()}")
    print(f"Upper-arm length median [cm]: {np.median(upper_cm):.1f} "
          f"(range {np.min(upper_cm):.1f}-{np.max(upper_cm):.1f})")
    print(f"Forearm length median [cm]: {np.median(fore_cm):.1f} "
          f"(range {np.min(fore_cm):.1f}-{np.max(fore_cm):.1f})")
    print(f"Adjacent shoulder jumps > {args.threshold_cm:g} cm: {len(jumps)}")
    for j in jumps[:15]:
        print(f"  t={t[j]:.3f} ({x_label}): shoulder step {shoulder_step_cm[j-1]:.1f} cm, "
              f"upper length step {upper_step_cm[j-1]:.1f} cm, "
              f"forearm length step {fore_step_cm[j-1]:.1f} cm")
    if len(jumps) > 15:
        print(f"  ... {len(jumps)-15} more")

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for k, name in enumerate(("X", "Y", "Z")):
        axes[0].plot(t, shoulder_delta_cm[:, k], label=f"Shoulder Δ{name}")
    axes[0].axhline(0, color="gray", linewidth=0.8)
    axes[0].set_ylabel("Position vs median [cm]")
    axes[0].set_title("Shoulder position relative to constant median reference")
    axes[0].legend(ncol=3)

    axes[1].plot(t, upper_cm, label="Shoulder–Elbow")
    axes[1].plot(t, fore_cm, label="Elbow–Wrist")
    axes[1].set_ylabel("Estimated segment length [cm]")
    axes[1].set_title("Segment length consistency (should be nearly constant)")
    axes[1].legend()

    # Use NaNs for missing/nonconsecutive interframe steps.
    sh = np.where(contiguous, shoulder_step_cm, np.nan)
    up = np.where(contiguous, upper_step_cm, np.nan)
    axes[2].plot(t[1:], sh, label="Shoulder 3D step")
    axes[2].plot(t[1:], up, label="Upper-arm length step")
    axes[2].axhline(args.threshold_cm, color="gray", linestyle="--", linewidth=1,
                    label=f"Shoulder threshold {args.threshold_cm:g} cm")
    for ax in axes:
        for j in jumps:
            ax.axvline(t[j], alpha=0.15, color="red")
        ax.grid(True, alpha=0.3)
    axes[2].set_ylabel("Step magnitude [cm]")
    axes[2].set_xlabel(x_label)
    axes[2].legend()
    fig.suptitle(f"AURORA camera geometry diagnostics — {side} arm")
    fig.tight_layout()

    if args.save:
        out = args.pose.with_name("camera_geometry_diagnostics.png")
        fig.savefig(out, dpi=160)
        plt.close(fig)
        print(f"Saved: {out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
