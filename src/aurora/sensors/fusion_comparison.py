# Majd
"""Offline planar fusion comparisons with an independent reference."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from aurora.json_utils import write_json
from aurora.paths import REPOSITORY_ROOT
from aurora.recording.emg_labels import digest
from aurora.time_utils import utc_now

METHODS = ("weighted_angle", "complementary", "camera_kalman", "planar_ekf")
SETTINGS = {
    "camera_weight_at_30hz": 0.25,
    "complementary_accel_alpha_at_100hz": 0.98,
    "camera_sigma_deg": 4.0,
    "camera_process_acceleration_deg_s2": 20.0,
    "gyro_process_sigma_deg": 1.0,
    "bias_process_sigma_deg_s": 0.01,
    "gravity_sigma_g": 0.08,
    "gravity_norm_tolerance_g": 0.15,
}


def angle_difference(first, second):
    return (np.asarray(first) - second + 180) % 360 - 180


def correct(state, covariance, residual, observation, noise):
    innovation = observation @ covariance @ observation.T + noise
    gain = np.linalg.solve(innovation, observation @ covariance).T
    updated = state + gain @ np.atleast_1d(residual)
    remainder = np.eye(len(state)) - gain @ observation
    # Joseph form keeps covariance symmetric and positive under repeated updates.
    covariance = remainder @ covariance @ remainder.T + gain @ noise @ gain.T
    return updated, (covariance + covariance.T) / 2


def compare(samples: np.ndarray) -> dict[str, np.ndarray]:
    """time, gyro, gravity sin/cos, camera angle, reference angle; units are s/deg/g."""
    values = np.asarray(samples, dtype=float)
    if values.ndim != 2 or values.shape[1] != 6 or len(values) < 3:
        raise ValueError("Need at least three rows with the six documented columns.")
    if (
        not np.isfinite(values[:, :4]).all()
        or not np.isfinite(values[:, 5]).all()
        or np.isinf(values[:, 4]).any()
    ):
        raise ValueError(
            "Time, gyro, gravity and reference must be finite; blank camera values are allowed."
        )
    intervals = np.diff(values[:, 0])
    if np.any(intervals <= 0) or np.max(intervals) > 0.1:
        raise ValueError("Use increasing aligned timestamps without gaps above 100 ms.")
    initial = (
        values[0, 4]
        if np.isfinite(values[0, 4])
        else math.degrees(math.atan2(values[0, 2], values[0, 3]))
    )
    weighted, complementary = initial, initial
    cv_state, cv_cov = np.array([initial, 0.0]), np.diag([16.0, 400.0])
    ekf_state, ekf_cov = (
        np.array([math.radians(initial), 0.0]),
        np.diag(np.radians([4.0, 2.0]) ** 2),
    )
    camera_time = values[0, 0] - 1 / 30
    outputs = {name: [initial] for name in METHODS}
    for index in range(1, len(values)):
        time, gyro, gravity_sin, gravity_cos, camera, _reference = values[index]
        dt = time - values[index - 1, 0]
        weighted += gyro * dt
        complementary += gyro * dt
        cv_f = np.array([[1.0, dt], [0.0, 1.0]])
        cv_state = cv_f @ cv_state
        cv_noise = SETTINGS["camera_process_acceleration_deg_s2"] ** 2 * np.array(
            [[dt**4 / 4, dt**3 / 2], [dt**3 / 2, dt**2]]
        )
        cv_cov = cv_f @ cv_cov @ cv_f.T + cv_noise
        ekf_f = np.array([[1.0, -dt], [0.0, 1.0]])
        ekf_state[0] += (math.radians(gyro) - ekf_state[1]) * dt
        ekf_cov = (
            ekf_f @ ekf_cov @ ekf_f.T
            + np.diag(
                np.radians(
                    [SETTINGS["gyro_process_sigma_deg"], SETTINGS["bias_process_sigma_deg_s"]]
                )
                ** 2
            )
            * dt
        )
        norm = math.hypot(gravity_sin, gravity_cos)
        if abs(norm - 1) <= SETTINGS["gravity_norm_tolerance_g"]:
            tilt = math.degrees(math.atan2(gravity_sin, gravity_cos))
            accel_weight = 1 - SETTINGS["complementary_accel_alpha_at_100hz"] ** (dt / 0.01)
            complementary += accel_weight * float(angle_difference(tilt, complementary))
            theta = ekf_state[0]
            observation = np.array([[math.cos(theta), 0.0], [-math.sin(theta), 0.0]])
            measured = np.array([gravity_sin, gravity_cos]) / norm
            expected = np.array([math.sin(theta), math.cos(theta)])
            ekf_state, ekf_cov = correct(
                ekf_state,
                ekf_cov,
                measured - expected,
                observation,
                np.eye(2) * SETTINGS["gravity_sigma_g"] ** 2,
            )
        if np.isfinite(camera):
            camera_dt = time - camera_time
            camera_time = time
            weight = 1 - (1 - SETTINGS["camera_weight_at_30hz"]) ** (camera_dt * 30)
            weighted += weight * float(angle_difference(camera, weighted))
            complementary += weight * float(angle_difference(camera, complementary))
            cv_state, cv_cov = correct(
                cv_state,
                cv_cov,
                angle_difference(camera, cv_state[0]),
                np.array([[1.0, 0.0]]),
                np.array([[SETTINGS["camera_sigma_deg"] ** 2]]),
            )
            residual = math.radians(float(angle_difference(camera, math.degrees(ekf_state[0]))))
            ekf_state, ekf_cov = correct(
                ekf_state,
                ekf_cov,
                residual,
                np.array([[1.0, 0.0]]),
                np.array([[math.radians(SETTINGS["camera_sigma_deg"]) ** 2]]),
            )
        for name, estimate in zip(
            METHODS, (weighted, complementary, cv_state[0], math.degrees(ekf_state[0])), strict=True
        ):
            outputs[name].append(estimate)
    return {name: np.asarray(result) for name, result in outputs.items()}


def scores(reference, estimated, times) -> dict:
    error = angle_difference(estimated, reference)
    # This is an offline temporal alignment diagnostic, not measured compute latency.
    step = float(np.median(np.diff(times)))
    candidates = range(
        -min(round(0.25 / step), len(times) // 4), min(round(0.25 / step), len(times) // 4) + 1
    )

    def shift_error(shift):
        if shift > 0:
            return float(np.mean(angle_difference(estimated[shift:], reference[:-shift]) ** 2))
        if shift < 0:
            return float(np.mean(angle_difference(estimated[:shift], reference[-shift:]) ** 2))
        return float(np.mean(error**2))

    lag = min(candidates, key=shift_error) if np.std(reference) > 1 else None
    return {
        "rmse_deg": float(np.sqrt(np.mean(error**2))),
        "mae_deg": float(np.mean(np.abs(error))),
        "p95_abs_error_deg": float(np.percentile(np.abs(error), 95)),
        "best_temporal_shift_ms": lag * step * 1000 if lag is not None else None,
    }


def synthetic_samples(seed=42) -> np.ndarray:
    random = np.random.default_rng(seed)
    time = np.arange(0, 12, 0.01)
    reference = 25 + 35 * np.sin(0.8 * time) + 8 * np.sin(2 * time)
    velocity = 28 * np.cos(0.8 * time) + 16 * np.cos(2 * time)
    gyro = velocity + 1.5 + random.normal(0, 0.7, len(time))
    gravity_sin = np.sin(np.radians(reference)) + random.normal(0, 0.025, len(time))
    gravity_cos = np.cos(np.radians(reference)) + random.normal(0, 0.025, len(time))
    camera = reference + random.normal(0, SETTINGS["camera_sigma_deg"], len(time))
    camera[np.arange(len(time)) % 4 != 0] = np.nan
    camera[(time > 4) & (time < 5)] = np.nan
    return np.column_stack((time, gyro, gravity_sin, gravity_cos, camera, reference))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--file", type=Path, help="Aligned CSV and a matching .json reference description"
    )
    source.add_argument(
        "--synthetic",
        action="store_true",
        help="Known synthetic trajectory; no hardware accuracy claim",
    )
    parser.add_argument("--output", type=Path, help="New diagnostic folder")
    args = parser.parse_args()
    from datetime import datetime

    output = args.output or REPOSITORY_ROOT / "outputs/diagnostics" / datetime.now().strftime(
        "fusion_%Y%m%d_%H%M%S_%f"
    )
    try:
        if args.synthetic:
            values = synthetic_samples()
            metadata = {
                "source": "synthetic",
                "independent_reference": True,
                "reference_source": "analytic_trajectory",
                "clock_domain": "synthetic_seconds",
                "frame": "calibrated_planar_sensor_axis",
            }
        else:
            metadata = json.loads(args.file.with_suffix(".json").read_text(encoding="utf-8"))
            if (
                metadata.get("independent_reference") is not True
                or not metadata.get("reference_source")
                or not metadata.get("clock_domain")
                or metadata.get("frame") != "calibrated_planar_sensor_axis"
            ):
                raise ValueError(
                    "Describe the common clock, calibrated frame and independent reference in the .json file."
                )
            with args.file.open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
            columns = (
                "timestamp_s",
                "gyro_dps",
                "gravity_sin_g",
                "gravity_cos_g",
                "camera_deg",
                "reference_deg",
            )
            values = np.asarray(
                [[float(row[key]) if row[key] else np.nan for key in columns] for row in rows]
            )
            metadata["source_sha256"] = digest(args.file)
        estimated = compare(values)
        report = {
            "format_version": 1,
            "created_utc": utc_now(),
            "task": "offline_planar_fusion_comparison",
            "reference": metadata,
            "settings": SETTINGS,
            "samples": len(values),
            "methods": {
                name: scores(values[:, 5], estimate, values[:, 0])
                for name, estimate in estimated.items()
            },
            "scope": "Planar algorithm comparison; not a full live-controller accuracy measurement.",
        }
        output.mkdir(parents=True, exist_ok=False)
        write_json(output / "comparison.json", report)
        np.savetxt(
            output / "estimates.csv",
            np.column_stack((values[:, 0], values[:, 5], *estimated.values())),
            delimiter=",",
            header=",".join(("timestamp_s", "reference_deg", *estimated)),
            comments="",
        )
        import matplotlib

        matplotlib.use("Agg")
        from matplotlib import pyplot as plt

        figure, axis = plt.subplots(figsize=(10, 5))
        axis.plot(values[:, 0], values[:, 5], color="black", label="Independent reference")
        for name, estimate in estimated.items():
            axis.plot(values[:, 0], estimate, label=name, alpha=0.8)
        axis.set(
            xlabel="Time [s]",
            ylabel="Planar angle [deg]",
            title="Synthetic fusion check" if args.synthetic else "Planar fusion comparison",
        )
        axis.legend()
        figure.tight_layout()
        figure.savefig(output / "comparison.png", dpi=160)
        plt.close(figure)
        print(f"Saved {output}")
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f"Fusion comparison: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
