# Christian och Majd
"""Prepare reproducible DMP experiments from examples or recorded cup trials."""

from __future__ import annotations

import argparse
import base64
import csv
import json
import math
import warnings
from pathlib import Path

import numpy as np

from aurora.file_utils import file_digest
from aurora.json_utils import read_json_object, write_json
from aurora.paths import RESOURCE_DIRECTORY
from aurora.simulation.dmp.dmp import fit, rollout_simple
from aurora.simulation.dmp.trajectory_io import load_angles_demo
from aurora.time_utils import utc_now

JOINTS = ("elbow_flexion", "shoulder_flexion", "shoulder_abduction", "shoulder_internal_rotation")
LIVE_CHANNELS = (*JOINTS[:3], "shoulder_rotation_proxy")
DEFAULT_EXAMPLE = RESOURCE_DIRECTORY / "examples/simulation/demo"


def load_demonstration(source: Path, *, start=0.0, end=0.0, sample_hz=50.0):
    """Return a uniform, complete trajectory; never fill unavailable channels with zero."""
    source = Path(source).resolve()
    directory = source.parent if source.is_file() else source
    if not all(math.isfinite(x) for x in (start, end, sample_hz)) or start < 0 or end < 0:
        raise ValueError("Start/end must be finite nonnegative times")
    if not 5 <= sample_hz <= 200:
        raise ValueError("Sample rate must be between 5 and 200 Hz")
    if (directory / "trial.json").is_file():
        return _load_cup(directory, start, end, sample_hz)
    # Prepared examples retain explicit timestamps when supplied. Legacy data
    # has no timing contract, so the historical one-second assumption is visible.
    with warnings.catch_warnings(record=True) as input_warnings:
        warnings.simplefilter("always")
        q = load_angles_demo(directory, source="clean")
    timing = directory / "timing.json"
    metadata = read_json_object(timing) if timing.exists() else {}
    duration = float(metadata.get("duration_seconds", 1.0))
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Example duration must be finite and positive")
    time = np.linspace(0, duration, len(q))
    stop = end or duration
    if not start < stop <= duration:
        raise ValueError(
            "Choose 0 <= start < end <= recording duration (end=0 means full duration)"
        )
    count = int(round((stop - start) * sample_hz)) + 1
    if count < 26:
        raise ValueError(
            "Choose a longer interval: at least 26 samples are required; increase sample rate or interval length"
        )
    selected = np.linspace(start, stop, count)
    values = np.column_stack([np.interp(selected, time, col) for col in q.T])
    paths = sorted(directory.glob("*.npz")) + ([timing] if timing.exists() else [])
    return (
        selected,
        values,
        {
            "source_kind": "prepared_angles",
            "synthetic": metadata.get("synthetic", False),
            "coordinate_frame": "actuator_angles",
            "timing_assumption": None
            if timing.exists()
            else "Legacy angles normalized to one second",
            "source_sha256": {p.name: file_digest(p) for p in paths},
            "warnings": [str(item.message) for item in input_warnings]
            + (
                []
                if timing.exists()
                else ["Legacy example has no measured duration; one second assumed."]
            ),
        },
    )


def _load_cup(directory, start, end, sample_hz):
    from aurora.sensors.replay_input import RecordedSensorInput

    manifest = read_json_object(directory / "trial.json")
    if manifest.get("complete") is not True:
        raise ValueError("Choose a completed cup trial; review or repeat incomplete captures first")
    if manifest.get("test_type") == "subject_reference":
        raise ValueError("Choose a movement trial, not a calibration reference")
    duration = float(manifest["duration_seconds"])
    stop = end or duration
    if not math.isfinite(duration) or not start < stop <= duration:
        raise ValueError(
            "Choose 0 <= start < end <= recording duration (end=0 means full duration)"
        )
    paths = [directory / name for name in ("trial.json", "ble/packets.jsonl", "camera/pose.json")]
    hashes = {p.relative_to(directory).as_posix(): file_digest(p) for p in paths}
    # Replay from the beginning to retain integration history before the crop.
    replay = RecordedSensorInput(directory, show_monitor=False, grip_source="emg")
    try:
        replay.start()
        if start < replay.elapsed - 1e-9:
            raise ValueError(
                f"First camera reference is at {replay.elapsed:.3f}s; choose a later start"
            )
        replay.calibrate()
        count = int(round((stop - start) * sample_hz)) + 1
        if count < 26:
            raise ValueError(
                "Choose a longer interval: at least 26 samples are required; increase sample rate or interval length"
            )
        time = np.linspace(start, stop, count)
        rows = []
        body_references = set()
        for stamp in time:
            state = replay.advance_to(float(stamp))
            angles = state.angles or {}
            missing = [name for name in LIVE_CHANNELS if name not in angles]
            if (
                not state.calibration_valid
                or missing
                or not (state.imu_ready or state.camera_ready)
            ):
                raise ValueError(
                    f"Unavailable/reconnected or unreferenced angles at {stamp:.3f}s "
                    f"({', '.join(missing) or state.status}). Choose a continuous valid interval."
                )
            values = [angles[name] for name in LIVE_CHANNELS]
            if not np.isfinite(values).all():
                raise ValueError(f"Non-finite angles at {stamp:.3f}s")
            rows.append(values)
            body_references.add(state.body_reference)
    finally:
        replay.close()
    if hashes != {p.relative_to(directory).as_posix(): file_digest(p) for p in paths}:
        raise ValueError("Recording changed during import; finish recording before fitting")
    return (
        time,
        np.deg2rad(rows),
        {
            "source_kind": "cup_trial",
            "synthetic": manifest.get("synthetic", False),
            "coordinate_frame": "human_estimates_with_rotation_proxy",
            "source_sha256": hashes,
            "subject_id": manifest.get("subject_id"),
            "side": manifest["side"],
            "pace_condition": manifest.get("pace_condition"),
            "body_references": sorted(body_references),
            "source_channels": list(LIVE_CHANNELS),
            "warnings": [
                "Fourth channel is a shoulder rotation proxy, not validated anatomical axial rotation.",
                "Reproduction error compares estimated angles; it is not physical tracking accuracy.",
            ]
            + (
                ["Shoulder estimate assumes a stationary trunk during part of this interval."]
                if "stationary_trunk_assumed" in body_references
                else []
            ),
        },
    )


def actuator_angles(q, frame):
    """Map unclipped human estimates to the existing left CAD playback convention."""
    result = np.array(q, copy=True)
    if frame == "human_estimates_with_rotation_proxy":
        result[:, 1] = np.pi / 2 - result[:, 1]
        result[:, 3] *= -1
    return result


def run_experiment(
    source, output, *, start=0.0, end=0.0, sample_hz=50.0, duration_scale=1.0, basis=15
):
    if not math.isfinite(duration_scale) or not 0.25 <= duration_scale <= 4:
        raise ValueError("Duration multiplier must be between 0.25 and 4")
    if not 3 <= basis <= 100:
        raise ValueError("Choose between 3 and 100 basis functions")
    source, output = Path(source).resolve(), Path(output).resolve()
    directory = source.parent if source.is_file() else source
    if output == directory or output.is_relative_to(directory):
        raise ValueError("Choose an output folder outside the source recording")
    print("Reading demonstration and checking selected interval...", flush=True)
    time, original, metadata = load_demonstration(source, start=start, end=end, sample_hz=sample_hz)
    tau = float(time[-1] - time[0])
    dt = tau / (len(time) - 1)
    model = fit([original], tau, dt, basis, 4.0, 25.0, 6.25)
    generated = rollout_simple(
        model, original[0], original[-1], tau * duration_scale, dt * duration_scale
    )
    error = np.rad2deg(generated - original)
    source_joints = LIVE_CHANNELS if metadata["source_kind"] == "cup_trial" else JOINTS
    metrics = {
        name: {
            "rmse_deg": float(np.sqrt(np.mean(error[:, i] ** 2))),
            "max_abs_error_deg": float(np.max(np.abs(error[:, i]))),
            "endpoint_error_deg": float(error[-1, i]),
        }
        for i, name in enumerate(source_joints)
    }
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "format_version": 1,
        "status": "running",
        "created_utc": utc_now(),
        "source": str(source),
        **metadata,
        "joint_order": list(source_joints),
        "playback_joint_order": list(JOINTS),
        "units": "rad",
        "samples": len(time),
        "crop_start_s": float(time[0]),
        "crop_end_s": float(time[-1]),
        "original_duration_s": tau,
        "duration_scale": duration_scale,
        "generated_duration_s": tau * duration_scale,
        "sample_interval_s": dt,
        "basis_functions": basis,
        "alpha_canonical": 4.0,
        "alpha_transformation": 25.0,
        "beta_transformation": 6.25,
        "comparison": "Phase-aligned reconstruction error, before robot limits",
        "metrics": metrics,
        "software_versions": {"numpy": np.__version__},
        "source_code_sha256": {
            p.name: file_digest(p)
            for p in (
                Path(__file__),
                Path(__file__).with_name("dmp.py"),
                Path(__file__).with_name("dmp_signal.py"),
            )
        },
        "playback_mapping": "left CAD actuator convention; shoulder flexion=90-human, rotation=-proxy"
        if metadata["source_kind"] == "cup_trial"
        else "Prepared actuator angles; unchanged",
        "playback_note": "Position/speed limits apply in the simulator; exported angles are unclipped.",
    }
    write_json(output / "experiment.json", report)
    try:
        relative = time - time[0]
        np.savez(
            output / "comparison.npz",
            time_s=relative,
            source_time_s=time,
            q_demo_rad=original,
            q_fit_rad=generated,
        )
        robot_original = actuator_angles(original, metadata["coordinate_frame"])
        robot_generated = actuator_angles(generated, metadata["coordinate_frame"])
        np.savez(
            output / "angles.npz",
            elbow_rad=robot_original[:, 0],
            shoulder_rad=robot_original[:, 1:],
        )
        write_json(
            output / "timing.json", {"duration_seconds": tau, "synthetic": metadata["synthetic"]}
        )
        np.savez(
            output / "dmp_rollout_clean.npz", q_gen_rad=robot_generated, dt=dt * duration_scale
        )
        np.savez(
            output / "model.npz",
            weights=model.weights,
            centers=model.centers,
            widths=model.widths,
            q0=original[0],
            goal=original[-1],
        )
        with (output / "comparison.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["source_time_s", "original_time_s", "dmp_time_s"]
                + [f"{name}_{kind}_rad" for name in source_joints for kind in ("original", "dmp")]
            )
            for i, stamp in enumerate(relative):
                writer.writerow(
                    [time[i], stamp, stamp * duration_scale]
                    + [
                        value
                        for pair in zip(original[i], generated[i], strict=True)
                        for value in pair
                    ]
                )
        from aurora.simulation.dmp.comparison import save_comparison

        save_comparison(output)
        report["status"] = "complete"
        report["artifacts_sha256"] = {
            p.name: file_digest(p)
            for p in output.iterdir()
            if p.is_file() and p.name != "experiment.json"
        }
    except BaseException as error:
        report.update(
            status="interrupted"
            if isinstance(error, (KeyboardInterrupt, InterruptedError))
            else "failed",
            error=str(error),
        )
        raise
    finally:
        write_json(output / "experiment.json", report)
    print(f"Saved DMP experiment: {output}", flush=True)
    print(
        "[AURORA] " + json.dumps({"event": "saved", "path": str(output), "kind": "dmp"}), flush=True
    )
    return output


def make_synthetic_trial(output):
    """Create explicitly synthetic packets/observations in the real replay format."""
    from aurora.sensors.ble_protocol import HIGH_RATE_IMU_PACKET

    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "ble").mkdir()
    (output / "camera").mkdir()
    origin, duration = 10_000_000_000, 4.0
    profile = {
        "imu_tilt_offsets": {"shoulder": 0, "wrist": 0},
        "imu_gyro_bias_z": {"shoulder": 0, "wrist": 0},
        "imu_twist_bias_dps": {role: [0, 0, 0] for role in ("shoulder", "wrist")},
    }
    write_json(
        output / "trial.json",
        {
            "synthetic": True,
            "source": "generated DMP import exercise; not participant data",
            "complete": True,
            "side": "left",
            "subject_id": "SYNTHETIC_DEMO",
            "trial_id": "T001",
            "duration_seconds": duration,
            "started_host_monotonic_ns": origin,
            "calibration": profile,
        },
    )
    observations = []
    with (output / "ble/packets.jsonl").open("w", encoding="utf-8") as stream:
        for index in range(401):
            stamp = 1_000_000 + index * 10_000
            packet = HIGH_RATE_IMU_PACKET.pack(
                0x103, index, stamp, stamp, 0, 0, 1000, 0, 0, 0, 0, 0, 1000, 0, 0, 0
            )
            stream.write(
                json.dumps(
                    {
                        "host_monotonic_ns": origin + index * 10_000_000,
                        "sensor": "imu",
                        "connection_generation": 0,
                        "data_base64": base64.b64encode(packet).decode(),
                    }
                )
                + "\n"
            )
            if index % 4 == 0:
                bump = math.sin(math.pi * index / 400) ** 2
                observations.append(
                    {
                        "host_monotonic_ns": origin + index * 10_000_000,
                        "angles_deg": dict(
                            zip(
                                LIVE_CHANNELS,
                                (30 + 15 * bump, 70 - 15 * bump, 10 + 10 * bump, 5 * bump),
                                strict=True,
                            )
                        ),
                    }
                )
    write_json(output / "camera/pose.json", {"observations": observations})
    print(f"Saved SYNTHETIC import exercise (not participant data): {output}", flush=True)
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("fit", "synthetic"))
    parser.add_argument("--source", type=Path, default=DEFAULT_EXAMPLE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start", type=float, default=0)
    parser.add_argument("--end", type=float, default=0)
    parser.add_argument("--sample-hz", type=float, default=50)
    parser.add_argument("--duration-scale", type=float, default=1)
    parser.add_argument("--basis", type=int, default=15)
    args = parser.parse_args(argv)
    from aurora.recording.common import enable_ctrl_break_as_interrupt

    enable_ctrl_break_as_interrupt(stdin_stop=True)
    try:
        if args.action == "synthetic":
            make_synthetic_trial(args.output)
        else:
            run_experiment(
                args.source,
                args.output,
                start=args.start,
                end=args.end,
                sample_hz=args.sample_hz,
                duration_scale=args.duration_scale,
                basis=args.basis,
            )
        return 0
    except (KeyboardInterrupt, InterruptedError):
        print("DMP action cancelled", flush=True)
        return 130
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"DMP action failed: {error}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
