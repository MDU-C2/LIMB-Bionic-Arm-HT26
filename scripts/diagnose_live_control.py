# Majd
"""Run a bounded live-control check and save the last sensor values and errors."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
if os.name == "nt":
    native_bin = Path(sys.prefix) / "Library" / "bin"
    if native_bin.is_dir():
        os.environ["PATH"] = str(native_bin) + os.pathsep + os.environ.get("PATH", "")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--depth", action="store_true")
    parser.add_argument("--mode", choices=("kinematic", "dynamic"), default="kinematic")
    parser.add_argument("--address", default="")
    parser.add_argument("--device", default="AURORA")
    parser.add_argument("--replay", type=Path, help="Use a recorded cup trial instead of hardware")
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/live_control_diagnostic.json")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    from aurora.simulation.interactive.runtime import InteractiveSimulator
    from aurora.simulation.interactive.settings import parse_args

    simulator = InteractiveSimulator(
        parse_args(
            [
                "--control",
                "camera-imu",
                "--grip-source",
                "emg",
                "--mode",
                args.mode,
                "--address",
                args.address,
                "--device",
                args.device,
                *(["--depth"] if args.depth else []),
                *(["--replay", str(args.replay)] if args.replay else []),
            ]
        )
    )
    report = {"depth": args.depth, "samples": [], "error": None}
    try:
        simulator.initialize_scene()
        simulator.initialize_arm()
        simulator.initialize_dashboard()
        simulator.initialize_hud()
        simulator.initialize_task()
        simulator.start_live_input()
        if args.replay:
            simulator.advance_live_setup()
        deadline = time.monotonic() + args.seconds
        control_started = time.monotonic()
        last_sample = 0.0
        while simulator.running and time.monotonic() < deadline:
            if not simulator.prepare_frame():
                continue
            simulator.process_events()
            for _ in range(simulator.steps_due):
                simulator.update_grasp()
                simulator.update_arm()
                if not simulator.advance_physics():
                    break
            simulator.update_camera()
            simulator.draw_hud()
            simulator.update_dashboard()
            if time.monotonic() - last_sample >= 1 and simulator.live_snapshot is not None:
                last_sample = time.monotonic()
                sample = vars(simulator.live_snapshot).copy()
                report["samples"].append(sample)
                print(json.dumps(sample), flush=True)
            simulator.clock.tick(60)
        report["running_at_end"] = simulator.running
        report["control_frames"] = simulator.frame_number
        report["control_elapsed_s"] = round(time.monotonic() - control_started, 3)
        report["control_frames_per_second"] = round(
            simulator.frame_number / max(0.001, time.monotonic() - control_started), 1
        )
        report["requested_joint_angles_deg"] = {
            "shoulder_x": simulator.arm.shoulder.angle_x,
            "shoulder_y": simulator.arm.shoulder.angle_y,
            "shoulder_z": simulator.arm.shoulder.angle_z,
            "elbow_x": simulator.arm.elbow.angle_x,
            "wrist_rotation": simulator.arm.wrist.rotation,
        }
        report["measured_joint_angles_deg"] = simulator.measured_angles_deg
        live = simulator.live_sensor_input
        if live is not None and live.reader is not None:
            data = live.reader.graph_data()
            report["sensor_health"] = {
                key: data[key]
                for key in (
                    "health",
                    "status",
                    "error",
                    "emg_calibration",
                    "emg_rest",
                    "emg_reference",
                )
            }
            report["emg_adc_range"] = [min(data["emg"]), max(data["emg"])] if data["emg"] else None
    except (Exception, SystemExit) as error:
        report["error"] = str(error)
        report["traceback"] = traceback.format_exc()
        print(report["traceback"], flush=True)
    finally:
        try:
            simulator.close()
        except Exception as error:
            report["cleanup_error"] = str(error)
        finally:
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return 1 if report["error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
