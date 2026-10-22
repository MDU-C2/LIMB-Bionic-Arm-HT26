# Majd
"""Keep camera and BLE open while recording a subject's cup transfers."""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/simulation/ai"))

from common import DEFAULT_OUTPUT, enable_ctrl_break_as_interrupt
from cup_trials import CupTrialSession, next_trial_number
from imu_twist import DEFAULT_TWIST_AXIS, TWIST_AXES
from live_sensor_input import LiveSensorInput

WINDOW = "LIMB | Cup recording"


def emit(event: str, **values) -> None:
    print("[CUP] " + json.dumps({"event": event, **values}), flush=True)


def read_commands(commands: queue.Queue) -> None:
    for line in sys.stdin:
        try:
            value = json.loads(line)
            if isinstance(value, dict):
                commands.put(value)
        except ValueError:
            emit("error", message="Could not read the recording command")
    commands.put("connection_closed")


class CupRecorder:
    def __init__(self, args, live):
        self.args = args
        self.live = live
        self.trials = CupTrialSession(args.output, args.subject, args.side, live.reader, live.cv2)
        self.rest_requested = False
        self.imu_requested = False
        self.running = True
        self.graph_root = None
        self.graphs = None
        self.last_frame = -1
        self.connection_generation = live.reader.connection_generation

    def state(self) -> dict:
        data = self.live.reader.graph_data()
        packet = self.live.reader.latest()
        camera = bool(
            self.live.camera_observation.get("arm_visible")
            and time.monotonic_ns() - self.live.frame_time_ns <= 1_000_000_000
        )
        imu = len(packet["imus"]) == 2
        emg = bool(data["emg"])
        calibrated = (
            self.rest_requested
            and self.imu_requested
            and packet["imu_angles"] is not None
            and packet["imu_calibrated"]
            and data["emg_calibration"] == "Grip calibrated"
        )
        stage = (
            "connect"
            if not (imu and emg)
            else "imu"
            if not self.imu_requested or not packet["imu_calibrated"]
            else "rest"
            if not self.rest_requested or not self.live.reader.emg_estimator.calibrated
            else "grip"
            if data["emg_calibration"] != "Grip calibrated"
            else "ready"
        )
        recording = ""
        if self.trials.active:
            elapsed = (time.monotonic_ns() - self.trials.started_ns) / 1e9
            recording = f"Recording {self.trials.metadata['trial_id']} ({elapsed:.1f} s) | "
        return {
            "ready": bool(camera and imu and emg and calibrated),
            "streams_ready": bool(imu and emg),
            "recording": self.trials.active,
            "calibration_stage": stage,
            "calibration_busy": bool(
                (stage == "imu" and self.imu_requested)
                or (stage == "rest" and self.rest_requested)
                or (stage == "grip" and data["emg_calibration"].startswith("Hold a strong"))
            ),
            "message": (
                recording + f"Camera: {'arm visible' if camera else 'frame the arm and table'} | "
                f"IMUs: {len(packet['imus'])}/2"
                f"{' (hold still)' if imu and not packet['imu_calibrated'] else ''} | "
                f"EMG: {data['emg_calibration'] if emg else 'waiting'}"
                f" | Cup: {'detected' if self.live.camera_observation.get('cup_visible') else 'not detected'}"
            ),
            "next_trial": f"T{next_trial_number(self.trials.directory):03d}",
        }

    def stop_trial(self, reason="Stopped by operator", *, complete=True) -> None:
        if not self.trials.active:
            return
        path = self.trials.stop(reason, complete=complete)
        manifest = json.loads((path / "trial.json").read_text(encoding="utf-8"))
        emit("saved", path=str(path), complete=manifest["complete"])

    def command(self, action) -> None:
        values = action if isinstance(action, dict) else {"action": action}
        action = values.get("action", "")
        try:
            if action == "stop":
                self.stop_trial()
            elif action in ("close", "connection_closed"):
                self.stop_trial(
                    "Subject finished" if action == "close" else "App connection closed",
                    complete=action == "close",
                )
                self.running = False
            elif action == "graphs":
                self.show_graphs()
            elif action == "marker":
                marker = int(values.get("marker", 0))
                self.trials.mark(marker, host_monotonic_ns=values.get("host_monotonic_ns"))
                emit("cue", message=f"Marked {marker}")
                if marker == 4:
                    self.stop_trial("Hand reached marker 4; stopped by operator")
            elif self.trials.active:
                raise ValueError("Stop the trial before changing calibration")
            elif action == "record":
                if not self.state()["ready"]:
                    raise ValueError(
                        "Check the camera, calibrate both IMUs, rest and strong grip first"
                    )
                self.trials.start(values.get("condition", "natural"))
                emit(
                    "recording",
                    trial=self.trials.metadata["trial_id"],
                    condition=self.trials.metadata["pace_condition"],
                )
            elif action in ("rest", "grip", "imu"):
                if not self.state()["streams_ready"]:
                    raise ValueError("Wait for both IMUs and EMG to connect")
                if action == "rest":
                    self.live.reader.calibrate_emg()
                    self.rest_requested = True
                    emit("cue", message="Relax the muscle for half a second")
                elif action == "grip":
                    if not self.rest_requested or not self.live.reader.calibrate_emg_strength():
                        raise ValueError("Finish relaxed-muscle calibration first")
                    emit("cue", message="Hold a strong comfortable grip for 3 seconds")
                else:
                    self.live.reader.calibrate(reset_emg=False)
                    self.imu_requested = True
                    emit("cue", message="Keep the arm still in its starting pose for half a second")
            else:
                raise ValueError("Unknown recording command")
        except (OSError, RuntimeError, TypeError, ValueError) as error:
            emit("error", message=str(error))

    def show_graphs(self) -> None:
        if self.graphs is None:
            import tkinter as tk

            sys.path.insert(0, str(ROOT / "src/simulation"))
            from interactive.live_signal_graphs import LiveSignalGraphs

            self.graph_root = tk.Tk()
            self.graph_root.withdraw()
            self.graphs = LiveSignalGraphs(
                self.graph_root, lambda: self.command("rest"), lambda: self.command("grip")
            )
        self.graphs.show()

    def run(self, commands: queue.Queue) -> None:
        cv2 = self.live.cv2
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 480, 360)
        if self.args.preview_position is not None:
            cv2.moveWindow(WINDOW, *self.args.preview_position)
        last_status = 0.0
        shown = False
        while self.running:
            snapshot = self.live.poll()
            if snapshot.stop_requested:
                raise RuntimeError("The camera stopped; the current trial was kept as incomplete")
            while not commands.empty():
                self.command(commands.get_nowait())
            if not self.running:
                break
            if self.live.reader.connection_generation != self.connection_generation:
                self.connection_generation = self.live.reader.connection_generation
                self.stop_trial("Bluetooth reconnected during the trial", complete=False)
                self.rest_requested = self.imu_requested = False
            state = self.state()
            if self.trials.active:
                if not state["streams_ready"] or self.live.reader.recording_error:
                    self.stop_trial("Sensor stream or file write interrupted", complete=False)
                    self.rest_requested = self.imu_requested = False
                    emit(
                        "error", message="Trial incomplete. Reconnect and calibrate before retrying"
                    )
                elif (
                    self.args.duration
                    and (time.monotonic_ns() - self.trials.started_ns) / 1e9 >= self.args.duration
                ):
                    self.stop_trial("Trial duration reached")
            if self.live.frame_sequence != self.last_frame and self.live.latest_frame is not None:
                self.last_frame = self.live.frame_sequence
                self.trials.add_frame(self.live)
                image = self.live.latest_frame.copy()
                label = self.trials.metadata.get("trial_id", state["next_trial"])
                recording = "RECORDING" if self.trials.active else "Preview"
                cv2.rectangle(image, (0, 0), (image.shape[1], 32), (24, 30, 38), -1)
                cv2.putText(
                    image,
                    f"{self.args.subject} | {label} | {recording}",
                    (10, 23),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (100, 230, 140),
                    1,
                )
                cv2.imshow(WINDOW, image)
                shown = True
            if cv2.waitKey(1) & 0xFF == ord("q") or (
                shown and cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
            ):
                self.command("close")
            if self.graphs is not None:
                self.graphs.update(self.live.reader)
                self.graph_root.update()
            if time.monotonic() - last_status >= 0.5:
                last_status = time.monotonic()
                emit("status", **self.state())
            time.sleep(0.005)

    def close(self) -> None:
        try:
            self.stop_trial("Recording session interrupted", complete=False)
        finally:
            if self.graph_root is not None:
                self.graph_root.destroy()
            self.live.cv2.destroyAllWindows()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subject", required=True)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--device", default="LIMBServer")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--duration", type=float, default=0)
    parser.add_argument("--preview-position", type=int, nargs=2, default=None)
    parser.add_argument("--shoulder-twist-axis", choices=TWIST_AXES, default=DEFAULT_TWIST_AXIS)
    parser.add_argument("--wrist-twist-axis", choices=TWIST_AXES, default=DEFAULT_TWIST_AXIS)
    args = parser.parse_args()
    if not args.subject.strip() or not math.isfinite(args.duration) or args.duration < 0:
        parser.error("Enter a subject and a duration of 0 or more seconds")
    enable_ctrl_break_as_interrupt()
    os.environ.setdefault("PYTHONUTF8", "1")
    live = LiveSensorInput(
        side=args.side,
        device=args.device,
        transport="ble",
        synchronize_clock=True,
        show_monitor=False,
        pose_model_complexity=1,
        shoulder_twist_axis=args.shoulder_twist_axis,
        wrist_twist_axis=args.wrist_twist_axis,
    )
    recorder = None
    try:
        emit("cue", message="Connecting camera and BLE")
        live.start()
        recorder = CupRecorder(args, live)
        commands = queue.Queue()
        threading.Thread(target=read_commands, args=(commands,), daemon=True).start()
        recorder.run(commands)
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        emit("error", message=str(error))
        raise
    finally:
        try:
            if recorder is not None:
                recorder.close()
        finally:
            live.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
