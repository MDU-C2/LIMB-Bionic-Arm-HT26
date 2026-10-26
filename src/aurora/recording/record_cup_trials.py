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

from aurora.json_utils import read_json_object, write_json
from aurora.recording.common import DEFAULT_OUTPUT, enable_ctrl_break_as_interrupt
from aurora.recording.cup_input import CupSensorInput
from aurora.recording.cup_protocol import (
    BASELINE_SECONDS,
    COUNTDOWN_SECONDS,
    PACE_PROMPTS,
    REFERENCE_STEPS,
    REST_SECONDS,
    TRIAL_SECONDS,
    CupReferenceGuide,
    CupTrialGuide,
)
from aurora.recording.cup_trials import CupTrialSession, next_trial_number
from aurora.recording.subject_calibration import digest, fit_subject_reference
from aurora.sensors.imu_twist import DEFAULT_TWIST_AXIS, TWIST_AXES
from aurora.sensors.settings import DEFAULT_BLE_DEVICE_NAME

WINDOW = "AURORA | Cup recording"


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
    """Own one subject's files and cues; the GUI only sends operator commands."""

    def __init__(self, args, live):
        self.args = args
        self.live = live
        self.trials = CupTrialSession(
            args.output,
            args.subject,
            args.side,
            live.reader,
            live.cv2,
            camera_metadata=live.camera_metadata() if hasattr(live, "camera_metadata") else {},
        )
        self.guide = self.new_trial_guide()
        self.pending_protocol = {}
        self.pending_cues = []
        self.last_reference_step = None
        self.last_protocol_cue = None
        self.last_saved = None
        self.repeat_of = None
        self.saved_counts = dict.fromkeys(("natural", "slow", "fast"), 0)
        for manifest in self.trials.directory.glob("T*/trial.json"):
            try:
                data = json.loads(manifest.read_text(encoding="utf-8"))
                if data.get("complete") and data.get("pace_condition") in self.saved_counts:
                    self.saved_counts[data["pace_condition"]] += 1
            except (OSError, ValueError, AttributeError):
                continue
        self.running = True
        self.graph_root = None
        self.graphs = None
        self.last_frame = -1
        self.connection_generation = live.reader.connection_generation
        self.reference_path = None
        self.reference_checks = {}
        self.alignment_profile = None
        # The CLI sets this for normal participant collection. Raw-only callers
        # remain available for diagnostics without inventing a calibration.
        self.require_alignment = getattr(args, "require_alignment", True)
        self.depth_was_receiving = True
        self.streams_were_ready = True
        self.capture_started = {}
        self.next_trial = f"T{next_trial_number(self.trials.directory):03d}"

    def capture_health(self):
        return self.live.capture_health() if hasattr(self.live, "capture_health") else {}

    def stream_state(self):
        """Check receipt times without copying every graph trace each video tick."""
        packet = self.live.reader.latest()
        frame = getattr(self.live, "camera_frame", None) or self.live
        camera = bool(
            frame.frame_time_ns > 0 and time.monotonic_ns() - frame.frame_time_ns <= 1_000_000_000
        )
        imu = len(packet["imus"]) == 2
        emg = bool(packet["emg_time"] > 0 and self.live.reader.clock() - packet["emg_time"] <= 0.5)
        return camera, imu, emg, packet

    def collect_frames(self):
        """Drain received frames separately from slower, optional tracking results."""
        if not hasattr(self.live, "drain_frames"):
            return
        for frame in self.live.drain_frames():
            self.trials.add_frame(frame, save_observation=False)
        for frame in self.live.drain_tracking():
            self.trials.add_tracking(frame)
        if hasattr(self.live, "drain_depth"):
            for frame in self.live.drain_depth():
                if self.trials.active:
                    self.trials.camera.add_depth(frame)

    def monitor_streams(self):
        """Flag gaps while keeping available data; reconnects invalidate preparation."""
        if self.live.reader.connection_generation != self.connection_generation:
            self.connection_generation = self.live.reader.connection_generation
            self.reference_path = None
            self.reference_checks.clear()
            self.alignment_profile = None
            if self.trials.active:
                self.trials.metadata["stream_interrupted"] = True
                self.trials.metadata["subject_reference_invalidated"] = True
                self.trials.cue("ble_reconnected", connection_generation=self.connection_generation)
        camera, imu, emg, _ = self.stream_state()
        ready = camera and imu and emg
        if self.trials.active and ready != self.streams_were_ready:
            self.streams_were_ready = ready
            self.trials.cue("streams_recovered" if ready else "stream_gap")
            if not ready:
                self.trials.metadata["stream_interrupted"] = True
        if self.trials.active and self.trials.camera_metadata.get("stereo_depth_enabled"):
            receiving = hasattr(self.live, "depth_receiving") and self.live.depth_receiving()
            if receiving != self.depth_was_receiving:
                self.depth_was_receiving = receiving
                self.trials.cue("depth_recovered" if receiving else "depth_gap")
                if not receiving:
                    self.trials.metadata["depth_interrupted"] = True

    def preview_frame(self):
        # Never switch the video backwards to a slower inference result.
        return getattr(self.live, "camera_frame", None)

    def state(self) -> dict:
        camera, imu, emg, packet = self.stream_state()
        frame = getattr(self.live, "camera_frame", None) or self.live
        tracking = getattr(self.live, "tracking_frame", None) or frame
        recording = ""
        if self.trials.active:
            elapsed = (time.monotonic_ns() - self.trials.started_ns) / 1e9
            recording = f"Recording {self.trials.metadata['trial_id']} ({elapsed:.1f} s) | "
        depth_enabled = self.trials.camera_metadata.get("stereo_depth_enabled", False)
        depth_receiving = hasattr(self.live, "depth_receiving") and self.live.depth_receiving()
        stereo = (
            self.live.stereo_status() if hasattr(self.live, "stereo_status") else {"ready": False}
        )
        alignment_ready = bool(
            self.alignment_profile
            and self.alignment_profile.get("status") == "passed"
            and self.alignment_profile.get("connection_generation")
            == self.live.reader.connection_generation
            and self.alignment_profile.get("subject_id") == self.args.subject
            and self.alignment_profile.get("side") == self.args.side
        )
        reference_ready = all(
            self.reference_checks.get(key, False)
            for key in ("imu_zero_ready", "emg_rest_ready", "emg_grip_ready", "sensor_to_arm_ready")
        )
        available = bool(
            camera
            and imu
            and emg
            and not self.guide.active
            and time.monotonic_ns() >= self.guide.rest_until_ns
        )
        health = self.capture_health()
        return {
            "ready": bool(camera and imu and emg),
            "streams_ready": bool(imu and emg),
            "recording": self.trials.active,
            "protocol": self.guide.view(),
            "saved_counts": dict(self.saved_counts),
            "can_start": available,
            "can_prepare": available
            and (not self.require_alignment or stereo.get("capture_ready", stereo["ready"])),
            "subject_reference": self.reference_path,
            "alignment_ready": alignment_ready,
            "calibration_ready": alignment_ready and reference_ready,
            "stereo": stereo,
            "can_record": bool(
                available
                and (
                    not self.require_alignment
                    or (alignment_ready and reference_ready and stereo["ready"])
                )
            ),
            "message": (
                recording
                + ("Streams receiving | " if camera and imu and emg and not recording else "")
                + f"Camera: {'receiving' if camera else 'waiting for frames'} | "
                f"IMUs: {len(packet['imus'])}/2"
                + f" | Raw EMG: {'receiving' if emg else 'waiting'}"
                + (
                    " | Depth: receiving"
                    if depth_enabled and depth_receiving
                    else " | Depth: waiting; RGB still available"
                    if depth_enabled
                    else " | RGB only"
                )
                + (
                    " | Camera calibration unavailable; metric 3D unavailable"
                    if self.trials.camera_metadata.get("camera_calibration_error")
                    else ""
                )
                + f" | Arm: {'visible' if camera and getattr(tracking, 'camera_observation', {}).get('arm_visible') else 'not tracked'}"
                + (
                    " | Reference saved"
                    if self.reference_path and reference_ready
                    else " | Reference saved; check IMU/EMG references in Signals"
                    if self.reference_path
                    else " | Calibrate the subject before trials"
                )
                + (
                    " | Calibration: " + self.reference_checks["alignment_message"]
                    if self.reference_checks.get("alignment_message") and not alignment_ready
                    else " | Alignment checks passed"
                    if alignment_ready
                    else ""
                )
                + (
                    " | EMG reference: "
                    + self.reference_checks.get("emg_message", "Repeat relaxed and grip references")
                    if alignment_ready and not reference_ready
                    else ""
                )
                + (
                    " | Stereo arm points: valid"
                    if stereo["ready"]
                    else " | "
                    + stereo.get("message", "Show both shoulders, elbow and wrist for stereo")
                    if depth_enabled
                    else ""
                )
                + (
                    " | Tracking unavailable; raw capture continues"
                    if health.get("tracking_error")
                    else ""
                )
                + (
                    " | Stream gap: available signals still recording"
                    if self.trials.active and not (camera and imu and emg)
                    else ""
                )
            ),
            "next_trial": self.next_trial,
        }

    def stop_trial(self, reason="Stopped by operator", *, complete=True) -> None:
        """Cancel an unopened countdown or keep a started attempt with its actual status."""
        if self.guide.active and not self.trials.active:
            practice = self.guide.practice
            self.guide.finish()
            self.pending_cues.clear()
            emit(
                "cue",
                message="Practice finished; no files saved."
                if practice
                else "Countdown cancelled; no files created.",
            )
            self._publish_protocol()
            return
        if not self.trials.active:
            return
        reference = isinstance(self.guide, CupReferenceGuide)
        complete = complete and (
            self.guide.completed
            if reference
            else self.guide.go_ns is not None
            and (not self.guide.duration_s or self.guide.window_finished)
        )
        self.collect_frames()
        health = self.capture_health()
        self.trials.metadata["capture_health"] = {
            key: value - self.capture_started.get(key, 0) if isinstance(value, int) else value
            for key, value in health.items()
        }
        self.trials.cue("stop", reason=reason)
        if reference:
            data = self.live.reader.graph_data()
            self.trials.metadata["calibration_after"] = self.live.reader.calibration_profile()
            self.trials.metadata["reference_checks"] = {
                "imu_zero_ready": bool(self.live.reader.latest()["imu_calibrated"]),
                "emg_rest_ready": bool(self.live.reader.emg_estimator.calibrated),
                "emg_grip_ready": data["emg_calibration"] == "Grip calibrated",
                "emg_message": data["emg_calibration"],
            }
        path = self.trials.stop(reason, complete=complete)
        self.next_trial = f"T{next_trial_number(self.trials.directory):03d}"
        manifest = json.loads((path / "trial.json").read_text(encoding="utf-8"))
        if reference:
            emit("cue", message="Fitting sensor-to-arm calibration from the saved reference...")
            profile = fit_subject_reference(path, manifest)
            write_json(path / "alignment.json", profile)
            checks = manifest["reference_checks"]
            checks.update(
                sensor_to_arm_ready=profile["status"] == "passed",
                alignment_message=profile.get("message", "Alignment checks passed"),
            )
            manifest.update(
                sensor_to_arm_alignment="fitted" if checks["sensor_to_arm_ready"] else "failed",
                alignment_file="alignment.json",
                alignment_sha256=digest(path / "alignment.json"),
                reference_checks=checks,
            )
            write_json(path / "trial.json", manifest)
            for source in ("camera", "ble"):
                metadata_path = path / source / "meta.json"
                metadata = read_json_object(metadata_path)
                metadata.update(
                    {
                        key: manifest[key]
                        for key in (
                            "sensor_to_arm_alignment",
                            "alignment_file",
                            "alignment_sha256",
                            "reference_checks",
                        )
                    }
                )
                write_json(metadata_path, metadata)
            self.reference_checks = checks
            if manifest["complete"]:
                self.reference_path = path.relative_to(self.trials.directory).as_posix()
                self.alignment_profile = profile if checks["sensor_to_arm_ready"] else None
            self.guide = self.new_trial_guide()
        else:
            self.last_saved = manifest["trial_id"]
        if manifest["complete"] and not reference:
            self.saved_counts[manifest["pace_condition"]] += 1
        self.guide.finish()
        emit(
            "saved",
            path=str(path),
            complete=manifest["complete"],
            reference=reference,
            checks=manifest.get("reference_checks"),
        )
        self._publish_protocol()

    def new_trial_guide(self):
        return CupTrialGuide(
            baseline_s=getattr(self.args, "baseline", BASELINE_SECONDS),
            countdown_s=getattr(self.args, "countdown", COUNTDOWN_SECONDS),
            rest_s=getattr(self.args, "rest", REST_SECONDS),
            duration_s=getattr(self.args, "duration", TRIAL_SECONDS),
        )

    def begin_reference(self):
        if not self.state()["can_start"]:
            raise ValueError("Wait for camera, both IMUs, EMG and the end of rest")
        stereo = self.state()["stereo"]
        if self.require_alignment and not stereo.get("capture_ready", stereo["ready"]):
            raise ValueError(stereo.get("message", "Wait for stereo depth and camera calibration"))
        self.capture_started = self.capture_health()
        self.trials.start(
            reference=True,
            mark_start=False,
            protocol={
                "countdown_seconds": COUNTDOWN_SECONDS,
                "reference_step_start": "operator",
                "countdown_before_each_step_seconds": COUNTDOWN_SECONDS,
                "reference_steps": [
                    {"cue": cue, "seconds": seconds, "instruction": instruction}
                    for cue, seconds, instruction in REFERENCE_STEPS
                ],
                "cue_delivery": "operator",
                "alignment_status": "functional_axes_and_static_reference",
            },
        )
        self.reference_path = None
        self.reference_checks.clear()
        self.alignment_profile = None
        self.guide = CupReferenceGuide()
        self.streams_were_ready = True
        self.last_protocol_cue = None
        self.last_reference_step = None
        self._publish_protocol()
        emit(
            "reference",
            message="Calibration: still pose, arm raises, elbow bends, forearm turns, return, then EMG references. Operator guides each step.",
        )

    def _begin_trial(self, condition, *, practice=False, duration=None):
        if not self.state()["ready"]:
            raise ValueError("Wait for camera frames, both IMUs and raw EMG samples")
        if self.guide.active or time.monotonic_ns() < self.guide.rest_until_ns:
            raise ValueError("Finish the current trial and rest before starting again")
        if not practice and self.require_alignment and not self.state()["can_record"]:
            raise ValueError(
                "Complete subject calibration and check stereo depth before recording. See the status for the step to repeat."
            )
        if duration is not None:
            duration = float(duration)
            if not math.isfinite(duration) or duration < 0:
                raise ValueError("Recording window must be 0 or more seconds")
            self.guide.duration_s = duration
        # Open files at GO. Each source keeps its own actual sample/receipt times.
        self.pending_protocol = {
            "baseline_seconds": self.guide.baseline_s,
            "countdown_seconds": self.guide.countdown_s,
            "recording_seconds": self.guide.duration_s,
            "rest_seconds": self.guide.rest_s,
            "movement_end": "unverified",
            "recording_end": "timer" if self.guide.duration_s else "operator",
            "cue_delivery": "operator",
            "repeat_of": self.repeat_of,
        }
        self.pending_cues = []
        self.guide.begin(condition, practice=practice)
        self.streams_were_ready = True
        self.depth_was_receiving = True
        self.repeat_of = None
        self.last_protocol_cue = None
        self._publish_protocol()
        emit(
            "recording" if not practice else "practice",
            trial=self.state()["next_trial"],
            condition=condition,
        )

    def _publish_protocol(self):
        """Apply each cue transition once, independently of the GUI refresh rate."""
        data = self.guide.view()
        data["trial"] = self.trials.metadata.get("trial_id", "") if self.trials.active else ""
        key = data["phase"], data["cue"]
        if key == self.last_protocol_cue:
            return
        stamp = time.monotonic_ns()
        if (
            data["phase"] in ("movement", "practice")
            and self.last_protocol_cue
            and self.last_protocol_cue[0] == "countdown"
        ):
            if not self.guide.practice:
                if not self.state()["ready"]:
                    self.guide.go_ns = None
                    self.stop_trial("Streams unavailable at GO", complete=False)
                    emit("error", message="Start cancelled: wait for camera, EMG and both IMUs.")
                    return
                if self.require_alignment and not (
                    self.state()["stereo"]["ready"] and self.state()["calibration_ready"]
                ):
                    self.guide.go_ns = None
                    self.stop_trial("Stereo unavailable at GO", complete=False)
                    emit(
                        "error",
                        message="Start cancelled: restore stereo arm points and repeat subject calibration if BLE reconnected.",
                    )
                    return
                self.capture_started = self.capture_health()
                self.trials.start(
                    self.guide.condition,
                    protocol=self.pending_protocol,
                    reference_path=self.reference_path,
                    alignment_profile=self.alignment_profile,
                )
                for cue, cue_stamp, phase in self.pending_cues:
                    self.trials.cue(
                        cue, host_monotonic_ns=cue_stamp, phase=phase, delivery="operator"
                    )
                self.pending_cues.clear()
                stamp = time.monotonic_ns()
                self.trials.metadata["movement_go_host_monotonic_ns"] = stamp
            self.guide.go_ns = self.guide.phase_started_ns = stamp
        if data["phase"] == "reference" and data["reference_step"] != self.last_reference_step:
            self.last_reference_step = data["reference_step"]
            cue = REFERENCE_STEPS[self.last_reference_step][0]
            if cue == "STILL":
                self.live.reader.calibrate(reset_emg=False)
            elif cue == "RELAX":
                self.live.reader.calibrate_emg()
            elif cue == "GRIP":
                self.live.reader.calibrate_emg_strength()
        if not self.trials.active and data["phase"] in ("baseline", "countdown"):
            self.pending_cues.append((data["cue"], stamp, data["phase"]))
        self.trials.cue(
            data["cue"],
            host_monotonic_ns=stamp,
            phase=data["phase"],
            delivery="operator",
            reference_step=data.get("reference_step"),
        )
        self.last_protocol_cue = key
        emit("protocol", **data)

    def advance_protocol(self):
        self.guide.tick()
        if isinstance(self.guide, CupReferenceGuide) and self.guide.completed:
            self.stop_trial("Subject reference sequence finished")
            return
        self._publish_protocol()
        if (
            isinstance(self.guide, CupTrialGuide)
            and self.guide.active
            and self.guide.window_finished
        ):
            self.stop_trial("Recording window finished")

    def command(self, action) -> None:
        values = action if isinstance(action, dict) else {"action": action}
        action = values.get("action", "")
        try:
            if action == "reference":
                self.begin_reference()
            elif action == "reference_step":
                if not isinstance(self.guide, CupReferenceGuide):
                    raise ValueError("Press Calibrate subject first")
                self.guide.start_step()
                self._publish_protocol()
            elif action == "stop":
                self.stop_trial()
            elif action in ("close", "connection_closed"):
                self.stop_trial(
                    "Subject finished" if action == "close" else "App connection closed",
                    complete=action == "close",
                )
                self.running = False
            elif action == "graphs":
                self.show_graphs()
            elif action == "pace":
                if self.guide.active:
                    raise ValueError("Choose the next pace after the current trial")
                if values.get("condition") not in PACE_PROMPTS:
                    raise ValueError("Choose natural, slow or fast pace")
                self.guide.condition = values["condition"]
                self.last_protocol_cue = None
                self._publish_protocol()
            elif action == "retry":
                if isinstance(self.guide, CupReferenceGuide):
                    self.stop_trial("Reference repeat requested", complete=False)
                    emit(
                        "cue",
                        message="Reference attempt kept. Press Calibrate subject again.",
                    )
                    return
                self.repeat_of = (
                    self.trials.metadata.get("trial_id") if self.trials.active else self.last_saved
                )
                if self.trials.active:
                    self.trials.metadata["retry_requested"] = True
                self.stop_trial("Restart requested; keep this attempt", complete=False)
                emit(
                    "retry",
                    condition=self.guide.condition,
                    message="Reset cup to 2 and hand to 1, then start after rest. Earlier attempts stay saved.",
                )
            elif action == "record":
                self._begin_trial(
                    values.get("condition", "natural"), duration=values.get("duration")
                )
            elif action == "practice":
                self._begin_trial(
                    values.get("condition", "natural"),
                    practice=True,
                    duration=values.get("duration"),
                )
            else:
                raise ValueError("Unknown recording command")
        except (OSError, RuntimeError, TypeError, ValueError) as error:
            emit("error", message=str(error))

    def show_graphs(self) -> None:
        if self.graphs is None:
            import tkinter as tk

            from aurora.gui.sensor_graphs import LiveSignalGraphs

            if self.graph_root is None:
                self.graph_root = tk.Tk()
                self.graph_root.withdraw()
            self.graphs = LiveSignalGraphs(self.graph_root, show_activation=False)
        self.graphs.show()

    def run(self, commands: queue.Queue) -> None:
        cv2 = self.live.cv2
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 480, 360)
        if self.args.preview_position is not None:
            x, y = self.args.preview_position
            cv2.moveWindow(WINDOW, x, y)
        last_status = 0.0
        last_state = 0.0
        state = None
        shown = False
        while self.running:
            self.live.poll()
            self.collect_frames()
            self.monitor_streams()
            while not commands.empty():
                self.command(commands.get_nowait())
            if not self.running:
                break
            if state is None or time.monotonic() - last_state >= 0.1:
                state = self.state()
                last_state = time.monotonic()
            if self.trials.active and self.live.reader.recording_error:
                self.stop_trial("File write interrupted", complete=False)
                emit(
                    "error",
                    message="File write failed. Attempt kept as incomplete; check the output drive.",
                )
            self.advance_protocol()
            frame = self.preview_frame()
            if frame is not None and frame.frame_sequence > self.last_frame:
                self.last_frame = frame.frame_sequence
                image = frame.latest_frame.copy()
                if frame is getattr(self.live, "camera_frame", None):
                    image = image[:, ::-1].copy()
                label = (
                    self.trials.metadata["trial_id"] if self.trials.active else state["next_trial"]
                )
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
                tracked = getattr(self.live, "tracking_frame", None)
                age_ms = (time.monotonic_ns() - tracked.frame_time_ns) / 1e6 if tracked else None
                observation = (
                    tracked.camera_observation if age_ms is not None and age_ms < 1000 else {}
                )
                tracking_label = (
                    f"Last tracking: {age_ms:.0f} ms ago"
                    if observation
                    else "Waiting for arm tracking"
                )
                tracking_label += " | hand " + (
                    "found" if observation.get("hand_visible") else "--"
                )
                tracking_label += " | cup " + ("found" if observation.get("cup_visible") else "--")
                cv2.rectangle(
                    image,
                    (0, image.shape[0] - 28),
                    (image.shape[1], image.shape[0]),
                    (24, 30, 38),
                    -1,
                )
                cv2.putText(
                    image,
                    tracking_label,
                    (10, image.shape[0] - 9),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (220, 225, 230),
                    1,
                )
                cv2.imshow(WINDOW, image)
                shown = True
            if cv2.waitKey(1) & 0xFF == ord("q") or (
                shown and cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
            ):
                self.command("close")
            if self.graph_root is not None:
                if self.graphs is not None:
                    self.graphs.update(self.live.reader)
                self.graph_root.update()
            if time.monotonic() - last_status >= 0.25:
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
    parser.add_argument("--subject-id", "--subject", dest="subject", required=True)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--device", default=DEFAULT_BLE_DEVICE_NAME)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--duration",
        type=float,
        default=TRIAL_SECONDS,
        help="Recording window after GO in seconds; 0 selects manual stopping.",
    )
    parser.add_argument(
        "--depth",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Record aligned stereo depth; --no-depth records RGB only.",
    )
    parser.add_argument(
        "--raw-only",
        action="store_true",
        help="Diagnostic capture without subject alignment; not the participant collection workflow.",
    )
    parser.add_argument("--baseline", type=float, default=BASELINE_SECONDS)
    parser.add_argument("--countdown", type=float, default=COUNTDOWN_SECONDS)
    parser.add_argument("--rest", type=float, default=REST_SECONDS)
    parser.add_argument("--preview-position", type=int, nargs=2, default=None)
    parser.add_argument("--shoulder-twist-axis", choices=TWIST_AXES, default=DEFAULT_TWIST_AXIS)
    parser.add_argument("--wrist-twist-axis", choices=TWIST_AXES, default=DEFAULT_TWIST_AXIS)
    args = parser.parse_args()
    args.require_alignment = not args.raw_only
    if args.require_alignment and not args.depth:
        parser.error(
            "Participant recordings require stereo depth. Use --raw-only for RGB diagnostics."
        )
    if not args.subject.strip() or any(
        not math.isfinite(v) or v < 0
        for v in (args.duration, args.baseline, args.countdown, args.rest)
    ):
        parser.error("Enter a subject and a duration of 0 or more seconds")
    enable_ctrl_break_as_interrupt()
    os.environ.setdefault("PYTHONUTF8", "1")
    live = CupSensorInput(
        side=args.side,
        device=args.device,
        shoulder_twist_axis=args.shoulder_twist_axis,
        wrist_twist_axis=args.wrist_twist_axis,
        depth=args.depth,
    )
    recorder = None
    try:
        emit("cue", message="Connecting camera and BLE")
        live.start()
        live.start_background()
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
