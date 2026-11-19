# Majd
"""Guide live calibration before enabling arm movement."""

from __future__ import annotations

import time

from aurora.paths import RECORDINGS_DIRECTORY
from aurora.recording.common import announce_result
from aurora.recording.cup_protocol import REFERENCE_STEPS, CupReferenceGuide


class LiveSetup:
    def __init__(self, live, reference, *, subject_id="diagnostic", output=None):
        self.live, self.reference = live, reference
        self.subject_id = subject_id
        self.output = output or RECORDINGS_DIRECTORY.parent / "calibration"
        self.functional = getattr(live, "depth", False) is True
        self.stage = "mount" if self.functional else "align"
        self.capture = self.mount_guide = None
        self._now_ns = 0
        self.pending = False
        self.still_since = None
        self.previous_angles = None
        self.connection_generation = live.reader.connection_generation
        self.latest_snapshot = None
        self.preparation = None
        self.message = (
            "Fit sensor mounting: upper-arm IMU 0x6B, forearm IMU 0x6A. "
            "Show shoulders, hips, elbow and wrist. Follow five guided movements."
            if self.functional
            else "Configured-axis diagnostic: sensor axes must match the advanced settings. "
            "Enable stereo depth for guided mounting calibration. "
            "Rest your open hand at marker 1, thumb up, elbow about 90°."
        )

    def cancel(self):
        if self.capture is not None:
            if self.live.reader is not None:
                self.live.reader.detach_reference_capture(self.capture)
            if not self.capture.started:
                self.capture.error = "Calibration cancelled; repeat all reference movements"
                self.capture.end_step(round(self.live.clock() * 1e9))
                if self.capture.stages:
                    self.capture.finish(self.output)
            self.capture = self.mount_guide = None

    def _restart(self, message):
        self.cancel()
        self.preparation = None
        self.stage, self.pending = ("mount" if self.functional else "align"), False
        self.message = message

    def close(self):
        capture = self.capture
        self.cancel()
        if capture is not None:
            try:
                capture.close()
            except Exception:
                self.capture = capture
                raise

    def action(self) -> bool:
        if self.pending:
            return False
        if self.stage == "mount":
            if self.latest_snapshot is None or not (
                self.latest_snapshot.camera_ready and self.latest_snapshot.imu_ready
            ):
                self.message = "Wait for both arm IMUs and the camera before calibrating mounting."
                return False
            self._now_ns = round(self.live.clock() * 1e9)
            if self.capture is None:
                self.capture = self.live.reader.begin_reference_capture(self.subject_id)
                self.mount_guide = CupReferenceGuide(
                    clock=lambda: self._now_ns, steps=REFERENCE_STEPS[:5]
                )
            self.mount_guide.start_step()
            self.pending = True
        elif self.stage == "align":
            self.pending = True
            self.still_since = self.previous_angles = None
            self.message = (
                "Keep your trunk and arm still for 2 seconds. Keep shoulders, hips and arm in view."
            )
        elif self.stage == "rest":
            self._prepare_emg("RELAX")
            self.pending = True
            self.message = "Keep the hand open and the muscle relaxed. Wait for the rest baseline."
        elif self.stage == "grip":
            self._prepare_emg("GRIP")
            self.pending = True
            self.message = "Prepare to grip firmly for 3 seconds. Begin only on GO."
        elif self.stage == "ready":
            snapshot = self.latest_snapshot
            away = any(
                abs(self.live.raw_camera_angles.get(name, origin) - origin) > 12
                for name, origin in self.live.camera_origin.items()
                if name != "shoulder_rotation_proxy"
            )
            if (
                snapshot is None
                or not snapshot.calibration_valid
                or not snapshot.camera_ready
                or not snapshot.imu_ready
                or away
            ):
                self.message = "Return the hand to marker 1 and show the arm before starting."
                return False
            if self.live.grip_source != "camera" and (
                snapshot.emg_activation is None or snapshot.emg_activation > 0.2
            ):
                self.message = "Open the hand and relax before starting."
                return False
            self.stage = "running"
            self.message = (
                "Control active. Check one movement at a time. Pause if a direction is wrong."
            )
            return True
        return False

    def _prepare_emg(self, cue):
        if hasattr(self.live, "clock"):
            self._now_ns = round(self.live.clock() * 1e9)
        step = next(step for step in REFERENCE_STEPS if step[0] == cue)
        self.preparation = CupReferenceGuide(clock=lambda: self._now_ns, steps=(step,))
        self.preparation.start_step()

    def repeat(self):
        self.live.invalidate_calibration()
        self._restart("Mounting changed. Repeat preparation before enabling control.")

    def pause(self):
        if (
            self.stage in {"ready", "running"}
            and self.latest_snapshot is not None
            and self.latest_snapshot.calibration_valid
            and self.live.reader.connection_generation == self.connection_generation
        ):
            self.stage, self.pending = "ready", False
            self.message = "Paused. Calibration retained for this connection and mounting. Return to marker 1, then Start control."
        else:
            self._restart("Preparation interrupted. Repeat the displayed step before control.")

    def view(self):
        if self.mount_guide is not None and self.stage == "mount":
            return self.mount_guide.view()
        if self.preparation is not None:
            return self.preparation.view()
        movement = {
            "align": "STILL",
            "imu": "STILL",
            "rest": "RELAX",
            "grip": "GRIP",
            "running": "CUP",
        }.get(self.stage, "RETURN")
        progress = 0
        if self.stage == "align" and self.pending and self.still_since is not None:
            progress = min(1, max(0, (self._now_ns / 1e9 - self.still_since) / 2))
        if self.stage in {"rest", "grip"} and self.pending:
            state = self.live.reader.graph_data()["emg_calibration_state"]
            progress = state.get("progress", 0)
        return {
            "movement": movement,
            "phase": self.stage,
            "cue": f"GO: {movement}"
            if self.pending and self.stage in {"rest", "grip"}
            else self.button,
            "instruction": self.message,
            "progress": progress,
        }

    def update(self, snapshot, now=None) -> None:
        now = time.monotonic() if now is None else now
        self._now_ns = round(now * 1e9)
        self.latest_snapshot = snapshot
        if self.stage in ("running", "ready") and not snapshot.calibration_valid:
            self._restart("Sensor reference invalidated. Repeat mounting and calibration.")
            return
        if self.live.reader.connection_generation != self.connection_generation:
            self.connection_generation = self.live.reader.connection_generation
            self._restart("Bluetooth reconnected. Repeat mounting and calibration.")
            return
        if self.stage == "running":
            if (not snapshot.camera_ready and not snapshot.imu_ready) or (
                self.live.grip_source != "camera" and snapshot.emg_raw is None
            ):
                self._restart("Tracking lost. Reconnect and repeat the reference movements.")
            return
        if self.stage in ("mount", "fit"):
            self._update_mounting(snapshot, now)
            return
        if not self.pending:
            return
        if self.preparation is not None:
            self.preparation.tick()
            if self.preparation.phase == "reference_countdown":
                view = self.preparation.view()
                self.message = view["cue"] + ": " + view["instruction"]
                return
            self.preparation = None
            if self.stage == "rest":
                self.live.calibrate_emg()
                self.message = "GO. Keep the hand open and muscle relaxed while the rest baseline is collected."
            elif not self.live.calibrate_emg_strength():
                self.pending = False
                self.message = "Finish a valid relaxed baseline before grip calibration."
                return
            else:
                self.message = "GO. Hold a firm, comfortable grip for 3 seconds, then relax."
        if self.stage == "align":
            angles = self.live.raw_camera_angles
            if (
                len(snapshot.sensors) != 2
                or not snapshot.imu_ready
                or (self.live.grip_source != "camera" and snapshot.emg_raw is None)
            ):
                self.still_since = None
                self.message = (
                    "Waiting for the arm IMUs and EMG. Check the BLE connection before aligning."
                )
                return
            moving = any(
                abs(value) > 5
                for sensor in snapshot.sensors.values()
                for value in sensor.get("gyro_dps", {}).values()
            )
            changed = self.previous_angles is not None and any(
                abs(value - self.previous_angles.get(name, value)) > 5
                for name, value in angles.items()
            )
            if self.previous_angles is None or changed:
                self.previous_angles = dict(angles)
            if not snapshot.camera_ready or not angles or moving or changed:
                self.still_since = None
                self.message = "Hold still at marker 1. Show both shoulders, hips, elbow and wrist."
                return
            if self.still_since is None:
                self.still_since = now
            if now - self.still_since >= 2:
                self.live.calibrate(self.reference)
                self.stage, self.pending = "imu", True
                self.message = (
                    "Starting pose aligned. Keep still while the IMUs finish their rest baseline."
                )
        elif self.stage == "imu":
            packet = self.live.reader.latest()
            if packet.get("imu_calibrated"):
                self.stage, self.pending = (
                    ("ready" if self.live.grip_source == "camera" else "rest"),
                    False,
                )
                self.message = (
                    "Arm aligned. Return to marker 1, then press 4. Start control."
                    if self.stage == "ready"
                    else "Arm aligned. Relax the muscle, then press 2. Calibrate relaxed EMG."
                )
        elif self.stage in ("rest", "grip"):
            graph = self.live.reader.graph_data()
            state = graph["emg_calibration_state"]
            status = state["message"]
            if state["phase"] in ("clipping", "gap", "failed"):
                self.pending = False
                self.message = status + ". Check the signal, then repeat this step."
                return
            if self.stage == "rest" and state["phase"] == "grip_reference_needed":
                self.stage, self.pending = "grip", False
                self.message = "Rest baseline saved. Press 3, then hold a firm grip for 3 seconds."
            elif self.stage == "grip":
                if state["phase"] == "ready":
                    self.stage, self.pending = "ready", False
                    self.message = "Grip calibrated. Return the open hand to marker 1, then press 4. Start control."

    def _update_mounting(self, snapshot, now):
        self._now_ns = round(now * 1e9)
        if self.stage == "fit":
            if not self.capture.finished:
                return
            profile = self.capture.result
            if profile["status"] != "passed":
                self._restart(
                    profile.get("message", "Mounting fit failed") + ". Repeat preparation."
                )
                return
            if not (snapshot.camera_ready and snapshot.imu_ready):
                self.message = (
                    "Fit passed. Return to the first pose and show both arm IMUs and camera."
                )
                return
            angles = self.live.raw_camera_angles
            if any(
                name not in angles or abs(angles[name] - value) > 12
                for name, value in (
                    ("elbow_flexion", profile["reference_elbow_flexion_deg"]),
                    ("shoulder_flexion", 0),
                    ("shoulder_abduction", 0),
                )
            ):
                self.message = (
                    "Fit passed. Return the upper arm beside the body, elbow 90 degrees, thumb up."
                )
                return
            try:
                self.live.reader.apply_alignment(
                    profile, subject_id=self.subject_id, side=self.live.side
                )
                self.live.calibrate(self.reference)
            except ValueError as error:
                self._restart(str(error))
                return
            announce_result(self.capture.directory, "reference")
            self.stage, self.pending = "imu", True
            self.message = "Mounting passed. Keep still at marker 1; preparing EMG calibration."
            return
        if self.mount_guide is None:
            return
        previous_phase = self.mount_guide.phase
        self.mount_guide.tick()
        phase = self.mount_guide.phase
        if previous_phase == "reference" and phase != "reference":
            self.capture.end_step(self._now_ns)
        if previous_phase != "reference" and phase == "reference":
            self.capture.begin_step(REFERENCE_STEPS[self.mount_guide.step][0], self._now_ns)
        frame = self.live.camera_frame
        if frame is not None:
            self.capture.add_camera(
                self.live.camera_observation,
                host_ns=frame.frame_time_ns,
                device_ns=frame.device_time_ns,
            )
        if self.mount_guide.completed:
            self.live.reader.detach_reference_capture(self.capture)
            self.capture.finish(self.output)
            self.stage, self.pending = "fit", True
            self.message = "Checking mounting axes, movement isolation, depth and return error..."
            return
        view = self.mount_guide.view()
        self.message = view["cue"] + ": " + view["instruction"]
        if phase == "reference":
            if not (
                snapshot.camera_ready
                and "keypoints_camera_m" in self.live.camera_observation
                and self.live.camera_observation.get("depth_time_difference_ns") is not None
            ):
                self.message += " Stereo arm points missing: restore the camera view."
            elif REFERENCE_STEPS[self.mount_guide.step][0] in ("STILL", "RETURN") and any(
                abs(value) > 5
                for sensor in snapshot.sensors.values()
                for value in sensor.get("gyro_dps", {}).values()
            ):
                self.message += " Arm moving: keep still."
        self.pending = phase != "reference_prepare"

    @property
    def button(self):
        if self.stage == "mount":
            return (
                "Start guided mounting calibration"
                if self.mount_guide is None
                else "Start reference step"
            )
        return {
            "fit": "Checking reference...",
            "align": "1. Align starting pose (hold still 2 s)",
            "imu": "Finishing IMU baseline...",
            "rest": "2. Calibrate relaxed EMG",
            "grip": "3. Calibrate firm grip (3 s)",
            "ready": "4. Start interactive control",
            "running": "Control active",
        }[self.stage]
