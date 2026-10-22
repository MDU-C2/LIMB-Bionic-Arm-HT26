# Majd
"""Guide live calibration before enabling arm movement."""

from __future__ import annotations

import time


class LiveSetup:
    def __init__(self, live, reference):
        self.live, self.reference = live, reference
        self.stage = "align"
        self.pending = False
        self.still_since = None
        self.previous_angles = None
        self.connection_generation = live.reader.connection_generation
        self.latest_snapshot = None
        self.message = "Sit upright. Rest your open hand at marker 1, thumb up, elbow about 90°."

    def action(self) -> bool:
        if self.pending:
            return False
        if self.stage == "align":
            self.pending = True
            self.still_since = self.previous_angles = None
            self.message = (
                "Keep your trunk and arm still for 2 seconds. Keep shoulders, hips and arm in view."
            )
        elif self.stage == "rest":
            self.live.calibrate_emg()
            self.pending = True
            self.message = "Keep the hand open and the muscle relaxed. Wait for the rest baseline."
        elif self.stage == "grip":
            self.pending = bool(self.live.calibrate_emg_strength())
            self.message = (
                "Grip firmly for 3 seconds, then relax."
                if self.pending
                else "Finish the rest baseline first."
            )
        elif self.stage == "ready":
            snapshot = self.latest_snapshot
            away = any(
                abs(self.live.raw_camera_angles.get(name, origin) - origin) > 12
                for name, origin in self.live.camera_origin.items()
                if name != "shoulder_rotation_proxy"
            )
            if snapshot is None or not snapshot.camera_ready or not snapshot.imu_ready or away:
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

    def update(self, snapshot, now=None) -> None:
        now = time.monotonic() if now is None else now
        self.latest_snapshot = snapshot
        if self.live.reader.connection_generation != self.connection_generation:
            self.connection_generation = self.live.reader.connection_generation
            self.stage, self.pending = "align", False
            self.message = "Bluetooth reconnected. Align and calibrate again."
            return
        if self.stage == "running":
            if (not snapshot.camera_ready and not snapshot.imu_ready) or (
                self.live.grip_source != "camera" and snapshot.emg_raw is None
            ):
                self.stage, self.pending = "align", False
                self.message = "Tracking lost. Reconnect and align the starting pose again."
            return
        if not self.pending:
            return
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
            status = graph.get("emg_calibration", "")
            if "clipping" in status.lower() or "gap" in status.lower():
                self.pending = False
                self.message = status + ". Check the signal, then repeat this step."
                return
            if self.stage == "rest" and graph.get("emg_calibration", "").startswith(
                "Calibrate strong grip"
            ):
                self.stage, self.pending = "grip", False
                self.message = "Rest baseline saved. Press 3, then hold a firm grip for 3 seconds."
            elif self.stage == "grip":
                status = graph.get("emg_calibration", "")
                if status == "Grip calibrated":
                    self.stage, self.pending = "ready", False
                    self.message = "Grip calibrated. Return the open hand to marker 1, then press 4. Start control."
                elif (
                    "weak" in status.lower()
                    or "repeat" in status.lower()
                    or "clipping" in status.lower()
                ):
                    self.pending = False
                    self.message = status + ". Press 3 to repeat."

    @property
    def button(self):
        return {
            "align": "1. Align starting pose (hold still 2 s)",
            "imu": "Finishing IMU baseline...",
            "rest": "2. Calibrate relaxed EMG",
            "grip": "3. Calibrate firm grip (3 s)",
            "ready": "4. Start interactive control",
            "running": "Control active",
        }[self.stage]
