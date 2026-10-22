# Majd
"""Replay recorded BLE packets and camera observations without hardware."""

from __future__ import annotations

import base64
import json
import time
from dataclasses import replace
from pathlib import Path

if __package__:
    from .live_sensor_input import LiveSensorInput
else:
    from live_sensor_input import LiveSensorInput


class RecordedSensorInput(LiveSensorInput):
    def __init__(self, trial: Path, **kwargs):
        self.directory = trial.parent if trial.is_file() else trial
        self.manifest = json.loads((self.directory / "trial.json").read_text(encoding="utf-8"))
        self.replay_time = 1000.0
        kwargs.pop("clock", None)
        kwargs["side"] = self.manifest["side"]
        super().__init__(clock=lambda: self.replay_time, **kwargs)
        self.events = []
        self.index = 0
        self.elapsed = 0.0
        self.duration = float(self.manifest["duration_seconds"])
        self.ended = False
        self.playing = False
        self.capture = None
        self.capture_frame = -1

    def start(self) -> None:
        import cv2
        import numpy as np
        from ble_sensor_input import BleSensorReader

        self.cv2, self.np = cv2, np
        self.camera_status = "Saved camera observations"
        profile = self.manifest["calibration"]
        axes = profile.get("imu_twist_axes", {})
        self.reader = BleSensorReader(
            shoulder_twist_axis=axes.get("shoulder", "y"),
            wrist_twist_axis=axes.get("wrist", "y"),
            clock=self.clock,
        )
        self.reader.status, self.reader.error = "Recorded data", ""
        imu = self.reader.imu_estimator
        imu.tilt_offsets = dict(profile.get("imu_tilt_offsets", {}))
        imu.gyro_bias_z = dict(profile.get("imu_gyro_bias_z", {}))
        imu.elbow_zero = profile.get("imu_elbow_zero", 0.0)
        imu.active_roles = tuple(imu.tilt_offsets)
        imu.yaw_delta = dict(profile.get("imu_yaw_delta", dict.fromkeys(imu.active_roles, 0.0)))
        imu.camera_offsets = dict(profile.get("imu_camera_offsets", {}))
        imu.reference_angles = dict(profile.get("imu_reference_angles", {}))
        imu.filtered_accel = {
            key: tuple(value) for key, value in profile.get("imu_filtered_accel", {}).items()
        }
        imu.twist.shoulder_angle = profile.get("imu_shoulder_twist_deg", 0.0)
        imu.twist.wrist_angle = profile.get("imu_wrist_twist_deg", 0.0)
        imu.twist.bias = {
            key: tuple(value) for key, value in profile.get("imu_twist_bias_dps", {}).items()
        }
        imu.twist.dual_ready = len(imu.twist.bias) == 2
        self.reader._emg_period = profile.get("emg_period", 0.00025)
        emg = self.reader.emg_estimator
        emg.baseline = profile.get("emg_baseline_adc")
        emg.rest_envelope = profile.get("emg_rest", 0.0)
        emg.reference = profile.get("emg_reference")
        emg.strong_envelope = profile.get("emg_strong")
        emg.envelope = emg.rest_envelope
        emg._energy = emg.rest_envelope**2
        origin = self.manifest["started_host_monotonic_ns"]
        packets = self.directory / "ble/packets.jsonl"
        with packets.open(encoding="utf-8") as source:
            for line in source:
                packet = json.loads(line)
                stamp = packet.get("synced_host_monotonic_ns") or packet["host_monotonic_ns"]
                self.events.append(((stamp - origin) / 1e9, 0, packet))
        pose = json.loads((self.directory / "camera/pose.json").read_text(encoding="utf-8"))
        for observation in pose["observations"]:
            self.events.append(((observation["host_monotonic_ns"] - origin) / 1e9, 1, observation))
        self.events.sort(key=lambda event: event[:2])
        if not self.events:
            raise ValueError("This trial contains no recorded sensor or camera events")
        if self.show_monitor:
            self.capture = cv2.VideoCapture(str(self.directory / "camera/video.mp4"))
            from live_sensor_input import WINDOW

            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        self.wall_started = time.monotonic()
        first_camera = next((event[0] for event in self.events if event[1] == 1), 0.0)
        self.advance_to(max(0.0, first_camera))

    def play(self) -> None:
        self.wall_started = time.monotonic() - self.elapsed
        self.playing = True

    def pause(self) -> None:
        self.playing = False

    def start_background(self) -> None:
        """Replay decodes saved packets and does not run camera inference."""

    def advance_to(self, elapsed: float):
        if elapsed < self.elapsed:
            raise ValueError("Restart the replay before moving backward")
        elapsed = min(elapsed, self.duration)
        while self.index < len(self.events) and self.events[self.index][0] <= elapsed:
            stamp, kind, value = self.events[self.index]
            self.replay_time = 1000.0 + stamp
            if kind == 0:
                self.reader.notification(value["sensor"], base64.b64decode(value["data_base64"]))
            else:
                self._poll_sources()
                self.camera_observation = value
                self.accept_camera_observation(value, self.clock())
                self.frame_sequence += 1
            self.index += 1
        self.elapsed = elapsed
        self.replay_time = 1000.0 + elapsed
        self.ended = elapsed >= self.duration
        snapshot = self._poll_sources()
        return replace(
            snapshot, status=f"Replay {elapsed:.1f}/{self.duration:.1f}s | {snapshot.status}"
        )

    def poll(self):
        snapshot = self.advance_to(
            time.monotonic() - self.wall_started if self.playing else self.elapsed
        )
        if self.capture is not None:
            fps = self.capture.get(self.cv2.CAP_PROP_FPS) or 30
            desired = int(self.elapsed * fps)
            while self.capture_frame < desired:
                ok, frame = self.capture.read()
                self.capture_frame += 1
                if not ok:
                    break
                self.latest_frame = frame
        return self._render_monitor(snapshot) if self.show_monitor else snapshot

    def calibrate(self, reference_angles=None) -> None:
        # The saved rest and grip calibration belongs to this recording.
        self.reference_angles = reference_angles
        self.camera_origin = dict(self.raw_camera_angles)
        self.camera_smoother.reset()
        if reference_angles is not None:
            self.camera_angles = dict(reference_angles)
            self.reader.correct_angles(reference_angles, 1.0)

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()
        super().close()
