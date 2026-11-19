"""Deterministic teaching signals passed through the normal sensor processing core."""

from __future__ import annotations

import csv
import math
import struct
from collections import deque
from datetime import datetime
from pathlib import Path
from uuid import uuid4

import numpy as np

from aurora.json_utils import write_json
from aurora.recording.cup_protocol import REFERENCE_STEPS, CupReferenceGuide, CupTrialGuide
from aurora.sensors.arm_alignment import ImuSeries, fit_alignment
from aurora.sensors.ble_input import BleSensorReader
from aurora.sensors.emg_packets import HIGH_RATE_EMG_HEADER
from aurora.sensors.imu_packets import HIGH_RATE_IMU_PACKET
from aurora.sensors.live_input import LiveSensorInput

STEP_S = 0.01
EMG_RATE = 4000
WINDOW_S = 20
MOUNTINGS = ("Along segment", "Quarter turn", "Half turn", "Tilted")
LESSONS = {key: instruction for key, _duration, instruction in REFERENCE_STEPS}
LESSONS.update(
    MANUAL="Move the sliders; sensor rates follow the changing pose.",
    CUP="Rehearse the four markers. This is an illustrative motion, not a measured cup trajectory.",
)
FAULTS = (
    "None",
    "Forearm missing",
    "Camera missing",
    "50 Hz interference",
    "ADC clipping",
    "Gyro bias",
)


def rotation(axis, angle_deg):
    """Proper active rotation; columns express sensor axes in the segment frame."""
    angle = math.radians(angle_deg)
    c, s = math.cos(angle), math.sin(angle)
    if axis == "x":
        return np.array(((1, 0, 0), (0, c, -s), (0, s, c)))
    if axis == "y":
        return np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))
    return np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))


def mounting(name):
    return {
        "Along segment": np.eye(3),
        "Quarter turn": rotation("z", 90),
        "Half turn": rotation("y", 180),
        "Tilted": rotation("z", 35) @ rotation("y", 55),
    }[name]


def orientations(angles, side):
    """Teaching frame +Y is up. Segment Y is distal; X is positive flexion."""
    shoulder, elbow, turn = angles
    upper = np.diag((-1.0, -1.0, 1.0)) @ rotation("x", shoulder)
    forearm = upper @ rotation("x", elbow) @ rotation("y", turn if side == "left" else -turn)
    return {"shoulder": upper, "wrist": forearm}


def body_rate(previous, current):
    """Angular velocity in the current segment frame, in degrees/s."""
    delta = previous.T @ current
    vector = (
        np.array((delta[2, 1] - delta[1, 2], delta[0, 2] - delta[2, 0], delta[1, 0] - delta[0, 1]))
        / 2
    )
    sine = float(np.linalg.norm(vector))
    if sine < 1e-12:
        return np.zeros(3)
    angle = math.atan2(sine, (np.trace(delta) - 1) / 2)
    return vector * math.degrees(angle) / (sine * STEP_S)


class SensorLesson:
    """One virtual source. It opens no BLE, serial, camera or model connection."""

    def __init__(self, side="left", seed=42):
        self.side = side
        self.random = np.random.default_rng(seed)
        self.t = 0.0
        self.index = 0
        self.settings = dict(
            lesson="BEND",
            shoulder=0.0,
            elbow=90.0,
            turn=0.0,
            effort=0.0,
            upper_mount=MOUNTINGS[0],
            forearm_mount=MOUNTINGS[0],
            noise=False,
            fault="None",
        )
        self.angles = np.array((0.0, 90.0, 0.0))
        self.previous = orientations(self.angles, side)
        self.history = deque(maxlen=round(WINDOW_S / STEP_S))
        self.live = LiveSensorInput(side=side, show_monitor=False, clock=lambda: 1000 + self.t)
        self.reader = BleSensorReader(side=side, clock=self.live.clock)
        self.reader.status, self.reader.error = "Synthetic input", ""
        self.live.camera_status = "Generated camera reference; no camera connected"
        self.live.reader = self.reader
        self.guide = None
        self.capture = {}
        self.camera = {}
        self.fit_message = (
            "Mounting not fitted. Raw signals are available; estimates can be relative."
        )
        self.latest = None
        self._phase = None
        self.origin = 0.0

    def configure(self, values):
        for key, choices in (
            ("lesson", LESSONS),
            ("upper_mount", MOUNTINGS),
            ("forearm_mount", MOUNTINGS),
            ("fault", FAULTS),
        ):
            if key in values and values[key] not in choices:
                raise ValueError(f"Unknown {key}")
        for key, low, high in (
            ("shoulder", 0, 75),
            ("elbow", 0, 140),
            ("turn", -90, 90),
            ("effort", 0, 1),
        ):
            if key in values and (not math.isfinite(values[key]) or not low <= values[key] <= high):
                raise ValueError(f"{key} must be finite and between {low} and {high}")
        changed_mount = any(
            values.get(key, self.settings[key]) != self.settings[key]
            for key in ("upper_mount", "forearm_mount")
        )
        if changed_mount:
            self.live.invalidate_calibration()
            self.guide = None
            self.capture, self.camera = {}, {}
            self.fit_message = (
                "Sensor moved: repeat mounting rehearsal. Real calibration is unaffected."
            )
        if values.get("lesson", self.settings["lesson"]) != self.settings["lesson"]:
            self.origin = self.t
            self.guide = None
        self.settings.update(values)

    def start_preparation(self):
        self.guide = CupReferenceGuide(clock=lambda: round(self.t * 1e9))
        self.capture, self.camera = {}, {}
        self._phase = None
        self.live.invalidate_calibration()
        self.fit_message = "Virtual reference: press Start step, then inspect each movement."

    def start_step(self):
        if isinstance(self.guide, CupReferenceGuide):
            self.guide.start_step()
        else:
            raise ValueError("Start a mounting rehearsal before advancing preparation")

    def start_trial(self):
        self.guide = CupTrialGuide(clock=lambda: round(self.t * 1e9))
        self.guide.begin("natural", practice=True)
        self.origin = self.t

    def _movement(self):
        key = self.settings["lesson"]
        seconds = self.t - self.origin
        if isinstance(self.guide, CupReferenceGuide):
            if self.guide.active:
                key = self.guide.steps[self.guide.step][0]
                seconds = (self.t * 1e9 - self.guide.phase_started_ns) / 1e9
            if not self.guide.active or self.guide.phase != "reference":
                key = "STILL"
        elif isinstance(self.guide, CupTrialGuide):
            key = "CUP" if self.guide.phase in ("movement", "practice") else "STILL"
            seconds = (self.t * 1e9 - (self.guide.go_ns or self.t * 1e9)) / 1e9
        wave = (1 - math.cos(2 * math.pi * max(0, seconds) / 5)) / 2
        target = (0, 90, 0)
        effort = self.settings["effort"]
        if key == "RAISE":
            target = (45 * wave, 90, 0)
        elif key == "BEND":
            target = (0, 90 + 45 * wave, 0)
        elif key == "TURN":
            target = (0, 90, 65 * math.sin(2 * math.pi * max(0, seconds) / 4))
        elif key == "MANUAL":
            target = tuple(self.settings[name] for name in ("shoulder", "elbow", "turn"))
            # Sliders use the published signed angles; the right-arm teaching
            # pose uses the opposite turn parameter to mirror palm-down cues.
            if self.side == "right":
                target = (*target[:2], -target[2])
        elif key == "GRIP":
            effort = 0.8
        elif key in ("RELAX", "RELEASE"):
            effort = 0.0
        elif key == "CUP":
            target = (15 + 20 * wave, 90 - 30 * wave, 0)
            effort = 0.7 if 0.2 < (seconds % 8) / 8 < 0.75 else 0.0
        return key, np.asarray(target, dtype=float), effort

    def advance(self):
        self.index += 1
        self.t = self.index * STEP_S
        old_step = self.guide.step if isinstance(self.guide, CupReferenceGuide) else None
        if self.guide is not None:
            self.guide.tick()
            if (
                isinstance(self.guide, CupTrialGuide)
                and self.guide.active
                and self.guide.window_finished
            ):
                self.guide.finish()
        if old_step == 4 and self.guide.step != 4:
            self.fit_reference()
        phase = (
            (self.guide.step, self.guide.phase)
            if isinstance(self.guide, CupReferenceGuide)
            else None
        )
        if phase != self._phase and phase is not None and phase[1] == "reference":
            key = self.guide.steps[phase[0]][0]
            if key == "RELAX":
                self.live.calibrate_emg()
            elif key == "GRIP":
                self.live.calibrate_emg_strength()
        self._phase = phase
        key, target, effort = self._movement()
        self.angles += np.clip(target - self.angles, -60 * STEP_S, 60 * STEP_S)
        matrices = orientations(self.angles, self.side)
        raw = {}
        mounts = {
            "shoulder": mounting(self.settings["upper_mount"]),
            "wrist": mounting(self.settings["forearm_mount"]),
        }
        for role, orientation in matrices.items():
            accel = mounts[role].T @ (orientation.T @ np.array((0, 1, 0)))
            gyro = mounts[role].T @ body_rate(self.previous[role], orientation)
            if self.settings["noise"]:
                accel += self.random.normal(0, 0.008, 3)
                gyro += self.random.normal(0, 0.25, 3)
            if self.settings["fault"] == "Gyro bias":
                gyro += (0.4, 3.5, -0.3)
            raw[role] = (accel, gyro)
        self.previous = matrices
        mask = 1 if self.settings["fault"] == "Forearm missing" else 3
        stamp = 1_000_000 + round(self.t * 1e6)
        wire = [
            max(-32768, min(32767, round(v * scale)))
            for role in ("shoulder", "wrist")
            for vector, scale in zip(raw[role], (1000, 100), strict=True)
            for v in vector
        ]
        imu_payload = HIGH_RATE_IMU_PACKET.pack(0x100 | mask, self.index, stamp, stamp, *wire)
        self.reader.notification("imu", imu_payload)
        times = self.t - STEP_S + np.arange(40) / EMG_RATE
        amplitude = 2 + 220 * effort
        if self.settings["fault"] == "ADC clipping":
            amplitude *= 15
        values = 2048 + self.random.normal(0, amplitude, 40)
        if self.settings["fault"] == "50 Hz interference":
            values += 180 * np.sin(2 * math.pi * 50 * times)
        samples = np.clip(np.rint(values), 0, 4095).astype(int).tolist()
        first_us = stamp - 10_000
        emg_payload = HIGH_RATE_EMG_HEADER.pack(0x100, self.index, first_us, 250, 40, 0)
        emg_payload += struct.pack("<40H", *samples)
        self.reader.notification("emg", emg_payload)
        truth = dict(
            zip(
                ("shoulder_flexion", "elbow_flexion", "wrist_rotation"),
                self.angles.tolist(),
                strict=True,
            )
        )
        if self.side == "right":
            truth["wrist_rotation"] *= -1
        truth["shoulder_abduction"] = truth["shoulder_rotation_proxy"] = 0.0
        if self.index % 3 == 0 and self.settings["fault"] != "Camera missing":
            self.live.accept_camera_observation(
                {"angles_deg": truth, "reference_visible": True}, self.live.clock()
            )
        snapshot = self.live._poll_sources()
        left = np.array((0.2, 0, -1.6))
        right = np.array((-0.2, 0, -1.6))
        shoulder = left if self.side == "left" else right
        elbow = shoulder + 0.3 * matrices["shoulder"][:, 1]
        wrist = elbow + 0.26 * matrices["wrist"][:, 1]
        points = {
            "left_shoulder": left,
            "right_shoulder": right,
            f"{self.side}_elbow": elbow,
            f"{self.side}_wrist": wrist,
        }
        if (
            phase is not None
            and phase[1] == "reference"
            and key in ("STILL", "RAISE", "BEND", "TURN", "RETURN")
        ):
            stage = self.capture.setdefault(key, {role: [] for role in raw})
            for role in raw:
                sensor = snapshot.sensors.get(role)
                if sensor is None or sensor.get("connected") is not True:
                    continue
                stage[role].append(
                    (
                        self.t,
                        np.array([sensor["accel_g"][axis] for axis in "xyz"]),
                        np.array([sensor["gyro_dps"][axis] for axis in "xyz"]),
                    )
                )
            if self.index % 10 == 0 and self.settings["fault"] != "Camera missing":
                self.camera.setdefault(key, []).append(points)
        self.latest = dict(
            time_s=self.t,
            movement=key,
            side=self.side,
            true_angles=truth,
            angles=snapshot.angles or {},
            relative=snapshot.relative_angles or {},
            targets=snapshot.robot_targets or {},
            status=snapshot.status,
            raw={
                role: [
                    value / (1000 if axis < 3 else 100)
                    for axis, value in enumerate(wire[i * 6 : i * 6 + 6])
                ]
                for i, role in enumerate(("shoulder", "wrist"))
            },
            received_roles=[
                role for role, sensor in snapshot.sensors.items() if sensor.get("connected") is True
            ],
            boards={role: (matrices[role] @ mounts[role]).tolist() for role in matrices},
            points={name: value.tolist() for name, value in points.items()},
            emg=samples,
            emg_first_us=first_us,
            emg_rms=self.reader.emg_estimator.envelope,
            activation=snapshot.emg_activation,
            grip=snapshot.hand_curl,
            emg_state=self.reader.emg_estimator.calibration_state,
            calibration_valid=snapshot.calibration_valid,
            guide=self.guide.view() if self.guide is not None else {},
            fit=self.fit_message,
            fault=self.settings["fault"],
            packets=dict(self.reader.decoder.counts),
        )
        self.history.append(self.latest)
        return self.latest

    def fit_reference(self):
        try:
            stages = {
                key: {
                    role: ImuSeries(
                        np.array([r[0] for r in rows]),
                        np.array([r[1] for r in rows]),
                        np.array([r[2] for r in rows]),
                    )
                    for role, rows in roles.items()
                }
                for key, roles in self.capture.items()
            }
            profile = fit_alignment(stages, self.camera, side=self.side)
            profile.update(
                source="synthetic_teaching_model",
                subject_id="synthetic",
                connection_generation=self.reader.connection_generation,
                mounting_session_id=self.reader.mounting_session_id,
            )
            self.reader.apply_alignment(profile, subject_id="synthetic", side=self.side)
            self.live.calibrate()
            self.fit_message = (
                "Model mounting fit passed. This does not calibrate physical sensors."
            )
        except (ValueError, KeyError) as error:
            self.fit_message = f"Mounting rehearsal needs a repeat: {error}"

    def export(self, root: Path):
        """Save only the bounded visible history, explicitly separate from participant trials."""
        if not self.history:
            raise ValueError("Run the lesson before saving a signal window")
        path = root / f"synthetic_{datetime.now():%Y%m%d_%H%M%S}_{uuid4().hex[:8]}"
        path.mkdir(parents=True, exist_ok=False)
        with (path / "signals.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                (
                    "time_s",
                    "role",
                    "ax_g",
                    "ay_g",
                    "az_g",
                    "gx_dps",
                    "gy_dps",
                    "gz_dps",
                    "movement",
                    "received",
                )
            )
            for row in self.history:
                for role, values in row["raw"].items():
                    writer.writerow(
                        (
                            row["time_s"],
                            role,
                            *values,
                            row["movement"],
                            role in row["received_roles"],
                        )
                    )
        with (path / "emg.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("device_time", "channel", "value"))
            for row in self.history:
                writer.writerows(
                    (row["emg_first_us"] + i * 250, 0, value) for i, value in enumerate(row["emg"])
                )
        angle_rows = [
            dict(
                time_s=row["time_s"],
                movement=row["movement"],
                phase=row["guide"].get("phase", "explore"),
                **{
                    f"{group}_{name}": value
                    for group in ("true_angles", "angles", "relative", "targets")
                    for name, value in row[group].items()
                },
            )
            for row in self.history
        ]
        with (path / "angles.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=list(dict.fromkeys(key for row in angle_rows for key in row))
            )
            writer.writeheader()
            writer.writerows(angle_rows)
        write_json(
            path / "lesson.json",
            dict(
                format_version=1,
                source="synthetic_teaching_model",
                participant_data=False,
                accuracy_validated=False,
                side=self.side,
                window_seconds=self.history[-1]["time_s"] - self.history[0]["time_s"] + STEP_S,
                settings=self.settings,
                calibration=self.reader.calibration_profile(),
                clock=dict(
                    domain="synthetic_device_us",
                    origin_us=1_000_000,
                    imu_period_us=10_000,
                    emg_period_us=250,
                ),
                window_start_s=self.history[0]["time_s"] - STEP_S,
                window_end_s=self.history[-1]["time_s"],
                frame="synthetic Cartesian frame: +Y up, segment Y distal, X positive flexion",
                latest=self.latest,
                note="Gravity and rotation model; no translation acceleration, electrodes or real radio. Saves at most the last 20 simulated seconds.",
            ),
        )
        return path

    def close(self):
        self.live.close()
