# Majd
"""Collect a bounded functional reference on the existing sensor connection."""

from __future__ import annotations

import hashlib
import json
import threading
import uuid
from pathlib import Path

import numpy as np

from aurora.json_utils import write_json
from aurora.sensors.arm_alignment import METHOD, AlignmentError, ImuSeries, fit_alignment


class ReferenceCapture:
    """Keep measured vectors and stereo points separate from estimated angles.

    Acquisition only appends small records. Fitting and persistence run once in
    a worker after all five movements. A cancelled fit cannot change live state.
    """

    def __init__(self, *, subject_id, side, connection_generation, mounting_session_id):
        self.context = dict(
            subject_id=subject_id,
            side=side,
            connection_generation=connection_generation,
            mounting_session_id=mounting_session_id,
        )
        self.stages = {}
        self.active = None
        self.error = ""
        self.result = None
        self.directory = None
        self._lock = threading.Lock()
        self._finished = threading.Event()
        self._worker = None

    def begin_step(self, cue, host_ns):
        with self._lock:
            if self.active is not None or self._worker is not None:
                raise ValueError("Finish the current reference step first")
            self.active = cue
            self.stages[cue] = dict(
                start_host_ns=host_ns, imus={"shoulder": [], "wrist": []}, camera=[]
            )

    def add_imus(self, sensors):
        with self._lock:
            if self.active is None:
                return
            stage = self.stages[self.active]
            for role, sensor in sensors.items():
                if sensor.get("gyro_saturated"):
                    self.error = f"{self.active}: gyro reached its wire limit; repeat more slowly"
                samples = stage["imus"][role]
                stamp = sensor.get("sample_time_us")
                if stamp is None:
                    self.error = "Timestamped IMUs are required; update the sensor firmware"
                    continue
                if samples and stamp <= samples[-1][0]:
                    continue
                if len(samples) >= 2000:
                    self.error = "Reference sample buffer full; repeat the step"
                    continue
                samples.append(
                    [stamp, sensor["host_monotonic_ns"]]
                    + [sensor[kind][axis] for kind in ("accel_g", "gyro_dps") for axis in "xyz"]
                )

    def add_camera(self, observation, *, host_ns, device_ns):
        with self._lock:
            if self.active is None or "keypoints_camera_m" not in observation:
                return
            samples = self.stages[self.active]["camera"]
            if samples and host_ns <= samples[-1]["host_monotonic_ns"]:
                return
            if len(samples) >= 600:
                self.error = "Reference camera buffer full; repeat the step"
                return
            samples.append(
                dict(
                    host_monotonic_ns=host_ns,
                    device_time_ns=device_ns,
                    keypoints_camera_m=observation["keypoints_camera_m"],
                    depth_time_difference_ns=observation.get("depth_time_difference_ns"),
                )
            )

    def end_step(self, host_ns):
        with self._lock:
            if self.active is not None:
                self.stages[self.active]["end_host_ns"] = host_ns
                self.active = None

    def finish(self, output: Path):
        with self._lock:
            if self.active is not None or self._worker is not None:
                raise ValueError("Finish each movement before fitting the reference")
            self._worker = threading.Thread(target=self._fit, args=(output,), daemon=True)
            self._worker.start()

    @property
    def finished(self):
        return self._finished.is_set()

    @property
    def started(self):
        return self._worker is not None

    def close(self, timeout=2.0):
        if self._worker is not None:
            self._worker.join(timeout)
            if self._worker.is_alive():
                raise TimeoutError("Reference save is still running; retry cleanup")

    def _fit(self, output):
        profile = {
            **self.context,
            "format_version": 1,
            "method": METHOD,
            "status": "failed",
            "accuracy_validated": False,
        }
        try:
            # Keep failed attempts too. These are normalized calibration inputs,
            # not replacement raw BLE/video recordings or participant cup trials.
            self.directory = Path(output) / uuid.uuid4().hex
            self.directory.mkdir(parents=True, exist_ok=False)
            source = {
                "format_version": 1,
                **self.context,
                "imu_units": ["g", "dps"],
                "imu_columns": [
                    "device_time_us",
                    "host_monotonic_ns",
                    "ax",
                    "ay",
                    "az",
                    "gx",
                    "gy",
                    "gz",
                ],
                "cue_assignment_clock": "host_receipt_monotonic_ns",
                "stages": self.stages,
                "error": self.error,
            }
            payload = json.dumps(source, allow_nan=False).encode("utf-8")
            (self.directory / "inputs.json").write_bytes(payload)
            profile["source_sha256"] = {"inputs.json": hashlib.sha256(payload).hexdigest()}
            profile["source_code_sha256"] = {
                name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                for name in ("reference_capture.py", "arm_alignment.py")
            }
            if self.error:
                raise AlignmentError(self.error)
            stages, camera = {}, {}
            for cue, stage in self.stages.items():
                settle = 1 if cue in ("STILL", "RETURN") else 0.25
                start = stage["start_host_ns"] + round(settle * 1e9)
                end = stage["end_host_ns"] - 200_000_000
                stages[cue] = {}
                for role, samples in stage["imus"].items():
                    rows = np.asarray(
                        [row for row in samples if start <= row[1] < end], dtype=float
                    ).reshape(-1, 8)
                    stages[cue][role] = ImuSeries(rows[:, 0] / 1e6, rows[:, 2:5], rows[:, 5:8])
                camera[cue] = [
                    row["keypoints_camera_m"]
                    for row in stage["camera"]
                    if start <= row["host_monotonic_ns"] < end
                    and row["depth_time_difference_ns"] is not None
                    and abs(row["depth_time_difference_ns"]) <= 20_000_000
                ]
            profile.update(fit_alignment(stages, camera, side=self.context["side"]))
        except Exception as error:
            profile.update(status="failed", message=str(error))
        try:
            if self.directory is not None:
                write_json(self.directory / "calibration.json", profile)
        except OSError as error:
            profile.update(status="failed", message=f"Cannot save calibration: {error}")
        finally:
            self.result = profile
            self._finished.set()
