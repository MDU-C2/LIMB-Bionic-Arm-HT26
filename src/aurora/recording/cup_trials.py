# Majd
"""Save repeated cup transfers without reconnecting the sensors."""

from __future__ import annotations

import re
import time
from datetime import datetime
from pathlib import Path

from aurora.json_utils import write_json
from aurora.recording.common import safe_label, utc_now, write_metadata
from aurora.recording.oak_session import CameraSession

PACE_CONDITIONS = ("natural", "slow", "fast")
TASK_MARKERS = {
    "1": "hand starts here",
    "2": "grasp the cup",
    "3": "place and release the cup in the right circle",
    "4": "move the empty hand here",
}


def next_trial_number(subject_directory: Path) -> int:
    numbers = []
    if subject_directory.is_dir():
        for path in subject_directory.iterdir():
            match = re.match(r"T(\d+)_", path.name)
            if path.is_dir() and match:
                numbers.append(int(match[1]))
    return max(numbers, default=0) + 1


class CupTrialSession:
    """Keep the camera and raw BLE files together under one subject and trial."""

    def __init__(self, output: Path, subject: str, side: str, reader, cv2):
        self.subject = subject
        self.side = side
        self.directory = output / safe_label(subject)
        self.reader = reader
        self.cv2 = cv2
        self.camera = None
        self.path = None
        self.started_ns = 0
        self.metadata = {}
        self.calibration = {}

    @property
    def active(self) -> bool:
        return self.camera is not None and self.camera.recording

    def start(self, condition: str = "natural") -> None:
        if self.active:
            raise RuntimeError("Stop the current trial first")
        if condition not in PACE_CONDITIONS:
            raise ValueError("Choose natural, slow or fast pace")
        data = self.reader.graph_data()
        if not data["emg"] or len(self.reader.latest()["imus"]) != 2:
            raise RuntimeError("Wait for EMG and both IMUs before recording")
        if data["emg_calibration"] != "Grip calibrated":
            raise RuntimeError("Calibrate relaxed muscle and strong grip before recording")
        number = next_trial_number(self.directory)
        trial_id = f"T{number:03d}"
        self.path = self.directory / f"{trial_id}_{datetime.now():%Y%m%d_%H%M%S_%f}"
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / "ble").mkdir()
        self.started_ns = time.monotonic_ns()
        self.calibration = {
            key: data[key]
            for key in (
                "emg_rest",
                "emg_reference",
                "emg_strong",
                "emg_threshold",
                "emg_period",
                "emg_calibration",
            )
        }
        self.calibration.update(self.reader.calibration_profile())
        self.metadata = {
            "format_version": 3,
            "subject_id": self.subject,
            "side": self.side,
            "trial_id": trial_id,
            "test_type": "cup_transfer",
            "protocol_version": 1,
            "pace_condition": condition,
            "task_markers": dict(TASK_MARKERS),
            "operator_events": [],
            "session_id": self.path.name,
            "batch_sources": "camera,ble",
            "camera_position": "in_front_of_subject",
            "pose_model_complexity": 1,
            "stereo_depth_enabled": False,
            "started_utc": utc_now(),
            "started_host_monotonic_ns": self.started_ns,
            "trial_started_host_monotonic_ns": self.started_ns,
            "calibration": self.calibration,
        }
        write_json(
            self.path / "trial.json", {**self.metadata, "complete": False, "status": "recording"}
        )
        self.camera = CameraSession(
            self.cv2,
            self.subject,
            self.side,
            directory=self.path / "camera",
            metadata=self.metadata,
        )
        try:
            self.camera.start()
            self.reader.start_recording(self.path / "ble")
            self.mark(1)
        except Exception:
            self.stop("Could not start both recording sources", complete=False)
            raise

    def mark(self, marker: int, *, host_monotonic_ns=None) -> None:
        if not self.active:
            raise ValueError("Start a trial before marking a task event")
        if str(marker) not in TASK_MARKERS:
            raise ValueError("Use marker 1, 2, 3 or 4")
        now = time.monotonic_ns()
        stamp = now if host_monotonic_ns is None else host_monotonic_ns
        if not isinstance(stamp, int) or not self.started_ns <= stamp <= now:
            raise ValueError("Marker time must be within the active trial")
        self.metadata["operator_events"].append(
            {
                "marker": marker,
                "host_monotonic_ns": stamp,
                "time_s": round((stamp - self.started_ns) / 1e9, 4),
                "source": "operator_button",
            }
        )
        write_json(
            self.path / "trial.json", {**self.metadata, "complete": False, "status": "recording"}
        )

    def add_frame(self, live) -> None:
        if self.active and live.frame_time_ns >= self.started_ns:
            for key in (
                "cup_detector",
                "cup_model_sha256",
                "cup_confidence_threshold",
                "hand_tracking_source",
            ):
                if key in live.camera_observation:
                    self.metadata[key] = live.camera_observation[key]
            self.camera.add_frame(
                live.latest_frame,
                live.camera_sample,
                live.camera_observation,
                host_monotonic_ns=live.frame_time_ns,
            )

    def stop(self, reason: str = "Stopped by operator", *, complete: bool = True) -> Path | None:
        if self.camera is None:
            return None
        camera, self.camera = self.camera, None
        health = self.reader.stop_recording()
        counts = health.get("packet_counts", {})
        complete = bool(
            complete
            and not health.get("write_error")
            and camera.video_frames
            and counts.get("emg")
            and counts.get("imu")
        )
        final = {
            **self.metadata,
            "complete": complete,
            "status": "complete" if complete else "incomplete",
            "stop_reason": reason,
            "duration_seconds": round((time.monotonic_ns() - self.started_ns) / 1e9, 3),
            "finished_utc": utc_now(),
        }
        camera.metadata = final
        camera.stop()
        write_metadata(self.path / "ble", {"source": "ble", **final, **health})
        manifest = {
            **final,
            "sources": {"ble": "ble", "camera": "camera"},
            "camera_observation_frame_count": len(camera.observations),
            "camera_pose_frame_count": len(camera.pose_frames),
            "camera_video_frame_count": camera.video_frames,
            **health,
        }
        write_json(self.path / "trial.json", manifest)
        return self.path
