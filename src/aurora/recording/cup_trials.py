# Majd
"""Save repeated cup transfers without reconnecting the sensors."""

from __future__ import annotations

import re
import time
from datetime import datetime
from itertools import chain
from pathlib import Path

from aurora.json_utils import write_json
from aurora.recording.common import safe_label, utc_now, verify_subject_files, write_metadata
from aurora.recording.oak_session import CameraSession
from aurora.recording.subject_calibration import export_aligned_imu

PACE_CONDITIONS = ("natural", "slow", "fast")
TASK_MARKERS = {
    "1": "hand starts here",
    "2": "grasp the cup",
    "3": "place and release the cup in the right circle",
    "4": "move the empty hand here",
}


def next_trial_number(subject_directory: Path, prefix="T") -> int:
    numbers = []
    if subject_directory.is_dir():
        for path in subject_directory.iterdir():
            match = re.match(rf"{re.escape(prefix)}(\d+)_", path.name)
            if path.is_dir() and match:
                numbers.append(int(match[1]))
    return max(numbers, default=0) + 1


class CupTrialSession:
    """Keep the camera and raw BLE files together under one subject and trial."""

    def __init__(self, output: Path, subject: str, side: str, reader, cv2, *, camera_metadata=None):
        if not subject.strip() or safe_label(subject) != subject:
            raise ValueError("Use a subject ID such as S01, with letters, numbers, - or _.")
        self.subject = subject
        self.side = side
        self.directory = output / safe_label(subject)
        verify_subject_files(
            chain(
                self.directory.glob("T*/trial.json"),
                self.directory.glob("references/R*/trial.json"),
            ),
            subject,
        )
        self.reader = reader
        self.cv2 = cv2
        self.camera = None
        self.path = None
        self.started_ns = 0
        self.metadata = {}
        self._stopped_health = None
        self.calibration = {}
        self.camera_metadata = dict(camera_metadata or {})

    @property
    def active(self) -> bool:
        return self.camera is not None

    @property
    def stopping(self) -> bool:
        return self.active and "recording_ended_host_monotonic_ns" in self.metadata

    @property
    def recording(self) -> bool:
        return self.active and not self.stopping and self.camera.recording

    def start(
        self,
        condition: str = "natural",
        *,
        protocol=None,
        mark_start=False,
        reference=False,
        reference_path=None,
        alignment_profile=None,
    ) -> None:
        if self.active:
            raise RuntimeError("Stop the current trial first")
        if condition not in PACE_CONDITIONS:
            raise ValueError("Choose natural, slow or fast pace")
        self._stopped_health = None
        data = self.reader.graph_data()
        packet = self.reader.latest()
        if not data["emg"] or len(packet["imus"]) != 2:
            raise RuntimeError("Wait for EMG and both IMUs before recording")
        parent = self.directory / "references" if reference else self.directory
        prefix = "R" if reference else "T"
        number = next_trial_number(parent, prefix)
        trial_id = f"{prefix}{number:03d}"
        self.path = parent / f"{trial_id}_{datetime.now():%Y%m%d_%H%M%S_%f}"
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
            "test_type": "subject_reference" if reference else "cup_transfer",
            "protocol_version": 3 if protocol else 1,
            "protocol": dict(protocol or {}),
            "pace_condition": None if reference else condition,
            "task_markers": {} if reference else dict(TASK_MARKERS),
            "subject_reference": reference_path,
            "sensor_to_arm_alignment": "fitted" if alignment_profile else "not_estimated",
            "alignment_profile": alignment_profile,
            "connection_generation": self.reader.connection_generation,
            "mounting_session_id": self.reader.mounting_session_id,
            "operator_events": [],
            "cue_events": [],
            "session_id": self.path.name,
            "batch_sources": "camera,ble",
            "camera_position": "in_front_of_subject",
            "pose_model_complexity": 1,
            "stereo_depth_enabled": False,
            "recording_mode": "raw_with_alignment" if alignment_profile else "raw_signals",
            "movement_labels": "not_reviewed",
            "task_completion_verified": False,
            **self.camera_metadata,
            "calibration_status": {
                "imu_bias_ready": bool(packet["imu_calibrated"]),
                "emg_rest_ready": bool(self.reader.emg_estimator.calibrated),
                "emg_grip_ready": data["emg_calibration_state"]["phase"] == "ready",
                "emg_message": data["emg_calibration"],
            },
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
            depth_enabled=bool(self.metadata["stereo_depth_enabled"]),
            directory=self.path / "camera",
            metadata=self.metadata,
            raw_video=True,
        )
        try:
            self.camera.start()
            self.reader.start_recording(self.path / "ble")
            if mark_start and not reference:
                self.mark(1)
        except Exception:
            self.stop("Could not start both recording sources", complete=False)
            raise

    def mark(self, marker: int, *, host_monotonic_ns=None, source="operator_button") -> None:
        """Save an explicit observation for advanced callers; normal trials use no markers."""
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
                "source": source,
            }
        )
        write_json(
            self.path / "trial.json", {**self.metadata, "complete": False, "status": "recording"}
        )

    def cue(self, name: str, *, host_monotonic_ns=None, **values) -> None:
        if not self.active:
            return
        now = time.monotonic_ns() if host_monotonic_ns is None else host_monotonic_ns
        self.metadata["cue_events"].append(
            {
                "cue": name,
                "host_monotonic_ns": now,
                "time_s": round((now - self.started_ns) / 1e9, 4),
                "source": "recorder_cue",
                **values,
            }
        )
        write_json(
            self.path / "trial.json", {**self.metadata, "complete": False, "status": "recording"}
        )

    def add_frame(self, live, *, save_observation=True) -> None:
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
                frame_sequence=getattr(live, "frame_sequence", None),
                device_frame_sequence=getattr(live, "device_frame_sequence", None),
                device_time_ns=getattr(live, "device_time_ns", None),
                save_observation=save_observation,
            )

    def add_tracking(self, frame) -> None:
        if self.active:
            for key in (
                "cup_detector",
                "cup_model_sha256",
                "cup_confidence_threshold",
                "hand_tracking_source",
            ):
                if key in frame.camera_observation:
                    self.metadata[key] = frame.camera_observation[key]
            self.camera.add_observation(
                frame.camera_sample,
                frame.camera_observation,
                host_monotonic_ns=frame.frame_time_ns,
            )

    def stop(self, reason: str = "Stopped by operator", *, complete: bool = True) -> Path | None:
        if self.camera is None:
            return None
        camera = self.camera
        stopped_ns = self.metadata.setdefault(
            "recording_ended_host_monotonic_ns", time.monotonic_ns()
        )
        errors = []
        try:
            if self._stopped_health is None:
                self._stopped_health = self.reader.stop_recording()
            health = dict(self._stopped_health)
        except Exception as error:
            errors.append(f"BLE recording: {error}")
            health = {"write_error": str(error)}
        profile = self.metadata.get("alignment_profile")
        if profile is not None and not errors and not self.metadata.get("aligned_imu"):
            try:
                if self.metadata.get("subject_reference_invalidated"):
                    raise ValueError("BLE reconnected: calibration invalidated; raw IMU data kept")
                self.metadata["aligned_imu"] = export_aligned_imu(
                    self.path / "ble/imu.csv",
                    self.path / "ble/imu_aligned.csv",
                    profile,
                    units=health.get("imu_units", ()),
                )
            except (OSError, ValueError, KeyError, IndexError) as error:
                health["alignment_export_error"] = str(error)
        counts = health.get("packet_counts", {})
        capture = self.metadata.get("capture_health", {})
        raw_losses = any(
            capture.get(key, 0)
            for key in (
                "raw_frame_queue_overruns",
                "depth_frame_queue_overruns",
                "camera_sequence_gaps",
                "depth_sequence_gaps",
                "camera_error",
            )
        ) or any(
            sensor.get("missing_packets", 0)
            or sensor.get("decode_errors", 0)
            or sensor.get("sample_time_gaps", 0)
            or sensor.get("gyro_saturation_samples", 0)
            for sensor in health.get("packet_health", {}).values()
        )
        complete = bool(
            complete
            and not errors
            and not self.metadata.get("capture_stop_failed")
            and not raw_losses
            and not health.get("raw_packet_queue_overruns")
            and not health.get("device_clock_resets")
            and not self.metadata.get("stream_interrupted")
            and not health.get("write_error")
            and not health.get("alignment_export_error")
            and not self.metadata.get("depth_interrupted")
            and (not self.metadata["stereo_depth_enabled"] or camera.depth_frames)
            and camera.video_frames
            and counts.get("emg")
            and counts.get("imu")
        )
        final = {
            **self.metadata,
            "complete": complete,
            "status": "complete" if complete else "incomplete",
            "stop_reason": reason,
            "duration_seconds": round((stopped_ns - self.started_ns) / 1e9, 3),
            "finished_utc": utc_now(),
            "recording_ended_host_monotonic_ns": stopped_ns,
            "movement_duration_seconds": round(
                (stopped_ns - self.metadata["movement_go_host_monotonic_ns"]) / 1e9, 3
            )
            if "movement_go_host_monotonic_ns" in self.metadata
            else None,
        }
        camera.metadata = final
        try:
            camera.stop()
        except Exception as error:
            errors.append(f"Camera recording: {error}")
        if errors:
            self.metadata["capture_stop_failed"] = True
            self.metadata["shutdown_error"] = "; ".join(errors)
            final.update(
                complete=False,
                status="incomplete",
                capture_stop_failed=True,
                shutdown_error=self.metadata["shutdown_error"],
            )
        try:
            write_metadata(self.path / "ble", {"source": "ble", **final, **health})
        except Exception as error:
            errors.append(f"BLE metadata: {error}")
            self.metadata["capture_stop_failed"] = True
            final.update(complete=False, status="incomplete", capture_stop_failed=True)
        manifest = {
            **final,
            "sources": {"ble": "ble", "camera": "camera"},
            "camera_observation_frame_count": len(camera.observations),
            "camera_pose_frame_count": len(camera.pose_frames),
            "camera_video_frame_count": camera.video_frames,
            "camera_source_frame_count": camera.source_frames,
            "camera_depth_frame_count": camera.depth_frames,
            "quality_review_required": True,
            **health,
        }
        write_json(self.path / "trial.json", manifest)
        if errors:
            raise RuntimeError("; ".join(errors))
        self.camera = None
        return self.path
