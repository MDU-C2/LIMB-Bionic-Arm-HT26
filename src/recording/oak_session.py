# Christian, Majd
"""Save OAK-D video, poses, and recording metadata."""

from __future__ import annotations

import contextlib
import json
import time

from common import (
    create_session_directory,
    experiment_metadata,
    utc_now,
    write_metadata,
)
from oak_camera import FRAME_SIZE, VIDEO_FPS


class CameraSession:
    """Start and stop files without interrupting the live camera."""

    def __init__(
        self,
        cv2,
        subject: str,
        side: str,
        depth_enabled: bool = False,
        *,
        directory=None,
        metadata=None,
    ):
        self.cv2 = cv2
        self.subject = subject
        self.side = side
        self.depth_enabled = depth_enabled
        self.directory = directory
        self.metadata = metadata
        self.writer = None
        self.session = None
        self.started = 0.0
        self.started_host_monotonic_ns = 0
        self.started_utc = ""
        self.video_frames = 0
        self.pose_frames: list[dict[str, object]] = []
        self.observations: list[dict[str, object]] = []

    @property
    def recording(self) -> bool:
        return self.writer is not None

    def start(self) -> None:
        if self.directory is None:
            self.session = create_session_directory("oak_pose", self.subject)
        else:
            self.session = self.directory
            self.session.mkdir(parents=True, exist_ok=False)
        writer = self.cv2.VideoWriter(
            str(self.session / "video.mp4"),
            self.cv2.VideoWriter_fourcc(*"mp4v"),
            VIDEO_FPS,
            FRAME_SIZE,
        )
        if not writer.isOpened():
            writer.release()
            with contextlib.suppress(OSError):
                self.session.rmdir()
            self.session = None
            raise RuntimeError("Could not open video.mp4 in the selected output folder")
        self.writer = writer
        self.started_host_monotonic_ns = time.monotonic_ns()
        self.started = self.started_host_monotonic_ns / 1e9
        self.started_utc = utc_now()
        self.video_frames = 0
        self.pose_frames = []
        self.observations = []
        print(f"REC started: {self.session}")

    def add_frame(self, frame, sample, observation, *, host_monotonic_ns=None) -> None:
        if not self.recording:
            return
        if host_monotonic_ns is None:
            host_monotonic_ns = time.monotonic_ns()
        elapsed = (host_monotonic_ns - self.started_host_monotonic_ns) / 1e9
        self.observations.append(
            {
                **observation,
                "time_s": round(elapsed, 4),
                "host_monotonic_ns": host_monotonic_ns,
            }
        )
        if sample is not None:
            self.pose_frames.append(
                {
                    **sample,
                    "time_s": round(elapsed, 4),
                    "host_monotonic_ns": host_monotonic_ns,
                }
            )
        # VideoWriter uses fixed FPS; duplicate when tracking is slower so
        # playback duration stays close to wall-clock recording time.
        expected = min(
            max(self.video_frames + 1, round(elapsed * VIDEO_FPS)),
            self.video_frames + int(VIDEO_FPS),
        )
        while self.video_frames < expected:
            self.writer.write(frame)
            self.video_frames += 1

    def stop(self) -> None:
        if not self.recording:
            return
        elapsed = time.monotonic() - self.started
        self.writer.release()
        self.writer = None
        pose_path = self.session / "pose.json"
        pose_path.write_text(
            json.dumps(
                {
                    "user_id": self.subject,
                    "sequence": len(self.pose_frames),
                    "side": self.side,
                    "data": self.pose_frames,
                    "observations": self.observations,
                    "coordinate_note": (
                        "keypoints_2d are normalized raw-camera x/y; data x/y are raw "
                        "camera pixels. cup_detections bbox_xyxy are raw camera pixels. "
                        "video.mp4 is mirrored for the operator; depth_m "
                        "is RGB-aligned stereo depth"
                    ),
                    "angle_note": (
                        "Elbow flexion, shoulder flexion, abduction and a forearm-based "
                        "rotation proxy follow Oscar's "
                        "six-point trunk frame; mediapipe_world_estimate is monocular"
                    ),
                    "hand_note": (
                        "The hand nearest the selected pose wrist supplies 21 MediaPipe "
                        "landmarks and finger flexion proxies; these are model estimates"
                    ),
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        write_metadata(
            self.session,
            {
                **(experiment_metadata() if self.metadata is None else self.metadata),
                "source": "oak_pose",
                "subject": self.subject,
                "side": self.side,
                "started_utc": self.started_utc,
                "duration_seconds": round(elapsed, 3),
                "started_host_monotonic_ns": self.started_host_monotonic_ns,
                "clock_domain": "host_monotonic_ns",
                "pose_frames": len(self.pose_frames),
                "observation_frames": len(self.observations),
                "video_frames": self.video_frames,
                "video_fps": VIDEO_FPS,
                "stereo_depth_enabled": self.depth_enabled,
                "video_mirrored": True,
                "hand_tracking": next(
                    (
                        item["hand_tracking_source"]
                        for item in self.observations
                        if "hand_tracking_source" in item
                    ),
                    "selected_arm_nearest_wrist_21_landmarks",
                ),
                "cup_detector": next(
                    (item["cup_detector"] for item in self.observations if "cup_detector" in item),
                    "unavailable_no_model",
                ),
            },
        )
        print(f"REC saved: {self.session} ({len(self.pose_frames)} depth-backed pose frames)")
