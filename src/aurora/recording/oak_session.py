# Christian, Majd
"""Save OAK-D video, poses, and recording metadata."""

from __future__ import annotations

import contextlib
import json
import time

from aurora.json_utils import write_json
from aurora.recording.common import (
    create_session_directory,
    experiment_metadata,
    utc_now,
    write_metadata,
)
from aurora.sensors.camera.oak_camera import FRAME_SIZE, FRAME_TIMESTAMP_REFERENCE, VIDEO_FPS


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
        raw_video=False,
    ):
        self.cv2 = cv2
        self.subject = subject
        self.side = side
        self.depth_enabled = depth_enabled
        self.directory = directory
        self.metadata = metadata
        self.raw_video = raw_video
        self.frame_times = None
        self.depth_times = None
        self.depth_frames = 0
        self.source_frames = 0
        self.writer = None
        self.session = None
        self._finalized = False
        self._closing = False
        self.started = 0.0
        self.started_host_monotonic_ns = 0
        self.started_utc = ""
        self.video_frames = 0
        self.pose_frames: list[dict[str, object]] = []
        self.observations: list[dict[str, object]] = []

    @property
    def recording(self) -> bool:
        return self.writer is not None and not self._closing

    def start(self) -> None:
        if self.session is not None and not self._finalized:
            raise RuntimeError(
                "Finish closing the previous camera recording before starting another"
            )
        origin = (self.metadata or {}).get("trial_started_host_monotonic_ns", time.monotonic_ns())
        if type(origin) is not int or origin <= 0:
            raise ValueError("Camera recording needs a positive host monotonic start time")
        self.started_host_monotonic_ns = origin
        self.started = origin / 1e9
        self.started_utc = utc_now()
        self.video_frames = self.source_frames = self.depth_frames = 0
        self.pose_frames, self.observations = [], []
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
        self._finalized = False
        self._closing = False
        try:
            self.frame_times = (self.session / "frame_times.jsonl").open(
                "w", encoding="utf-8", buffering=1
            )
            if self.raw_video and self.depth_enabled:
                (self.session / "depth").mkdir()
                self.depth_times = (self.session / "depth_times.jsonl").open(
                    "w", encoding="utf-8", buffering=1
                )
        except Exception as error:
            self.metadata = {
                **(self.metadata or {}),
                "complete": False,
                "status": "incomplete",
                "start_error": str(error),
            }
            with contextlib.suppress(Exception):
                self.stop()
            raise
        print(f"REC started: {self.session}")

    def add_observation(self, sample, observation, *, host_monotonic_ns: int) -> None:
        if not self.recording:
            return
        if host_monotonic_ns < self.started_host_monotonic_ns:
            return
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

    def add_frame(
        self,
        frame,
        sample,
        observation,
        *,
        host_monotonic_ns: int,
        frame_sequence=None,
        device_frame_sequence=None,
        device_time_ns=None,
        save_observation=True,
    ) -> None:
        if not self.recording or host_monotonic_ns < self.started_host_monotonic_ns:
            return
        elapsed = (host_monotonic_ns - self.started_host_monotonic_ns) / 1e9
        if save_observation:
            self.add_observation(sample, observation, host_monotonic_ns=host_monotonic_ns)
        # VideoWriter uses fixed FPS; duplicate when tracking is slower so
        # playback duration stays close to wall-clock recording time.
        expected = min(
            max(self.video_frames + 1, round(elapsed * VIDEO_FPS)),
            self.video_frames + int(VIDEO_FPS),
        )
        first_video_frame = self.video_frames
        while self.video_frames < expected:
            self.writer.write(frame)
            self.video_frames += 1
        self.source_frames += 1
        if self.frame_times is not None:
            self.frame_times.write(
                json.dumps(
                    {
                        "source_frame_sequence": frame_sequence,
                        "device_frame_sequence": device_frame_sequence,
                        "device_time_ns": device_time_ns,
                        "host_monotonic_ns": host_monotonic_ns,
                        "time_s": round(elapsed, 6),
                        "video_frame_index": first_video_frame,
                        "encoded_copies": self.video_frames - first_video_frame,
                    }
                )
                + "\n"
            )

    def add_depth(self, frame) -> None:
        """Keep received depth maps lossless; pose processing must not replace them."""
        if (
            self._closing
            or self.depth_times is None
            or frame.frame_time_ns < self.started_host_monotonic_ns
        ):
            return
        if frame.depth_mm.ndim != 2 or frame.depth_mm.dtype != "uint16":
            raise ValueError("Depth recording requires a two-dimensional uint16 millimetre map")
        relative = f"depth/{self.depth_frames:06d}.png"
        encoded, png = self.cv2.imencode(
            ".png", frame.depth_mm, [self.cv2.IMWRITE_PNG_COMPRESSION, 1]
        )
        if not encoded:
            raise OSError(f"Could not save {relative}")
        # OpenCV's Windows file writer fails on paths containing letters such as Å.
        (self.session / relative).write_bytes(png.tobytes())
        self.depth_times.write(
            json.dumps(
                {
                    "file": relative,
                    "source_frame_sequence": frame.frame_sequence,
                    "host_monotonic_ns": frame.frame_time_ns,
                    "device_time_ns": frame.device_time_ns,
                    "time_s": (frame.frame_time_ns - self.started_host_monotonic_ns) / 1e9,
                    "shape": list(frame.depth_mm.shape),
                    "units": "mm",
                }
            )
            + "\n"
        )
        self.depth_frames += 1

    def stop(self) -> None:
        if self.session is None or self._finalized:
            return
        self._closing = True
        elapsed = time.monotonic() - self.started
        errors = []
        for name, method in (
            ("writer", "release"),
            ("frame_times", "close"),
            ("depth_times", "close"),
        ):
            resource = getattr(self, name)
            if resource is None:
                continue
            try:
                getattr(resource, method)()
            except Exception as error:
                errors.append(f"{name}: {error}")
            else:
                setattr(self, name, None)
        if errors:
            raise RuntimeError("; ".join(errors))
        pose_path = self.session / "pose.json"
        write_json(
            pose_path,
            {
                "format_version": 3,
                "subject_id": self.subject,
                "trial_id": (self.metadata or experiment_metadata()).get("trial_id", ""),
                "pose_frame_count": len(self.pose_frames),
                "observation_frame_count": len(self.observations),
                "video_frame_count": self.video_frames,
                "source_frame_count": self.source_frames,
                "depth_frame_count": self.depth_frames,
                "depth_times_file": "depth_times.jsonl"
                if self.raw_video and self.depth_enabled
                else None,
                "video_kind": "raw_rgb" if self.raw_video else "annotated_preview",
                "video_mirrored": not self.raw_video,
                "frame_times_file": "frame_times.jsonl",
                "coordinate_system": "rgb_pixels_stereo_depth",
                "timestamp_reference": FRAME_TIMESTAMP_REFERENCE,
                "side": self.side,
                "data": self.pose_frames,
                "observations": self.observations,
                "coordinate_note": (
                    "keypoints_2d are normalized raw-camera x/y; data x/y are raw "
                    "camera pixels. cup_detections bbox_xyxy are raw camera pixels. "
                    f"video.mp4 is {'unmirrored raw RGB' if self.raw_video else 'mirrored for the operator'}; depth_m "
                    "is RGB-aligned stereo depth"
                ),
                "angle_note": (
                    "Elbow flexion uses the shoulder, elbow and wrist. Shoulder angles "
                    "and the forearm-based rotation proxy require Oscar's six-point "
                    "trunk frame; mediapipe_world_estimate is monocular"
                ),
                "hand_note": (
                    "The hand nearest the selected pose wrist supplies 21 MediaPipe "
                    "landmarks and finger flexion proxies; these are model estimates"
                ),
            },
        )
        write_metadata(
            self.session,
            {
                **(experiment_metadata() if self.metadata is None else self.metadata),
                "source": "oak_pose",
                "format_version": 3,
                "subject_id": self.subject,
                "side": self.side,
                "started_utc": self.started_utc,
                "duration_seconds": round(elapsed, 3),
                "started_host_monotonic_ns": self.started_host_monotonic_ns,
                "clock_domain": "host_monotonic_ns",
                "timestamp_reference": FRAME_TIMESTAMP_REFERENCE,
                "pose_frame_count": len(self.pose_frames),
                "observation_frame_count": len(self.observations),
                "video_frame_count": self.video_frames,
                "source_frame_count": self.source_frames,
                "depth_frame_count": self.depth_frames,
                "depth_times_file": "depth_times.jsonl"
                if self.raw_video and self.depth_enabled
                else None,
                "video_kind": "raw_rgb" if self.raw_video else "annotated_preview",
                "frame_times_file": "frame_times.jsonl",
                "video_fps": VIDEO_FPS,
                "stereo_depth_enabled": self.depth_enabled,
                "video_mirrored": not self.raw_video,
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
        self._finalized = True
        print(f"REC saved: {self.session} ({len(self.pose_frames)} depth-backed pose frames)")
