# Christian, Majd
"""OAK-D pipeline setup and camera defaults."""

from __future__ import annotations

import contextlib
import os
import tempfile
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

FRAME_SIZE = (640, 480)


DEPTH_SIZE = (640, 400)


VIDEO_FPS = 25.0


WINDOW = "AURORA OAK-D | arm and mug task"

FRAME_TIMESTAMP_REFERENCE = "host_frame_receipt_before_tracking"
DEPTH_MATCH_TOLERANCE_NS = 20_000_000


@dataclass(frozen=True)
class DepthFrame:
    """RGB-aligned millimetre map with receipt and OAK acquisition clocks."""

    frame_sequence: int
    frame_time_ns: int
    device_time_ns: int | None
    depth_mm: object


@dataclass(frozen=True)
class CameraFrame:
    """Keep raw/tracked pixels attached to their source receipt and device times."""

    frame_sequence: int
    frame_time_ns: int
    latest_frame: object
    camera_sample: object = None
    camera_observation: dict = field(default_factory=dict)
    device_frame_sequence: int | None = None
    device_time_ns: int | None = None
    received_at_s: float | None = None


def next_tracking_frame(frames, depths, last_sequence, *, use_depth, now_ns):
    """Prefer the newest paired RGB; bounded waiting never invents metric points."""
    for frame in reversed(frames):
        if frame.frame_sequence <= last_sequence:
            break
        depth = matching_depth(frame.device_time_ns, depths) if use_depth else None
        if depth is not None or not use_depth:
            return frame, depth
    for frame in reversed(frames):
        if frame.frame_sequence <= last_sequence:
            break
        if now_ns - frame.frame_time_ns >= 200_000_000:
            return frame, None
    return None, None


def device_timestamp_ns(message) -> int | None:
    """Return OAK device time, never substitute host time for a missing clock."""
    try:
        return round(message.getTimestampDevice().total_seconds() * 1e9)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def matching_depth(device_time_ns: int | None, candidates):
    """Choose a map within half a frame period on the same device clock."""
    if device_time_ns is None:
        return None
    valid = [frame for frame in candidates if frame.device_time_ns is not None]
    if not valid:
        return None
    frame = min(valid, key=lambda item: abs(item.device_time_ns - device_time_ns))
    return frame if abs(frame.device_time_ns - device_time_ns) <= DEPTH_MATCH_TOLERANCE_NS else None


def receive_frame(message):
    """Timestamp a received RGB frame before running tracking on it."""
    receipt_ns = time.monotonic_ns()
    return message.getCvFrame(), receipt_ns


def receive_depth(message):
    receipt_ns = time.monotonic_ns()
    return DepthFrame(
        message.getSequenceNum(), receipt_ns, device_timestamp_ns(message), message.getFrame()
    )


class PreviewFrames:
    """Latest-frame QoS for previews; raw recorders retain their own received frames."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.rgb, self.depth = deque(maxlen=12), deque(maxlen=12)
        self.received = self.tracked_sequence = 0

    def poll(self, video_queue, depth_queue, *, receive=receive_frame):
        if depth_queue is not None:
            message = depth_queue.tryGet()
            if message is not None:
                self.depth.append(receive_depth(message))
        message = video_queue.tryGet() if video_queue is not None else None
        if message is not None:
            image, stamp = receive(message)
            if image is not None:
                self.received += 1
                self.rgb.append(
                    CameraFrame(
                        self.received,
                        stamp,
                        image,
                        device_frame_sequence=getattr(message, "getSequenceNum", lambda: None)(),
                        device_time_ns=device_timestamp_ns(message),
                        received_at_s=self.clock(),
                    )
                )
        camera, depth = next_tracking_frame(
            self.rgb,
            self.depth,
            self.tracked_sequence,
            use_depth=depth_queue is not None,
            now_ns=time.monotonic_ns(),
        )
        if camera is not None:
            self.tracked_sequence = camera.frame_sequence
        return camera, depth


def configure_depthai_runtime() -> None:
    """Use stable, local DepthAI diagnostics on Windows lab machines."""
    os.environ.setdefault("DEPTHAI_CRASHDUMP", "0")
    os.environ.setdefault("DEPTHAI_TELEMETRY", "0")
    os.environ.setdefault(
        "DEPTHAI_CACHE_DIR",
        str(Path(tempfile.gettempdir()) / "aurora_depthai_cache"),
    )


def require_pose_model(mp, complexity: int) -> None:
    """Fail locally instead of letting MediaPipe download an optional model."""
    name = ("lite", "full", "heavy")[complexity]
    model = (
        Path(mp.__file__).resolve().parent
        / "modules"
        / "pose_landmark"
        / f"pose_landmark_{name}.tflite"
    )
    if not model.is_file():
        raise RuntimeError(
            f"The {name} pose model is not installed. Use the bundled full model "
            "(--pose-model 1), or repair the MediaPipe installation with Setup."
        )


def build_pipeline(dai, use_depth: bool = False):
    """Run RGB tracking; add stereo only when explicitly requested."""
    try:
        pipeline = dai.Pipeline()
    except RuntimeError as error:
        if "No available devices" in str(error):
            raise RuntimeError(
                "OAK-D camera unavailable. Connect its USB data cable, close other camera "
                "previews, wait a few seconds and try again."
            ) from error
        raise
    try:
        return _configure_pipeline(pipeline, dai, use_depth)
    except Exception:
        with contextlib.suppress(Exception):
            pipeline.stop()
        raise


def _configure_pipeline(pipeline, dai, use_depth):
    color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    color_output = color.requestOutput(
        FRAME_SIZE,
        # NV12 halves colour traffic. BGR + aligned depth can saturate USB 2.
        type=dai.ImgFrame.Type.NV12,
        fps=VIDEO_FPS,
        enableUndistortion=use_depth,
    )
    video_queue = color_output.createOutputQueue(maxSize=2, blocking=False)
    if not use_depth:
        return pipeline, video_queue, None
    mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    left_output = mono_left.requestOutput(DEPTH_SIZE, type=dai.ImgFrame.Type.GRAY8, fps=VIDEO_FPS)
    right_output = mono_right.requestOutput(DEPTH_SIZE, type=dai.ImgFrame.Type.GRAY8, fps=VIDEO_FPS)
    stereo = pipeline.create(dai.node.StereoDepth)
    # The default v3 preset enables expensive post-processing/subpixel work.
    # Keep the RVC2 OAK-D within its live RGB + stereo processing budget.
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_DENSITY)
    stereo.setSubpixel(False)
    stereo.setLeftRightCheck(True)
    left_output.link(stereo.left)
    right_output.link(stereo.right)
    # Align depth to the same RGB stream used for tracking.
    color_output.link(stereo.inputAlignTo)
    return (
        pipeline,
        video_queue,
        stereo.depth.createOutputQueue(maxSize=2, blocking=False),
    )
