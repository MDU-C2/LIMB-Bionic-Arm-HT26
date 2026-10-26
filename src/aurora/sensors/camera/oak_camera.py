# Christian, Majd
"""OAK-D pipeline setup and camera defaults."""

from __future__ import annotations

import os
import tempfile
import time
from pathlib import Path

FRAME_SIZE = (640, 480)


DEPTH_SIZE = (640, 400)


VIDEO_FPS = 25.0


WINDOW = "AURORA OAK-D | arm and mug task"

FRAME_TIMESTAMP_REFERENCE = "host_frame_receipt_before_tracking"


def receive_frame(message):
    """Timestamp a received RGB frame before running tracking on it."""
    frame = message.getCvFrame()
    return frame, time.monotonic_ns()


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
