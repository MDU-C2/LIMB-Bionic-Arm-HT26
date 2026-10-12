# Christian, Majd
"""OAK-D pipeline setup and camera defaults."""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

FRAME_SIZE = (640, 480)


DEPTH_SIZE = (640, 400)


VIDEO_FPS = 25.0


WINDOW = "AURORA OAK-D | arm and mug task"


def configure_depthai_runtime() -> None:
    """Use stable, local DepthAI diagnostics on Windows lab machines."""
    os.environ.setdefault("DEPTHAI_CRASHDUMP", "0")
    os.environ.setdefault("DEPTHAI_TELEMETRY", "0")
    os.environ.setdefault(
        "DEPTHAI_CACHE_DIR",
        str(Path(tempfile.gettempdir()) / "aurora_depthai_cache"),
    )


def build_pipeline(dai, use_depth: bool = False):
    """Run RGB tracking; add stereo only when explicitly requested."""
    pipeline = dai.Pipeline()
    color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    color_output = color.requestOutput(
        FRAME_SIZE,
        type=dai.ImgFrame.Type.BGR888p,
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
    # DepthAI v3 aligns RVC2 stereo depth by linking the exact RGB stream used
    # for tracking.  The former v2-style setDepthAlign(socket) configuration
    # could drop both queues with X_LINK_ERROR on the project's OAK-D 2.1.
    color_output.link(stereo.inputAlignTo)
    return (
        pipeline,
        video_queue,
        stereo.depth.createOutputQueue(maxSize=2, blocking=False),
    )
