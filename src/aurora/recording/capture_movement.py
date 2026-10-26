# Majd, Christian
"""Capture a camera movement profile with RGB pixels and measured stereo depth."""

from __future__ import annotations

import argparse
import json
import os
import re
import time
from contextlib import closing
from datetime import datetime
from pathlib import Path

from aurora.paths import RECORDINGS_DIRECTORY
from aurora.recording.common import safe_label, utc_now
from aurora.sensors.camera.hand_tracking import ArmHandTracker
from aurora.sensors.camera.oak_camera import (
    FRAME_TIMESTAMP_REFERENCE,
    build_pipeline,
    configure_depthai_runtime,
    receive_frame,
    require_pose_model,
)
from aurora.sensors.camera.oak_tracking import analyze_frame

DEFAULT_OUTPUT = RECORDINGS_DIRECTORY / "movement"


def arguments(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--subject-id", "--user-id", dest="subject_id", default=os.environ.get("AURORA_SUBJECT", "")
    )
    parser.add_argument(
        "--side", choices=("left", "right"), default=os.environ.get("AURORA_CAMERA_SIDE", "left")
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(os.environ["AURORA_OUTPUT_DIR"]) / "movement"
        if os.environ.get("AURORA_OUTPUT_DIR")
        else DEFAULT_OUTPUT,
    )
    args = parser.parse_args(argv)
    if not args.subject_id.strip() or safe_label(args.subject_id) != args.subject_id:
        parser.error("Use a subject ID such as S01, with letters, numbers, - or _.")
    return args


def next_trial_number(folder: Path) -> int:
    used = []
    for path in folder.glob("T*_*.json"):
        match = re.match(r"T(\d+)_", path.name)
        if match:
            used.append(int(match[1]))
    return max(used, default=0) + 1


def save_capture(folder: Path, subject_id: str, side: str, frames: list[dict]) -> Path:
    if not frames:
        raise ValueError("No depth-backed movement frames to save")
    folder.mkdir(parents=True, exist_ok=True)
    trial_id = f"T{next_trial_number(folder):03d}"
    path = folder / f"{trial_id}_{datetime.now():%Y%m%d_%H%M%S_%f}.json"
    payload = {
        "format_version": 3,
        "subject_id": subject_id,
        "trial_id": trial_id,
        "pose_frame_count": len(frames),
        "side": side,
        "captured_utc": utc_now(),
        "coordinate_system": "rgb_pixels_stereo_depth",
        "timestamp_reference": FRAME_TIMESTAMP_REFERENCE,
        "data": frames,
    }
    with path.open("x", encoding="utf-8") as output:
        json.dump(payload, output, indent=2)
    return path


def main(argv=None) -> None:
    args = arguments(argv)
    configure_depthai_runtime()
    import cv2
    import depthai as dai
    import mediapipe as mp
    import numpy as np

    require_pose_model(mp, 1)
    captured: list[dict] = []
    recording = False
    started_ns = 0
    pipeline, video_queue, depth_queue = build_pipeline(dai, use_depth=True)
    depth = None
    try:
        with (
            mp.solutions.pose.Pose(model_complexity=1, min_detection_confidence=0.5) as pose,
            closing(ArmHandTracker(mp, cv2)) as hands,
        ):
            pipeline.start()
            while pipeline.isRunning():
                depth_message = depth_queue.tryGet()
                if depth_message is not None:
                    depth = depth_message.getFrame()
                message = video_queue.tryGet()
                if message is None:
                    time.sleep(0.005)
                    continue
                frame, stamp = receive_frame(message)
                if frame is None:
                    continue
                image, _status, sample, observation = analyze_frame(
                    frame, depth, pose, mp, cv2, np, args.side, hands=hands
                )
                valid = sample is not None and observation.get("hand_visible", False)
                if recording and valid:
                    captured.append(
                        {**sample, "host_monotonic_ns": stamp, "time_s": (stamp - started_ns) / 1e9}
                    )
                status = (
                    f"REC {len(captured)} frames | SPACE: save" if recording else "SPACE: start"
                )
                if not valid:
                    status = "Show the selected arm and hand; stereo depth is required"
                cv2.putText(
                    image, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2
                )
                cv2.imshow("AURORA | Movement profile", image)
                key = cv2.waitKey(1) & 0xFF
                if key == 32:
                    if recording and captured:
                        path = save_capture(
                            args.output / args.subject_id, args.subject_id, args.side, captured
                        )
                        print(f"Saved {len(captured)} frames to {path}")
                        return
                    if not recording:
                        recording = True
                        started_ns = time.monotonic_ns()
                if (
                    key in (ord("q"), 27)
                    or cv2.getWindowProperty("AURORA | Movement profile", cv2.WND_PROP_VISIBLE) < 1
                ):
                    return
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
