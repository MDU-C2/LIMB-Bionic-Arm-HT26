# Christian, Majd
"""Record selected arm, hand, and finger motion from an OAK-D camera."""

from __future__ import annotations

import argparse
import math
import os
import time

from aurora.recording.common import announce_result, enable_ctrl_break_as_interrupt, env_float
from aurora.recording.oak_session import CameraSession as CameraSession
from aurora.sensors.camera.cup_detection import CupDetector
from aurora.sensors.camera.hand_tracking import ArmHandTracker
from aurora.sensors.camera.oak_camera import DEPTH_SIZE as DEPTH_SIZE
from aurora.sensors.camera.oak_camera import FRAME_SIZE as FRAME_SIZE
from aurora.sensors.camera.oak_camera import VIDEO_FPS as VIDEO_FPS
from aurora.sensors.camera.oak_camera import WINDOW as WINDOW
from aurora.sensors.camera.oak_camera import PreviewFrames, receive_frame, require_pose_model
from aurora.sensors.camera.oak_camera import build_pipeline as build_pipeline
from aurora.sensors.camera.oak_camera import configure_depthai_runtime as configure_depthai_runtime
from aurora.sensors.camera.oak_geometry import ANGLE_LABELS as ANGLE_LABELS
from aurora.sensors.camera.oak_geometry import FINGER_CHAINS as FINGER_CHAINS
from aurora.sensors.camera.oak_geometry import HAND_LANDMARK_NAMES as HAND_LANDMARK_NAMES
from aurora.sensors.camera.oak_geometry import PALM_CONNECTIONS as PALM_CONNECTIONS
from aurora.sensors.camera.oak_geometry import TIP_LABELS as TIP_LABELS
from aurora.sensors.camera.oak_geometry import arm_angles_deg as arm_angles_deg
from aurora.sensors.camera.oak_geometry import camera_point as camera_point
from aurora.sensors.camera.oak_geometry import camera_xyz as camera_xyz
from aurora.sensors.camera.oak_geometry import depth_at as depth_at
from aurora.sensors.camera.oak_geometry import display_pixel as display_pixel
from aurora.sensors.camera.oak_geometry import finger_angles_deg as finger_angles_deg
from aurora.sensors.camera.oak_geometry import landmark_ids as landmark_ids
from aurora.sensors.camera.oak_geometry import select_hand_near_wrist as select_hand_near_wrist
from aurora.sensors.camera.oak_tracking import add_hand_tracking as add_hand_tracking
from aurora.sensors.camera.oak_tracking import analyze_frame as analyze_frame


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--subject-id",
        "--subject",
        dest="subject",
        default=os.environ.get("AURORA_SUBJECT", "session"),
    )
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        default=os.environ.get("AURORA_CAMERA_SIDE", "left"),
        help="Subject's arm to highlight and save (default: left).",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Compatibility option; live preview is always shown first.",
    )
    parser.add_argument(
        "--auto-start",
        action="store_true",
        default=os.environ.get("AURORA_AUTO_START", "").strip() == "1",
        help="Start saving immediately after the camera and tracking models are ready.",
    )
    parser.add_argument(
        "--depth",
        action="store_true",
        help="Enable stereo depth for 3D playback points (experimental on this OAK-D).",
    )
    parser.add_argument(
        "--pose-model",
        type=int,
        choices=(0, 1, 2),
        default=1,
        help="MediaPipe pose complexity; 2 is closest to Oscar's heavy model.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=env_float("AURORA_DURATION_SECONDS", 0.0),
        help="Maximum seconds per recording; 0 means until stopped.",
    )
    return parser.parse_args()


def draw_window(frame, status: str, session: CameraSession, cv2, np, observation=None):
    """Render a clickable REC button below the camera image."""
    height, width = frame.shape[:2]
    canvas = np.zeros((height + 164, width, 3), dtype=np.uint8)
    canvas[:height] = frame
    canvas[height:] = (26, 31, 37)
    active = session.recording
    button = (width - 168, height + 116, width - 16, height + 156)
    cv2.rectangle(
        canvas,
        (button[0], button[1]),
        (button[2], button[3]),
        (40, 40, 210) if active else (35, 55, 205),
        -1,
    )
    cv2.putText(
        canvas,
        "STOP REC" if active else "START REC",
        (button[0] + 15, button[1] + 27),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        canvas, status[:78], (12, height + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (235, 235, 235), 1
    )
    angles = (observation or {}).get("angles_deg")
    if angles:
        parts = [
            f"{label}: {angles[name]:.0f}" if angles.get(name) is not None else f"{label}: --"
            for name, label in ANGLE_LABELS
        ]
        if (observation or {}).get("angle_source") == "mediapipe_world_estimate":
            parts[3] += " (estimate)"
        angle_lines = ("    |    ".join(parts[:2]), "    |    ".join(parts[2:]))
    else:
        angle_lines = ("Arm angles: frame both shoulders and hips", "")
    cv2.putText(
        canvas, angle_lines[0], (12, height + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (90, 200, 255), 1
    )
    cv2.putText(
        canvas, angle_lines[1], (12, height + 73), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (90, 200, 255), 1
    )
    curls = (observation or {}).get("finger_curl_deg")
    if curls:
        finger_line = "Finger curl T/I/M/R/P: " + "/".join(
            f"{curls[name]:.0f}" for name in ("thumb", "index", "middle", "ring", "pinky")
        )
    else:
        finger_line = "Finger curl: show the selected hand"
    cv2.putText(
        canvas, finger_line, (12, height + 97), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (80, 220, 110), 1
    )
    cv2.putText(
        canvas,
        ("Cup detected" if (observation or {}).get("cup_visible") else "Cup not detected")
        if (observation or {}).get("cup_detector_status") == "ready"
        else "Cup detector unavailable",
        (12, height + 121),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.47,
        (160, 180, 200),
        1,
    )
    cv2.putText(
        canvas,
        "R: record / stop    Q: close",
        (12, height + 149),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (190, 190, 190),
        1,
    )
    if active:
        cv2.circle(canvas, (width - 108, 26), 8, (25, 25, 240), -1)
        cv2.putText(
            canvas,
            f"REC {time.monotonic() - session.started:.1f}s",
            (width - 94, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (25, 25, 240),
            2,
        )
    return canvas, button


def main() -> int:
    enable_ctrl_break_as_interrupt(stdin_stop=True)
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        print("Duration must be finite and non-negative.")
        return 2
    configure_depthai_runtime()
    try:
        import cv2
        import depthai as dai
        import mediapipe as mp
        import numpy as np
    except ImportError as error:
        print(f"Camera or pose dependency missing: {error.name}")
        return 2

    try:
        pipeline, video_queue, depth_queue = build_pipeline(dai, args.depth)
        pipeline.start()
    except Exception as error:
        print(f"Could not start the OAK-D camera: {error}")
        return 1

    intrinsics = None
    if args.depth:
        try:
            calibration = pipeline.getDefaultDevice().readCalibration()
            intrinsics = calibration.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, *FRAME_SIZE)
        except Exception as error:
            print(f"Camera calibration unavailable; stereo angles disabled: {error}")

    session = CameraSession(cv2, args.subject, args.side, args.depth, on_saved=announce_result)
    controls = {"toggle": False, "button": None}

    def on_mouse(event, x, y, _flags, _userdata):
        rect = controls["button"]
        if (
            event == cv2.EVENT_LBUTTONUP
            and rect
            and rect[0] <= x <= rect[2]
            and rect[1] <= y <= rect[3]
        ):
            controls["toggle"] = True

    frames = PreviewFrames()
    pose = None
    hands = None
    cup_detector = None
    try:
        require_pose_model(mp, args.pose_model)
        pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=args.pose_model,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        hands = ArmHandTracker(mp, cv2)
        cup_detector = CupDetector(cv2, np)
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW, on_mouse)
        print(f"Live OAK-D tracking: subject's {args.side} arm, hand and fingers.")
        print(
            "Camera video is mirrored for the operator; tracking and saved "
            "coordinates use the raw frame."
        )
        print(f"Cup detector: {cup_detector.name} ({cup_detector.status})")
        if not args.depth:
            print("Stereo depth is off; use --depth to attempt depth-backed playback points.")
        if getattr(args, "auto_start", False):
            session.start()
        while pipeline.isRunning():
            camera, matched = frames.poll(video_queue, depth_queue, receive=receive_frame)
            if camera is not None:
                frame, frame_time_ns = camera.latest_frame, camera.frame_time_ns
                latest_depth = matched.depth_mm if matched is not None else None
                image, status, sample, observation = analyze_frame(
                    frame,
                    latest_depth,
                    pose,
                    mp,
                    cv2,
                    np,
                    args.side,
                    intrinsics,
                    hands,
                    cup_detector=cup_detector,
                )
                session.add_frame(
                    image,
                    sample,
                    observation,
                    host_monotonic_ns=frame_time_ns,
                    frame_sequence=camera.frame_sequence,
                    device_frame_sequence=camera.device_frame_sequence,
                    device_time_ns=camera.device_time_ns,
                )
                canvas, controls["button"] = draw_window(
                    image, status, session, cv2, np, observation
                )
                cv2.imshow(WINDOW, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if (
                hasattr(cv2, "getWindowProperty")
                and cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
            ):
                break
            if key == ord("r") or controls["toggle"]:
                controls["toggle"] = False
                try:
                    session.stop() if session.recording else session.start()
                except (OSError, RuntimeError) as error:
                    print(f"Recording error: {error}")
            if (
                session.recording
                and args.duration
                and time.monotonic() - session.started >= args.duration
            ):
                session.stop()
    except KeyboardInterrupt:
        pass
    finally:
        session.stop()
        if pose is not None:
            pose.close()
        if hands is not None:
            hands.close()
        if cup_detector is not None:
            cup_detector.close()
        cv2.destroyAllWindows()
        stop = getattr(pipeline, "stop", None)
        if callable(stop):
            stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
