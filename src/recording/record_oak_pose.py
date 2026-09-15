"""Record OAK-D arm and hand landmarks for playback and movement AI."""

from __future__ import annotations

import argparse
import json
import os
import time

from common import create_session_directory, env_float, utc_now, write_metadata


FRAME_SIZE = (640, 480)
DEPTH_SIZE = (640, 400)


def parse_args() -> argparse.Namespace:
    """Read GUI defaults and optional command-line overrides."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subject", default=os.environ.get("AURORA_SUBJECT", "session"))
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--duration",
        type=float,
        default=env_float("AURORA_DURATION_SECONDS", 0.0),
        help="Seconds to record; 0 records until stopped or Q is pressed.",
    )
    return parser.parse_args()


def depth_at(depth_frame, x: int, y: int, width: int, height: int, np) -> float | None:
    """Return median depth in metres around one RGB point."""
    depth_height, depth_width = depth_frame.shape
    depth_x = int(x * depth_width / width)
    depth_y = int(y * depth_height / height)
    radius = 6
    region = depth_frame[
        max(0, depth_y - radius) : min(depth_height, depth_y + radius + 1),
        max(0, depth_x - radius) : min(depth_width, depth_x + radius + 1),
    ]
    valid = region[(region > 200) & (region < 3000)]
    return None if valid.size == 0 else float(np.median(valid)) / 1000.0


def camera_point(landmark, depth_frame, width: int, height: int, np) -> list[float] | None:
    """Convert a MediaPipe landmark to the repository camera-point format."""
    x = int(landmark.x * width)
    y = int(landmark.y * height)
    depth = depth_at(depth_frame, x, y, width, height, np)
    if depth is None:
        return None
    return [float(x), float(y), depth]


def nearest_hand(results, wrist_xy: tuple[int, int]):
    """Choose the detected hand whose wrist is closest to the arm wrist."""
    if not results.multi_hand_landmarks:
        return None
    width, height = FRAME_SIZE
    wrist_x, wrist_y = wrist_xy
    return min(
        results.multi_hand_landmarks,
        key=lambda hand: (
            hand.landmark[0].x * width - wrist_x
        ) ** 2
        + (hand.landmark[0].y * height - wrist_y) ** 2,
    )


def build_pipeline(dai):
    """Create the DepthAI 3 color and stereo-depth pipeline used by LIMB25."""
    pipeline = dai.Pipeline()
    color = pipeline.create(dai.node.Camera).build()
    color.setSensorType(dai.CameraSensorType.COLOR)
    color_output = color.requestOutput(FRAME_SIZE, type=dai.ImgFrame.Type.BGR888p)

    mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    left_output = mono_left.requestOutput(DEPTH_SIZE, type=dai.ImgFrame.Type.GRAY8)
    right_output = mono_right.requestOutput(DEPTH_SIZE, type=dai.ImgFrame.Type.GRAY8)
    stereo = pipeline.create(dai.node.StereoDepth)
    left_output.link(stereo.left)
    right_output.link(stereo.right)
    return (
        pipeline,
        color_output.createOutputQueue(maxSize=4, blocking=False),
        stereo.depth.createOutputQueue(maxSize=4, blocking=False),
    )


def main() -> int:
    """Capture aligned pose landmarks and an annotated preview video."""
    args = parse_args()
    if args.duration < 0:
        print("Duration cannot be negative.")
        return 2
    try:
        import cv2
        import depthai as dai
        import mediapipe as mp
        import numpy as np
    except ImportError as error:
        print(f"Camera dependency missing: {error.name}")
        print("Recreate the simulation environment before using OAK-D recording.")
        return 2

    try:
        pipeline, video_queue, depth_queue = build_pipeline(dai)
        pipeline.start()
    except Exception as error:
        print(f"Could not start the OAK-D camera: {error}")
        return 1

    session = create_session_directory("oak_pose", args.subject)
    video_path = session / "video.mp4"
    pose_path = session / "pose.json"
    writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        30.0,
        FRAME_SIZE,
    )
    if not writer.isOpened():
        print(f"Could not open video output: {video_path}")
        stop = getattr(pipeline, "stop", None)
        if callable(stop):
            stop()
        return 1

    mp_hands = mp.solutions.hands
    mp_pose = mp.solutions.pose
    drawing = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    side_prefix = args.side.upper()
    shoulder_index = getattr(mp_pose.PoseLandmark, f"{side_prefix}_SHOULDER")
    elbow_index = getattr(mp_pose.PoseLandmark, f"{side_prefix}_ELBOW")
    wrist_index = getattr(mp_pose.PoseLandmark, f"{side_prefix}_WRIST")

    frames: list[dict[str, object]] = []
    video_frames = 0
    started = time.monotonic()
    started_utc = utc_now()
    latest_depth = None
    print(f"Recording the {args.side} arm. Press Q in the preview or Stop in the GUI.")
    print(f"Saving to {session}")

    try:
        while pipeline.isRunning() and (
            args.duration == 0 or time.monotonic() - started < args.duration
        ):
            frame_message = video_queue.get()
            depth_message = depth_queue.tryGet()
            if depth_message is not None:
                latest_depth = depth_message.getFrame()
            frame = frame_message.getCvFrame()
            if frame is None or latest_depth is None:
                continue

            height, width = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            hand_results = hands.process(rgb)
            pose_results = pose.process(rgb)
            if pose_results.pose_landmarks:
                drawing.draw_landmarks(
                    frame,
                    pose_results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                )
                landmarks = pose_results.pose_landmarks.landmark
                shoulder_lm = landmarks[shoulder_index]
                elbow_lm = landmarks[elbow_index]
                wrist_lm = landmarks[wrist_index]
                visible = min(shoulder_lm.visibility, elbow_lm.visibility, wrist_lm.visibility)
                if visible >= 0.5:
                    wrist_xy = (int(wrist_lm.x * width), int(wrist_lm.y * height))
                    hand = nearest_hand(hand_results, wrist_xy)
                    hand_points: list[dict[str, float | int]] = []
                    if hand is not None:
                        drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)
                        for landmark_id, landmark in enumerate(hand.landmark):
                            point = camera_point(landmark, latest_depth, width, height, np)
                            if point is not None:
                                hand_points.append(
                                    {
                                        "id": landmark_id,
                                        "x": point[0],
                                        "y": point[1],
                                        "depth_m": point[2],
                                    }
                                )

                    shoulder = camera_point(shoulder_lm, latest_depth, width, height, np)
                    elbow = camera_point(elbow_lm, latest_depth, width, height, np)
                    wrist = camera_point(wrist_lm, latest_depth, width, height, np)
                    if wrist is not None and not any(point["id"] == 0 for point in hand_points):
                        hand_points.insert(
                            0,
                            {"id": 0, "x": wrist[0], "y": wrist[1], "depth_m": wrist[2]},
                        )
                    if shoulder is not None and elbow is not None and wrist is not None:
                        frames.append(
                            {
                                "time_s": round(time.monotonic() - started, 4),
                                "shoulder": shoulder,
                                "elbow": elbow,
                                "hand": hand_points,
                            }
                        )

            cv2.putText(
                frame,
                f"REC  {len(frames)} pose frames  |  Q stops",
                (18, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (20, 20, 240),
                2,
            )
            writer.write(frame)
            video_frames += 1
            cv2.imshow("AURORA OAK-D pose recording", cv2.flip(frame, 1))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("Stopping OAK-D recording...")
    finally:
        writer.release()
        hands.close()
        pose.close()
        cv2.destroyAllWindows()
        stop = getattr(pipeline, "stop", None)
        if callable(stop):
            stop()

        pose_path.write_text(
            json.dumps(
                {
                    "user_id": args.subject,
                    "sequence": len(frames),
                    "side": args.side,
                    "data": frames,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        write_metadata(
            session,
            {
                "source": "oak_pose",
                "subject": args.subject,
                "side": args.side,
                "started_utc": started_utc,
                "duration_seconds": round(time.monotonic() - started, 3),
                "pose_frames": len(frames),
                "video_frames": video_frames,
            },
        )

    print(f"Saved {len(frames)} pose frames to {pose_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
