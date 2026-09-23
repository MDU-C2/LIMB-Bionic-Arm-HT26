"""Record pose and hand landmarks from an OAK camera for GRU training."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

os.environ.setdefault(
    "MPLCONFIGDIR", str(Path(__file__).resolve().parents[2] / "artifacts" / ".matplotlib")
)

import cv2
import depthai as dai
import mediapipe as mp


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = REPOSITORY_ROOT / "data" / "movement" / "training"


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--user-id", type=int, required=True)
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def next_sequence(folder: Path, user_id: int) -> int:
    used = []
    for path in folder.glob(f"ID{user_id}run*.json"):
        try:
            used.append(int(path.stem.split("run", 1)[1]))
        except (IndexError, ValueError):
            pass
    return max(used, default=0) + 1


def point(landmark, width: int, height: int) -> list[float]:
    return [float(landmark.x * width), float(landmark.y * height), float(landmark.z)]


def main() -> None:
    args = arguments()
    args.output.mkdir(parents=True, exist_ok=True)
    sequence = next_sequence(args.output, args.user_id)
    pose_index = (12, 14) if args.side == "right" else (11, 13)
    captured: list[dict] = []
    recording = False

    pose_api = mp.solutions.pose
    hand_api = mp.solutions.hands
    with pose_api.Pose(model_complexity=1, min_detection_confidence=0.5) as pose, hand_api.Hands(
        max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5
    ) as hands, dai.Pipeline() as pipeline:
        camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        queue = camera.requestOutput((640, 480), fps=20).createOutputQueue()
        pipeline.start()

        while pipeline.isRunning():
            frame = queue.get().getCvFrame()
            height, width = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pose_result = pose.process(rgb)
            hand_result = hands.process(rgb)
            valid = bool(pose_result.pose_landmarks and hand_result.multi_hand_landmarks)

            if valid:
                pose_points = pose_result.pose_landmarks.landmark
                hand_points = hand_result.multi_hand_landmarks[0].landmark
                for index in pose_index:
                    landmark = pose_points[index]
                    cv2.circle(frame, (int(landmark.x * width), int(landmark.y * height)), 7, (0, 255, 0), -1)
                mp.solutions.drawing_utils.draw_landmarks(
                    frame, hand_result.multi_hand_landmarks[0], hand_api.HAND_CONNECTIONS
                )
                if recording:
                    captured.append(
                        {
                            "shoulder": point(pose_points[pose_index[0]], width, height),
                            "elbow": point(pose_points[pose_index[1]], width, height),
                            "hand": [
                                {"id": index, "x": values[0], "y": values[1], "depth_m": values[2]}
                                for index, landmark in enumerate(hand_points)
                                for values in [point(landmark, width, height)]
                            ],
                        }
                    )

            status = f"RECORDING {len(captured)} frames" if recording else "SPACE: start recording"
            color = (0, 0, 255) if recording else (255, 255, 255)
            cv2.putText(frame, status, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            if not valid:
                cv2.putText(frame, "Show shoulder, elbow and hand", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            cv2.imshow("AURORA movement recorder", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 32:
                if recording and captured:
                    break
                recording = True
                captured.clear()
            if key in (ord("q"), 27):
                captured.clear()
                break

    cv2.destroyAllWindows()
    if captured:
        path = args.output / f"ID{args.user_id}run{sequence}.json"
        payload = {"user_id": args.user_id, "sequence": sequence, "captured_at": time.time(), "data": captured}
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"Saved {len(captured)} frames to {path}")


if __name__ == "__main__":
    try:
        main()
    finally:
        cv2.destroyAllWindows()
