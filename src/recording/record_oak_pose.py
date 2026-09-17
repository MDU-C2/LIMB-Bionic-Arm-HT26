"""Record the six upper-body points used for Oscar's left-arm angles."""

from __future__ import annotations

import argparse
import json
import os
import time

from common import create_session_directory, env_float, experiment_metadata, utc_now, write_metadata


FRAME_SIZE = (640, 480)
DEPTH_SIZE = (640, 400)
VIDEO_FPS = 25.0
WINDOW = "AURORA OAK-D | arm and mug task"
ANGLE_LABELS = (
    ("elbow_flexion", "Elbow flex"),
    ("shoulder_flexion", "Shoulder flex"),
    ("shoulder_abduction", "Shoulder abd"),
    ("shoulder_rotation_proxy", "Rotation proxy"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--subject", default=os.environ.get("AURORA_SUBJECT", "session"))
    parser.add_argument("--side", choices=("left", "right"),
                        default=os.environ.get("AURORA_CAMERA_SIDE", "left"),
                        help="Subject's arm to highlight and save (default: left).")
    parser.add_argument("--preview", action="store_true",
                        help="Compatibility option; live preview is always shown first.")
    parser.add_argument("--depth", action="store_true",
                        help="Enable stereo depth for 3D playback points (experimental on this OAK-D).")
    parser.add_argument("--pose-model", type=int, choices=(0, 1, 2), default=1,
                        help="MediaPipe pose complexity; 2 is closest to Oscar's heavy model.")
    parser.add_argument("--duration", type=float,
                        default=env_float("AURORA_DURATION_SECONDS", 0.0),
                        help="Maximum seconds per recording; 0 means until stopped.")
    return parser.parse_args()


def depth_at(depth_frame, x: int, y: int, width: int, height: int, np) -> float | None:
    """Median RGB-aligned stereo depth in metres at one image point."""
    depth_height, depth_width = depth_frame.shape
    depth_x = int(x * depth_width / width)
    depth_y = int(y * depth_height / height)
    # Oscar sampled the median of a 3-by-3 patch at each pose point.
    radius = 1
    region = depth_frame[
        max(0, depth_y - radius):min(depth_height, depth_y + radius + 1),
        max(0, depth_x - radius):min(depth_width, depth_x + radius + 1),
    ]
    valid = region[(region > 0) & (region < 3000)]
    return None if valid.size == 0 else float(np.median(valid)) / 1000.0


def camera_point(landmark, depth_frame, width: int, height: int, np) -> list[float] | None:
    if depth_frame is None or not (0 <= landmark.x <= 1 and 0 <= landmark.y <= 1):
        return None
    x = min(width - 1, int(landmark.x * width))
    y = min(height - 1, int(landmark.y * height))
    depth = depth_at(depth_frame, x, y, width, height, np)
    return None if depth is None else [float(x), float(y), depth]


def landmark_ids(mp_pose, side: str) -> dict[str, int]:
    """Select Oscar's arm and trunk points, keeping anatomical side labels."""
    other = "right" if side == "left" else "left"
    return {
        f"{side}_shoulder": getattr(mp_pose.PoseLandmark, f"{side.upper()}_SHOULDER"),
        f"{side}_elbow": getattr(mp_pose.PoseLandmark, f"{side.upper()}_ELBOW"),
        f"{side}_wrist": getattr(mp_pose.PoseLandmark, f"{side.upper()}_WRIST"),
        f"{other}_shoulder": getattr(mp_pose.PoseLandmark, f"{other.upper()}_SHOULDER"),
        "left_hip": mp_pose.PoseLandmark.LEFT_HIP,
        "right_hip": mp_pose.PoseLandmark.RIGHT_HIP,
    }


def arm_angles_deg(points: dict[str, object], side: str, np) -> dict[str, float | None] | None:
    """Oscar's trunk-relative elbow/shoulder geometry from six 3D points.

    Shoulder axial rotation is only a forearm-based proxy. It is undefined
    when the elbow is nearly straight or its transverse projection vanishes.
    """
    other = "right" if side == "left" else "left"
    try:
        shoulder = np.asarray(points[f"{side}_shoulder"], dtype=float)
        elbow = np.asarray(points[f"{side}_elbow"], dtype=float)
        wrist = np.asarray(points[f"{side}_wrist"], dtype=float)
        opposite = np.asarray(points[f"{other}_shoulder"], dtype=float)
        left_hip = np.asarray(points["left_hip"], dtype=float)
        right_hip = np.asarray(points["right_hip"], dtype=float)
    except (KeyError, TypeError, ValueError):
        return None
    if not all(np.all(np.isfinite(p)) and p.shape == (3,) for p in
               (shoulder, elbow, wrist, opposite, left_hip, right_hip)):
        return None

    def unit(vector):
        length = float(np.linalg.norm(vector))
        return None if length < 1e-6 else vector / length

    upper = elbow - shoulder
    forearm = wrist - elbow
    upper_unit, forearm_unit = unit(upper), unit(forearm)
    lateral = unit(shoulder - opposite)
    vertical = unit((shoulder + opposite - left_hip - right_hip) / 2.0)
    if any(vector is None for vector in (upper_unit, forearm_unit, lateral, vertical)):
        return None
    forward = unit(np.cross(lateral, vertical))
    if forward is None:
        return None
    lateral = unit(np.cross(vertical, forward))
    vertical = unit(np.cross(forward, lateral))
    trunk = np.column_stack((lateral, vertical, forward))
    upper_t = trunk.T @ upper_unit
    forearm_t = trunk.T @ forearm_unit
    elbow_angle = float(np.arccos(np.clip(np.dot(upper_unit, forearm_unit), -1.0, 1.0)))
    flexion = float(np.arctan2(upper_t[2], -upper_t[1]))
    abduction = float(np.arctan2(upper_t[0], -upper_t[1]))

    # Undo shoulder abduction and flexion before projecting the forearm onto
    # the transverse plane, following the report's lateral/medial proxy.
    ca, sa = np.cos(abduction), np.sin(abduction)
    cf, sf = np.cos(flexion), np.sin(flexion)
    undo_abduction = np.array([[ca, sa, 0], [-sa, ca, 0], [0, 0, 1]])
    undo_flexion = np.array([[1, 0, 0], [0, cf, -sf], [0, sf, cf]])
    compensated = undo_flexion @ undo_abduction @ forearm_t
    transverse = unit(np.array([compensated[0], 0.0, compensated[2]]))
    rotation = None if transverse is None or elbow_angle < np.radians(12) else float(
        np.degrees(np.arctan2(-transverse[0], transverse[2]))
    )
    return {
        "elbow_flexion": round(float(np.degrees(elbow_angle)), 1),
        "shoulder_flexion": round(float(np.degrees(flexion)), 1),
        "shoulder_abduction": round(float(np.degrees(abduction)), 1),
        "shoulder_rotation_proxy": None if rotation is None else round(rotation, 1),
    }


def camera_xyz(point: list[float], intrinsics) -> list[float]:
    """Deproject an RGB pixel and aligned depth using camera calibration."""
    x, y, depth_m = point
    return [
        (x - intrinsics[0][2]) * depth_m / intrinsics[0][0],
        -(y - intrinsics[1][2]) * depth_m / intrinsics[1][1],
        depth_m,
    ]


def build_pipeline(dai, use_depth: bool = False):
    """Run RGB tracking; add stereo only when explicitly requested."""
    pipeline = dai.Pipeline()
    color = pipeline.create(dai.node.Camera).build()
    color.setSensorType(dai.CameraSensorType.COLOR)
    color_output = color.requestOutput(FRAME_SIZE, type=dai.ImgFrame.Type.BGR888p,
                                       fps=VIDEO_FPS)
    video_queue = color_output.createOutputQueue(maxSize=2, blocking=False)
    if not use_depth:
        return pipeline, video_queue, None
    mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    left_output = mono_left.requestOutput(DEPTH_SIZE, type=dai.ImgFrame.Type.GRAY8,
                                          fps=VIDEO_FPS)
    right_output = mono_right.requestOutput(DEPTH_SIZE, type=dai.ImgFrame.Type.GRAY8,
                                            fps=VIDEO_FPS)
    stereo = pipeline.create(dai.node.StereoDepth)
    left_output.link(stereo.left)
    right_output.link(stereo.right)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(*DEPTH_SIZE)
    return (
        pipeline,
        video_queue,
        stereo.depth.createOutputQueue(maxSize=2, blocking=False),
    )


def analyze_frame(frame, depth, pose, mp, cv2, np, side: str, intrinsics=None):
    """Track six anatomical points and draw readable labels on the unflipped frame."""
    height, width = frame.shape[:2]
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb)
    output = frame.copy()
    observation = {
        "person_visible": False, "arm_visible": False,
        "reference_visible": False, "depth_valid": False,
    }
    if not results.pose_landmarks:
        return output, "No person visible - frame subject, left arm and hips", None, observation

    ids = landmark_ids(mp.solutions.pose, side)
    landmarks = results.pose_landmarks.landmark
    selected = {name: landmarks[index] for name, index in ids.items()}
    observation["person_visible"] = True
    observation["keypoints_2d"] = {
        name: [round(float(point.x), 5), round(float(point.y), 5),
               round(float(point.visibility), 3)]
        for name, point in selected.items()
    }
    arm_names = (f"{side}_shoulder", f"{side}_elbow", f"{side}_wrist")
    if min(selected[name].visibility for name in arm_names) < 0.5:
        return output, f"{side.title()} arm obscured or outside frame", None, observation

    observation["arm_visible"] = True
    observation["reference_visible"] = all(point.visibility >= 0.5 for point in selected.values())
    observation["arm_2d"] = {
        name.rsplit("_", 1)[-1]: observation["keypoints_2d"][name]
        for name in arm_names
    }
    arm_pixels = [
        (min(width - 1, max(0, int(selected[name].x * width))),
         min(height - 1, max(0, int(selected[name].y * height))))
        for name in arm_names
    ]
    for first, second in zip(arm_pixels, arm_pixels[1:]):
        cv2.line(output, first, second, (0, 220, 255), 3)
    for label, xy in zip(("S", "E", "W"), arm_pixels):
        cv2.circle(output, xy, 6, (0, 220, 255), -1)
        cv2.putText(output, label, (xy[0] + 8, xy[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 220, 255), 2)

    sample = None
    arm_depth = [camera_point(selected[name], depth, width, height, np) for name in arm_names]
    if all(point is not None for point in arm_depth):
        shoulder, elbow, wrist = arm_depth
        sample = {
            "shoulder": shoulder, "elbow": elbow,
            "hand": [{"id": 0, "x": wrist[0], "y": wrist[1], "depth_m": wrist[2]}],
        }
        observation["depth_valid"] = True

    if observation["reference_visible"]:
        world = getattr(results, "pose_world_landmarks", None)
        if world is not None:
            # MediaPipe world Y increases downward and Z increases away from
            # the camera; use Y up and Z toward the camera for the trunk frame.
            world_points = {
                name: [world.landmark[index].x, -world.landmark[index].y,
                       -world.landmark[index].z]
                for name, index in ids.items()
            }
            angles = arm_angles_deg(world_points, side, np)
            if angles is not None:
                observation["angles_deg"] = angles
                observation["angle_source"] = "mediapipe_world_estimate"

        if sample is not None and intrinsics is not None:
            six_depth = {
                name: camera_point(point, depth, width, height, np)
                for name, point in selected.items()
            }
            if all(point is not None for point in six_depth.values()):
                camera_points = {
                    name: camera_xyz(point, intrinsics)
                    for name, point in six_depth.items()
                }
                stereo_angles = arm_angles_deg(camera_points, side, np)
                sample["keypoints_camera_m"] = camera_points
                if stereo_angles is not None:
                    sample["angles_deg"] = stereo_angles
                    observation["angles_deg"] = stereo_angles
                    observation["angle_source"] = "stereo"

    if not observation["reference_visible"]:
        status = f"{side.title()} arm tracked | frame both shoulders and hips for angles"
    elif "angles_deg" not in observation:
        status = f"{side.title()} arm tracked | 3D pose unavailable"
    else:
        source = "stereo" if observation["angle_source"] == "stereo" else "estimated 3D"
        status = f"{side.title()} arm tracked | angles from {source}"
    return output, status, sample, observation


class CameraSession:
    """Start and stop files without interrupting the live camera."""

    def __init__(self, cv2, subject: str, side: str, depth_enabled: bool = False):
        self.cv2 = cv2
        self.subject = subject
        self.side = side
        self.depth_enabled = depth_enabled
        self.writer = None
        self.session = None
        self.started = 0.0
        self.started_utc = ""
        self.video_frames = 0
        self.pose_frames: list[dict[str, object]] = []
        self.observations: list[dict[str, object]] = []

    @property
    def recording(self) -> bool:
        return self.writer is not None

    def start(self) -> None:
        self.session = create_session_directory("oak_pose", self.subject)
        writer = self.cv2.VideoWriter(
            str(self.session / "video.mp4"), self.cv2.VideoWriter_fourcc(*"mp4v"),
            VIDEO_FPS, FRAME_SIZE
        )
        if not writer.isOpened():
            writer.release()
            self.session.rmdir()
            self.session = None
            raise RuntimeError("Could not open video.mp4 in the selected output folder")
        self.writer = writer
        self.started = time.monotonic()
        self.started_utc = utc_now()
        self.video_frames = 0
        self.pose_frames = []
        self.observations = []
        print(f"REC started: {self.session}")

    def add_frame(self, frame, sample, observation) -> None:
        if not self.recording:
            return
        elapsed = time.monotonic() - self.started
        self.observations.append({"time_s": round(elapsed, 4), **observation})
        if sample is not None:
            self.pose_frames.append({"time_s": round(elapsed, 4), **sample})
        # VideoWriter uses fixed FPS; duplicate when tracking is slower so
        # playback duration stays close to wall-clock recording time.
        expected = min(max(self.video_frames + 1, round(elapsed * VIDEO_FPS)),
                       self.video_frames + int(VIDEO_FPS))
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
        pose_path.write_text(json.dumps({
            "user_id": self.subject, "sequence": len(self.pose_frames),
            "side": self.side, "data": self.pose_frames,
            "observations": self.observations,
            "coordinate_note": (
                "keypoints_2d are normalized raw-camera x/y; data x/y are raw "
                "camera pixels. video.mp4 is unflipped; depth_m "
                "is RGB-aligned stereo depth"
            ),
            "angle_note": (
                "Elbow flexion, shoulder flexion, abduction and a forearm-based "
                "rotation proxy follow Oscar's "
                "six-point trunk frame; mediapipe_world_estimate is monocular"
            ),
        }, indent=2), encoding="utf-8")
        write_metadata(self.session, {
            "source": "oak_pose", "subject": self.subject, "side": self.side,
            "started_utc": self.started_utc, "duration_seconds": round(elapsed, 3),
            "pose_frames": len(self.pose_frames),
            "observation_frames": len(self.observations),
            "video_frames": self.video_frames, "video_fps": VIDEO_FPS,
            "stereo_depth_enabled": self.depth_enabled,
            "video_mirrored": False,
            "cup_detector": "unavailable_no_model", **experiment_metadata(),
        })
        print(f"REC saved: {self.session} ({len(self.pose_frames)} depth-backed pose frames)")


def draw_window(frame, status: str, session: CameraSession, cv2, np, observation=None):
    """Render a clickable REC button below the camera image."""
    height, width = frame.shape[:2]
    canvas = np.zeros((height + 136, width, 3), dtype=np.uint8)
    canvas[:height] = frame
    canvas[height:] = (26, 31, 37)
    active = session.recording
    button = (width - 168, height + 88, width - 16, height + 128)
    cv2.rectangle(canvas, (button[0], button[1]), (button[2], button[3]),
                  (40, 40, 210) if active else (35, 55, 205), -1)
    cv2.putText(canvas, "STOP REC" if active else "START REC",
                (button[0] + 15, button[1] + 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
    cv2.putText(canvas, status[:78], (12, height + 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (235, 235, 235), 1)
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
    cv2.putText(canvas, angle_lines[0], (12, height + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.47, (90, 200, 255), 1)
    cv2.putText(canvas, angle_lines[1], (12, height + 73),
                cv2.FONT_HERSHEY_SIMPLEX, 0.47, (90, 200, 255), 1)
    cv2.putText(canvas, "Cup detector: model unavailable", (12, height + 97),
                cv2.FONT_HERSHEY_SIMPLEX, 0.47, (160, 180, 200), 1)
    cv2.putText(canvas, "R: record / stop    Q: close", (12, height + 121),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (190, 190, 190), 1)
    if active:
        cv2.circle(canvas, (width - 108, 26), 8, (25, 25, 240), -1)
        cv2.putText(canvas, f"REC {time.monotonic() - session.started:.1f}s",
                    (width - 94, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (25, 25, 240), 2)
    return canvas, button


def main() -> int:
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
            intrinsics = calibration.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A, *FRAME_SIZE
            )
        except Exception as error:
            print(f"Camera calibration unavailable; stereo angles disabled: {error}")

    session = CameraSession(cv2, args.subject, args.side, args.depth)
    controls = {"toggle": False, "button": None}

    def on_mouse(event, x, y, _flags, _userdata):
        rect = controls["button"]
        if (event == cv2.EVENT_LBUTTONUP and rect
                and rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]):
            controls["toggle"] = True

    latest_depth = None
    pose = None
    try:
        pose = mp.solutions.pose.Pose(
            static_image_mode=False, model_complexity=args.pose_model,
            min_detection_confidence=0.5, min_tracking_confidence=0.5
        )
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW, on_mouse)
        print(f"Live OAK-D tracking: subject's {args.side} shoulder, elbow and wrist.")
        print("Camera video is unflipped; anatomical landmark selection uses the raw frame.")
        print("Cup detection needs a model; no cup model is present in this repository.")
        if not args.depth:
            print("Stereo depth is off; use --depth to attempt depth-backed playback points.")
        while pipeline.isRunning():
            if depth_queue is not None:
                depth_message = depth_queue.tryGet()
                if depth_message is not None:
                    latest_depth = depth_message.getFrame()
            frame_message = video_queue.tryGet()
            if frame_message is not None:
                frame = frame_message.getCvFrame()
                if frame is not None:
                    image, status, sample, observation = analyze_frame(
                        frame, latest_depth, pose, mp, cv2, np, args.side, intrinsics
                    )
                    session.add_frame(image, sample, observation)
                    canvas, controls["button"] = draw_window(
                        image, status, session, cv2, np, observation
                    )
                    cv2.imshow(WINDOW, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if (hasattr(cv2, "getWindowProperty")
                    and cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1):
                break
            if key == ord("r") or controls["toggle"]:
                controls["toggle"] = False
                try:
                    session.stop() if session.recording else session.start()
                except (OSError, RuntimeError) as error:
                    print(f"Recording error: {error}")
            if (session.recording and args.duration
                    and time.monotonic() - session.started >= args.duration):
                session.stop()
    except KeyboardInterrupt:
        pass
    finally:
        session.stop()
        if pose is not None:
            pose.close()
        cv2.destroyAllWindows()
        stop = getattr(pipeline, "stop", None)
        if callable(stop):
            stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
