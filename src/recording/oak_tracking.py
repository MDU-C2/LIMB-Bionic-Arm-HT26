# Christian, Majd
"""Analyze camera frames and draw the tracked arm and hand."""

from __future__ import annotations

from oak_geometry import (
    FINGER_CHAINS,
    HAND_LANDMARK_NAMES,
    PALM_CONNECTIONS,
    TIP_LABELS,
    arm_angles_deg,
    camera_point,
    camera_xyz,
    display_pixel,
    finger_angles_deg,
    landmark_ids,
    select_hand_near_wrist,
)


def add_hand_tracking(
    output, observation, sample, hand_results, pose_wrist, depth, width: int, height: int, cv2, np
) -> bool:
    """Draw and save only the hand connected to the selected arm."""
    selected = select_hand_near_wrist(hand_results, pose_wrist)
    if selected is None:
        return False
    _, hand, world, distance = selected
    landmarks = hand.landmark
    raw_pixels = [
        (
            min(width - 1, max(0, int(point.x * width))),
            min(height - 1, max(0, int(point.y * height))),
        )
        for point in landmarks
    ]
    pixels = [display_pixel(x, y, width) for x, y in raw_pixels]
    for chain in FINGER_CHAINS.values():
        for first, second in zip(chain, chain[1:], strict=False):
            cv2.line(output, pixels[first], pixels[second], (80, 235, 105), 2)
    for first, second in PALM_CONNECTIONS:
        cv2.line(output, pixels[first], pixels[second], (80, 235, 105), 2)
    for index, xy in enumerate(pixels):
        cv2.circle(output, xy, 3 if index not in TIP_LABELS else 5, (60, 220, 90), -1)
        if index in TIP_LABELS:
            cv2.putText(
                output,
                TIP_LABELS[index],
                (xy[0] + 5, xy[1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (60, 220, 90),
                1,
            )

    angle_points = world.landmark if world is not None else landmarks
    angles, curls = finger_angles_deg(angle_points, np)
    observation["hand_visible"] = True
    observation["hand_selection"] = "nearest_selected_arm_wrist"
    observation["hand_wrist_distance_normalized"] = round(float(distance), 5)
    observation["hand_keypoints_2d"] = {
        name: [round(float(point.x), 5), round(float(point.y), 5), round(float(point.z), 5)]
        for name, point in zip(HAND_LANDMARK_NAMES, landmarks, strict=False)
    }
    observation["finger_angles_deg"] = angles
    observation["finger_curl_deg"] = curls
    observation["finger_angle_source"] = (
        "mediapipe_hand_world_estimate"
        if world is not None
        else "mediapipe_normalized_hand_estimate"
    )

    if sample is not None:
        depth_points = []
        for index, (name, point) in enumerate(zip(HAND_LANDMARK_NAMES, landmarks, strict=False)):
            measured = camera_point(point, depth, width, height, np)
            if measured is not None:
                depth_points.append(
                    {
                        "id": index,
                        "name": name,
                        "x": measured[0],
                        "y": measured[1],
                        "depth_m": measured[2],
                    }
                )
        if depth_points:
            sample["hand"] = depth_points
            sample["finger_angles_deg"] = angles
    return True


def analyze_frame(frame, depth, pose, mp, cv2, np, side: str, intrinsics=None, hands=None):
    """Track on the raw frame and render a mirrored, readable operator view."""
    height, width = frame.shape[:2]
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb)
    # Tracking, depth, and saved coordinates stay in the camera's raw frame.
    # Mirror only the pixels, then draw annotations so their text is readable.
    output = frame[:, ::-1].copy()
    observation = {
        "person_visible": False,
        "arm_visible": False,
        "hand_visible": False,
        "reference_visible": False,
        "depth_valid": False,
    }
    if not results.pose_landmarks:
        return output, f"No person visible - frame subject, {side} arm and hips", None, observation

    ids = landmark_ids(mp.solutions.pose, side)
    landmarks = results.pose_landmarks.landmark
    selected = {name: landmarks[index] for name, index in ids.items()}
    observation["person_visible"] = True
    observation["keypoints_2d"] = {
        name: [
            round(float(point.x), 5),
            round(float(point.y), 5),
            round(float(point.visibility), 3),
        ]
        for name, point in selected.items()
    }
    arm_names = (f"{side}_shoulder", f"{side}_elbow", f"{side}_wrist")
    if min(selected[name].visibility for name in arm_names) < 0.5:
        return output, f"{side.title()} arm obscured or outside frame", None, observation

    observation["arm_visible"] = True
    observation["reference_visible"] = all(point.visibility >= 0.5 for point in selected.values())
    observation["arm_2d"] = {
        name.rsplit("_", 1)[-1]: observation["keypoints_2d"][name] for name in arm_names
    }
    raw_arm_pixels = [
        (
            min(width - 1, max(0, int(selected[name].x * width))),
            min(height - 1, max(0, int(selected[name].y * height))),
        )
        for name in arm_names
    ]
    arm_pixels = [display_pixel(x, y, width) for x, y in raw_arm_pixels]
    for first, second in zip(arm_pixels, arm_pixels[1:], strict=False):
        cv2.line(output, first, second, (0, 220, 255), 3)
    for label, xy in zip(("S", "E", "W"), arm_pixels, strict=False):
        cv2.circle(output, xy, 6, (0, 220, 255), -1)
        cv2.putText(
            output, label, (xy[0] + 8, xy[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 220, 255), 2
        )

    sample = None
    arm_depth = [camera_point(selected[name], depth, width, height, np) for name in arm_names]
    if all(point is not None for point in arm_depth):
        shoulder, elbow, wrist = arm_depth
        sample = {
            "shoulder": shoulder,
            "elbow": elbow,
            "hand": [{"id": 0, "x": wrist[0], "y": wrist[1], "depth_m": wrist[2]}],
        }
        observation["depth_valid"] = True

    if observation["reference_visible"]:
        world = getattr(results, "pose_world_landmarks", None)
        if world is not None:
            # MediaPipe world Y increases downward and Z increases away from
            # the camera; use Y up and Z toward the camera for the trunk frame.
            world_points = {
                name: [world.landmark[index].x, -world.landmark[index].y, -world.landmark[index].z]
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
                    name: camera_xyz(point, intrinsics) for name, point in six_depth.items()
                }
                stereo_angles = arm_angles_deg(camera_points, side, np)
                sample["keypoints_camera_m"] = camera_points
                if stereo_angles is not None:
                    sample["angles_deg"] = stereo_angles
                    observation["angles_deg"] = stereo_angles
                    observation["angle_source"] = "stereo"

    if hands is not None:
        hand_results = hands.process(rgb)
        add_hand_tracking(
            output,
            observation,
            sample,
            hand_results,
            selected[f"{side}_wrist"],
            depth,
            width,
            height,
            cv2,
            np,
        )

    if not observation["reference_visible"]:
        status = f"{side.title()} arm tracked | frame both shoulders and hips for angles"
    elif "angles_deg" not in observation:
        status = f"{side.title()} arm tracked | 3D pose unavailable"
    else:
        source = "stereo" if observation["angle_source"] == "stereo" else "estimated 3D"
        status = f"{side.title()} arm tracked | angles from {source}"
    status += " | hand/fingers tracked" if observation["hand_visible"] else " | show selected hand"
    return output, status, sample, observation
