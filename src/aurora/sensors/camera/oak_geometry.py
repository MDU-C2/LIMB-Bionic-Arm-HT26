# Christian, Majd
"""Arm and finger geometry from the existing OAK-D tracker."""

from __future__ import annotations

ANGLE_LABELS = (
    ("elbow_flexion", "Elbow flex"),
    ("shoulder_flexion", "Shoulder flex"),
    ("shoulder_abduction", "Shoulder abd"),
    ("shoulder_rotation_proxy", "Rotation proxy"),
)


HAND_LANDMARK_NAMES = (
    "wrist",
    "thumb_cmc",
    "thumb_mcp",
    "thumb_ip",
    "thumb_tip",
    "index_mcp",
    "index_pip",
    "index_dip",
    "index_tip",
    "middle_mcp",
    "middle_pip",
    "middle_dip",
    "middle_tip",
    "ring_mcp",
    "ring_pip",
    "ring_dip",
    "ring_tip",
    "pinky_mcp",
    "pinky_pip",
    "pinky_dip",
    "pinky_tip",
)


FINGER_CHAINS = {
    "thumb": (0, 1, 2, 3, 4),
    "index": (0, 5, 6, 7, 8),
    "middle": (0, 9, 10, 11, 12),
    "ring": (0, 13, 14, 15, 16),
    "pinky": (0, 17, 18, 19, 20),
}


PALM_CONNECTIONS = ((0, 5), (5, 9), (9, 13), (13, 17), (17, 0))


TIP_LABELS = {4: "T", 8: "I", 12: "M", 16: "R", 20: "P"}


def depth_at(depth_frame, x: int, y: int, width: int, height: int, np) -> float | None:
    """Median RGB-aligned stereo depth in metres at one image point."""
    depth_height, depth_width = depth_frame.shape
    depth_x = int(x * depth_width / width)
    depth_y = int(y * depth_height / height)
    # Oscar sampled the median of a 3-by-3 patch at each pose point.
    radius = 1
    region = depth_frame[
        max(0, depth_y - radius) : min(depth_height, depth_y + radius + 1),
        max(0, depth_x - radius) : min(depth_width, depth_x + radius + 1),
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


def elbow_flexion_deg(points, side, np):
    """Elbow flexion needs the shoulder, elbow and wrist, without a trunk frame."""
    try:
        shoulder, elbow, wrist = (
            np.asarray(points[f"{side}_{name}"], dtype=float)
            for name in ("shoulder", "elbow", "wrist")
        )
    except (KeyError, TypeError, ValueError):
        return None
    if any(
        point.shape != (3,) or not np.all(np.isfinite(point)) for point in (shoulder, elbow, wrist)
    ):
        return None
    upper, forearm = elbow - shoulder, wrist - elbow
    length = np.linalg.norm(upper) * np.linalg.norm(forearm)
    if length < 1e-10:
        return None
    return float(np.degrees(np.arccos(np.clip(np.dot(upper, forearm) / length, -1.0, 1.0))))


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
    if not all(
        np.all(np.isfinite(p)) and p.shape == (3,)
        for p in (shoulder, elbow, wrist, opposite, left_hip, right_hip)
    ):
        return None

    def unit(vector):
        length = float(np.linalg.norm(vector))
        return None if length < 1e-6 else vector / length

    upper = elbow - shoulder
    forearm = wrist - elbow
    upper_unit, forearm_unit = unit(upper), unit(forearm)
    # Keep one trunk frame for both selectable arms. Building lateral from the
    # selected shoulder reverses camera forward whenever ``side == right`` and
    # makes the same forward reach report opposite flexion.
    try:
        left_shoulder = np.asarray(points["left_shoulder"], dtype=float)
        right_shoulder = np.asarray(points["right_shoulder"], dtype=float)
    except (KeyError, TypeError, ValueError):
        return None
    lateral = unit(left_shoulder - right_shoulder)
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
    # Express lateral motion as outward-positive for whichever anatomical arm
    # is selected while leaving trunk forward/up identical.
    side_sign = 1.0 if side == "left" else -1.0
    upper_t[0] *= side_sign
    forearm_t[0] *= side_sign
    elbow_angle = float(np.arccos(np.clip(np.dot(upper_unit, forearm_unit), -1.0, 1.0)))
    # Include the other elevation component to avoid singularities with a
    # horizontal arm. Oscar's independent planar equations omit this component.
    flexion = float(np.arctan2(upper_t[2], np.hypot(upper_t[0], upper_t[1])))
    abduction = float(np.arctan2(upper_t[0], np.hypot(upper_t[1], upper_t[2])))

    # Undo shoulder abduction and flexion before projecting the forearm onto
    # the transverse plane, following the report's lateral/medial proxy.
    ca, sa = np.cos(abduction), np.sin(abduction)
    cf, sf = np.cos(flexion), np.sin(flexion)
    undo_abduction = np.array([[ca, sa, 0], [-sa, ca, 0], [0, 0, 1]])
    undo_flexion = np.array([[1, 0, 0], [0, cf, -sf], [0, sf, cf]])
    compensated = undo_flexion @ undo_abduction @ forearm_t
    transverse = unit(np.array([compensated[0], 0.0, compensated[2]]))
    rotation = (
        None
        if transverse is None or elbow_angle < np.radians(12)
        else float(np.degrees(np.arctan2(-transverse[0], transverse[2])))
    )
    return {
        "elbow_flexion": round(float(np.degrees(elbow_angle)), 1),
        "shoulder_flexion": round(float(np.degrees(flexion)), 1),
        "shoulder_abduction": round(float(np.degrees(abduction)), 1),
        "shoulder_rotation_proxy": None if rotation is None else round(rotation, 1),
    }


def select_hand_near_wrist(hand_results, pose_wrist, max_distance: float = 0.25):
    """Select the detected hand that belongs to the highlighted pose wrist."""
    detected = getattr(hand_results, "multi_hand_landmarks", None) or []
    if not detected:
        return None
    distances = [
        ((hand.landmark[0].x - pose_wrist.x) ** 2 + (hand.landmark[0].y - pose_wrist.y) ** 2) ** 0.5
        for hand in detected
    ]
    index = min(range(len(distances)), key=distances.__getitem__)
    if distances[index] > max_distance:
        return None
    world_hands = getattr(hand_results, "multi_hand_world_landmarks", None) or []
    world = world_hands[index] if index < len(world_hands) else None
    return index, detected[index], world, distances[index]


def finger_angles_deg(landmarks, np) -> tuple[dict[str, float], dict[str, float]]:
    """Return joint flexion proxies and a mean curl value for each finger."""
    points = np.asarray([[point.x, point.y, point.z] for point in landmarks], dtype=float)

    def flexion(first: int, joint: int, last: int) -> float:
        incoming = points[first] - points[joint]
        outgoing = points[last] - points[joint]
        lengths = float(np.linalg.norm(incoming) * np.linalg.norm(outgoing))
        if lengths < 1e-9:
            return 0.0
        interior = np.degrees(np.arccos(np.clip(np.dot(incoming, outgoing) / lengths, -1.0, 1.0)))
        return round(float(180.0 - interior), 1)

    names = ("mcp", "pip", "dip")
    angles: dict[str, float] = {}
    curls: dict[str, float] = {}
    for finger, chain in FINGER_CHAINS.items():
        values = [flexion(chain[i - 1], chain[i], chain[i + 1]) for i in range(1, 4)]
        joint_names = ("cmc", "mcp", "ip") if finger == "thumb" else names
        angles.update(
            {f"{finger}_{name}": value for name, value in zip(joint_names, values, strict=False)}
        )
        curls[finger] = round(sum(values) / len(values), 1)
    return angles, curls


def display_pixel(x: int, y: int, width: int) -> tuple[int, int]:
    """Mirror one raw-camera point for display without changing saved data."""
    return width - 1 - x, y


def camera_xyz(point: list[float], intrinsics) -> list[float]:
    """Deproject depth into the same trunk frame as MediaPipe world points.

    OAK-D depth increases away from the camera, while the normalized world
    points above use positive Z towards the camera.  Keeping that sign
    consistent prevents enabling stereo depth from reversing shoulder
    flexion.
    """
    x, y, depth_m = point
    return [
        (x - intrinsics[0][2]) * depth_m / intrinsics[0][0],
        -(y - intrinsics[1][2]) * depth_m / intrinsics[1][1],
        -depth_m,
    ]
