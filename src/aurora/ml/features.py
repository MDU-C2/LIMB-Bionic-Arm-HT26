# Majd
"""Shared preprocessing for the inherited camera movement GRU."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

SEQUENCE_LENGTH = 60
FEATURE_COUNT = 69
COORDINATE_SYSTEM = "rgb_pixels_stereo_depth"
PREPROCESSING = {
    "coordinate_system": COORDINATE_SYSTEM,
    "frames": SEQUENCE_LENGTH,
    "features": FEATURE_COUNT,
    "resampling": "linear_by_frame_index",
    "centering": "subtract_shoulder",
    "missing_hand_points": "zero_padding",
}


def _point3(value: object, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain x pixels, y pixels and depth in metres")
    try:
        point = np.asarray(value, dtype=np.float32)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain numeric coordinates") from error
    if not np.all(np.isfinite(point)):
        raise ValueError(f"{name} contains a missing or invalid value")
    if point[2] < 0:
        raise ValueError(f"{name} has negative stereo depth")
    return point.tolist()


def frame_to_vector(frame: object) -> np.ndarray:
    """Keep the original 69 features; missing hand points retain zero padding."""
    if not isinstance(frame, dict):
        raise ValueError("Each frame must be a JSON object")
    values = _point3(frame.get("shoulder"), "shoulder")
    values.extend(_point3(frame.get("elbow"), "elbow"))
    hand = frame.get("hand", [])
    if not isinstance(hand, list):
        raise ValueError("hand must be a list")
    hand_by_id = {}
    for point in hand:
        if not isinstance(point, dict) or "id" not in point:
            raise ValueError("Each hand point must have a landmark id")
        if "z_relative" in point:
            raise ValueError("Relative MediaPipe z cannot be used as stereo depth")
        try:
            landmark_id = int(point["id"])
        except (TypeError, ValueError) as error:
            raise ValueError("Hand landmark ids must be integers") from error
        if landmark_id in hand_by_id or not 0 <= landmark_id < 21:
            raise ValueError("Hand landmark ids must be unique and between 0 and 20")
        hand_by_id[landmark_id] = point
    for landmark_id in range(21):
        point = hand_by_id.get(landmark_id)
        if point is None:
            values.extend((0.0, 0.0, 0.0))
        else:
            values.extend(
                _point3(
                    [point.get("x"), point.get("y"), point.get("depth_m")],
                    f"hand landmark {landmark_id}",
                )
            )
    return np.asarray(values, dtype=np.float32)


def load_movement(path: Path) -> tuple[np.ndarray, str]:
    """Read new metadata or the unchanged LIMB1 user_id/sequence format."""
    path = Path(path)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"Could not read {path}: {error}") from error
    if not isinstance(raw, dict) or not isinstance(raw.get("data"), list):
        raise ValueError(f"{path} does not contain a recording data list")
    if raw.get("coordinate_system", COORDINATE_SYSTEM) != COORDINATE_SYSTEM:
        raise ValueError("This GRU requires RGB pixels and measured stereo depth")
    version = raw.get("format_version", 1)
    if version not in (1, 2, 3):
        raise ValueError("Unsupported movement file format_version")
    if version >= 2 and "coordinate_system" not in raw:
        raise ValueError("New movement files must declare their coordinate system")
    if not raw["data"]:
        raise ValueError(f"{path} contains no frames")
    if version >= 3 and "pose_frame_count" not in raw:
        raise ValueError("New movement files must include pose_frame_count")
    for field in ("pose_frame_count", "frame_count"):
        if field in raw and raw[field] != len(raw["data"]):
            raise ValueError(f"{field} does not match the saved data")
    subject_id = raw.get("subject_id", raw.get("user_id", raw.get("subject")))
    if subject_id is None or not str(subject_id).strip():
        raise ValueError("Recording must include a subject_id (legacy: user_id)")
    return np.stack([frame_to_vector(frame) for frame in raw["data"]]), str(subject_id)


def preprocess(sequence: np.ndarray) -> np.ndarray:
    """Resample to 60 frames and center on the shoulder, as in LIMB1 training."""
    source_time = np.arange(len(sequence), dtype=float)
    target_time = np.linspace(0, len(sequence) - 1, SEQUENCE_LENGTH)
    resampled = np.stack(
        [
            np.interp(target_time, source_time, sequence[:, feature])
            for feature in range(FEATURE_COUNT)
        ],
        axis=1,
    )
    points = resampled.reshape(SEQUENCE_LENGTH, FEATURE_COUNT // 3, 3)
    return (points - points[:, :1, :]).reshape(SEQUENCE_LENGTH, FEATURE_COUNT).astype(np.float32)


def load_sequence(path: Path) -> tuple[np.ndarray, str]:
    sequence, subject_id = load_movement(path)
    return preprocess(sequence), subject_id
