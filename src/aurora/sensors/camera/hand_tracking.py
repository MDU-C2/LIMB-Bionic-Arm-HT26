# Majd
"""Track a hand in the area around the selected arm's wrist."""

from __future__ import annotations

import math
from types import SimpleNamespace


def wrist_crop(width: int, height: int, wrist, elbow) -> tuple[int, int, int, int]:
    length = math.hypot((wrist.x - elbow.x) * width, (wrist.y - elbow.y) * height)
    size = round(min(width, height, max(min(width, height) * 0.28, length * 2.6)))
    size = max(1, size)
    x = max(0, min(width - size, round(wrist.x * width - size / 2)))
    y = max(0, min(height - size, round(wrist.y * height - size / 2)))
    return x, y, x + size, y + size


def restore_hand_coordinates(result, box, width: int, height: int):
    """Return landmarks in the original camera coordinates."""
    x1, y1, x2, y2 = box
    hands = []
    for hand in result.multi_hand_landmarks or []:
        hands.append(
            SimpleNamespace(
                landmark=[
                    SimpleNamespace(
                        x=(x1 + point.x * (x2 - x1)) / width,
                        y=(y1 + point.y * (y2 - y1)) / height,
                        z=point.z * (x2 - x1) / width,
                    )
                    for point in hand.landmark
                ]
            )
        )
    return SimpleNamespace(
        multi_hand_landmarks=hands,
        multi_hand_world_landmarks=result.multi_hand_world_landmarks,
        multi_handedness=result.multi_handedness,
    )


class ArmHandTracker:
    def __init__(self, mp, cv2):
        self.cv2 = cv2
        self.roi = None
        self.model = mp.solutions.hands.Hands(
            static_image_mode=True,
            max_num_hands=2,
            model_complexity=1,
            min_detection_confidence=0.4,
            min_tracking_confidence=0.5,
        )

    def process_for_arm(self, rgb, wrist, elbow):
        height, width = rgb.shape[:2]
        self.roi = wrist_crop(width, height, wrist, elbow)
        x1, y1, x2, y2 = self.roi
        crop = self.cv2.resize(rgb[y1:y2, x1:x2], (384, 384))
        result = self.model.process(crop)
        return restore_hand_coordinates(result, self.roi, width, height)

    def close(self) -> None:
        self.model.close()
