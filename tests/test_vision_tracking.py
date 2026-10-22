# Majd
"""Check cup coordinates, occlusion handling and wrist-crop landmarks."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np

from aurora.sensors.camera.cup_detection import (
    CupDetector,
    add_cup_tracking,
    decode_cups,
    prepare_frame,
)
from aurora.sensors.camera.hand_tracking import ArmHandTracker, restore_hand_coordinates, wrist_crop
from aurora.sensors.camera.oak_tracking import analyze_frame


class VisionTrackingTests(unittest.TestCase):
    def test_letterbox_preserves_colour_and_aspect_ratio(self):
        frame = np.full((100, 200, 3), (10, 20, 30), dtype=np.uint8)
        tensor, scale, padding = prepare_frame(frame, (640, 640), cv2, np)
        self.assertEqual((scale, padding), (3.2, (0, 160)))
        self.assertEqual(tensor.shape, (1, 3, 640, 640))
        self.assertEqual(tensor.dtype, np.float32)
        np.testing.assert_allclose(tensor[0, :, 320, 320], np.array([30, 20, 10]) / 255)
        np.testing.assert_allclose(tensor[0, :, 0, 0], 114 / 255)

    def test_cup_boxes_return_to_raw_pixels_and_duplicates_are_removed(self):
        rows = np.zeros((4, 84), dtype=np.float32)
        rows[:, :4] = (320, 320, 128, 160)
        rows[0, 45] = 0.9
        rows[1, 45] = 0.8
        rows[1, 0] += 2
        rows[2, 4] = 0.99  # A person is not a cup.
        rows[3, 45] = 0.1
        detections = decode_cups(rows.T[None], (200, 100), 3.2, (0, 160), 0.25, cv2, np)
        self.assertEqual(
            detections,
            [{"label": "cup", "confidence": 0.9, "bbox_xyxy": [80.0, 25.0, 120.0, 75.0]}],
        )

    def test_invalid_model_boxes_are_not_saved(self):
        rows = np.zeros((3, 84), dtype=np.float32)
        rows[:, 45] = 0.9
        rows[:, :4] = (320, 320, 128, 160)
        rows[0, 0] = np.nan
        rows[1, 2] = -10
        rows[2, 45] = np.inf
        self.assertEqual(decode_cups(rows.T[None], (640, 480), 1, (0, 80), 0.25, cv2, np), [])

    def test_saved_boxes_stay_raw_while_preview_is_mirrored(self):
        detector = SimpleNamespace(
            detect=lambda frame: [
                {"label": "cup", "confidence": 0.8, "bbox_xyxy": [10, 20, 30, 50]}
            ],
            name="test",
            status="ready",
            model_sha256="abc",
            threshold=0.25,
        )
        drawing = Mock()
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        observation = {}
        add_cup_tracking(frame, frame.copy(), observation, detector, drawing)
        self.assertEqual(observation["cup_detections"][0]["bbox_xyxy"], [10, 20, 30, 50])
        self.assertEqual(drawing.rectangle.call_args.args[1:3], ((69, 20), (89, 50)))
        self.assertTrue(observation["cup_visible"])

    def test_cup_detection_runs_without_a_visible_person(self):
        frame = np.zeros((80, 100, 3), dtype=np.uint8)
        detector = SimpleNamespace(
            detect=Mock(
                return_value=[{"label": "cup", "confidence": 0.8, "bbox_xyxy": [10, 20, 30, 50]}]
            ),
            name="test",
            status="ready",
            model_sha256="abc",
            threshold=0.25,
        )
        pose = SimpleNamespace(process=lambda rgb: SimpleNamespace(pose_landmarks=None))
        _, _, sample, observation = analyze_frame(
            frame, None, pose, None, cv2, np, "left", cup_detector=detector
        )
        detector.detect.assert_called_once()
        self.assertFalse(observation["person_visible"])
        self.assertTrue(observation["cup_visible"])
        self.assertIsNone(sample)
        self.assertNotIn("finger_angles_deg", observation)

    def test_missing_model_is_reported_without_fake_detections(self):
        with tempfile.TemporaryDirectory() as temporary:
            detector = CupDetector(cv2, np, Path(temporary) / "missing.onnx")
            self.assertEqual(detector.status, "model missing")
            self.assertEqual(detector.detect(np.zeros((80, 100, 3), dtype=np.uint8)), [])
            detector.close()

    def test_wrist_crop_stays_in_frame_at_edges(self):
        for x, y in ((-0.1, 0.5), (1.1, 0.5), (0.5, -0.1), (0.5, 1.1), (0.5, 0.5)):
            wrist = SimpleNamespace(x=x, y=y)
            elbow = SimpleNamespace(x=0.5, y=0.5)
            x1, y1, x2, y2 = wrist_crop(640, 480, wrist, elbow)
            self.assertTrue(0 <= x1 < x2 <= 640)
            self.assertTrue(0 <= y1 < y2 <= 480)
            self.assertEqual(x2 - x1, y2 - y1)

    def test_crop_landmarks_map_back_without_changing_world_estimates(self):
        point = SimpleNamespace(x=0.25, y=0.75, z=-0.2)
        world, handedness = object(), object()
        result = SimpleNamespace(
            multi_hand_landmarks=[SimpleNamespace(landmark=[point])],
            multi_hand_world_landmarks=world,
            multi_handedness=handedness,
        )
        restored = restore_hand_coordinates(result, (100, 50, 300, 250), 640, 480)
        mapped = restored.multi_hand_landmarks[0].landmark[0]
        self.assertAlmostEqual(mapped.x, 150 / 640)
        self.assertAlmostEqual(mapped.y, 200 / 480)
        self.assertAlmostEqual(mapped.z, -40 / 640)
        self.assertIs(restored.multi_hand_world_landmarks, world)
        self.assertIs(restored.multi_handedness, handedness)
        self.assertEqual(point.x, 0.25)

    def test_obscured_hand_does_not_reuse_previous_landmarks(self):
        model = Mock()
        point = SimpleNamespace(x=0.5, y=0.5, z=0)
        model.process.side_effect = [
            SimpleNamespace(
                multi_hand_landmarks=[SimpleNamespace(landmark=[point])],
                multi_hand_world_landmarks=None,
                multi_handedness=None,
            ),
            SimpleNamespace(
                multi_hand_landmarks=None, multi_hand_world_landmarks=None, multi_handedness=None
            ),
        ]
        mp = SimpleNamespace(
            solutions=SimpleNamespace(hands=SimpleNamespace(Hands=Mock(return_value=model)))
        )
        tracker = ArmHandTracker(mp, cv2)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        wrist, elbow = SimpleNamespace(x=0.5, y=0.5), SimpleNamespace(x=0.4, y=0.4)
        first = tracker.process_for_arm(frame, wrist, elbow)
        second = tracker.process_for_arm(frame, wrist, elbow)
        self.assertEqual(len(first.multi_hand_landmarks), 1)
        self.assertEqual(second.multi_hand_landmarks, [])
        tracker.close()
        model.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
