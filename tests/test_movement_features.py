# Majd
"""Check coordinate units and compatibility of new and inherited profiles."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, Mock, patch

import numpy as np

from aurora.ml.features import load_sequence
from aurora.recording import capture_movement


def measured_frame():
    return {
        "shoulder": [320, 160, 1.2],
        "elbow": [330, 260, 1.1],
        "hand": [{"id": 0, "x": 340, "y": 340, "depth_m": 1.0}],
    }


class MovementFeatureTests(unittest.TestCase):
    def test_new_and_legacy_profiles_have_identical_features(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            frames = [measured_frame(), measured_frame()]
            frames[1]["elbow"][0] += 12
            legacy = root / "legacy.json"
            legacy.write_text(json.dumps({"user_id": "S01", "sequence": 2, "data": frames}))
            original = legacy.read_bytes()
            new = capture_movement.save_capture(root / "S01", "S01", "left", frames)
            old_features, old_subject = load_sequence(legacy)
            new_features, new_subject = load_sequence(new)
            np.testing.assert_array_equal(new_features, old_features)
            self.assertEqual(new_features.shape, (60, 69))
            self.assertEqual(old_subject, new_subject)
            self.assertEqual(legacy.read_bytes(), original)

    def test_trial_numbers_continue_per_subject_and_frame_count_is_not_trial_number(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for subject in ("S01", "S02"):
                for number in (1, 2):
                    path = capture_movement.save_capture(
                        root / subject, subject, "left", [measured_frame()] * 3
                    )
                    payload = json.loads(path.read_text())
                    self.assertEqual(payload["trial_id"], f"T{number:03d}")
                    self.assertEqual(payload["pose_frame_count"], 3)
                    self.assertEqual(payload["subject_id"], subject)
                    self.assertNotIn("sequence", payload)
                    self.assertNotIn("user_id", payload)

    def test_relative_or_undeclared_new_coordinates_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "profile.json"
            cases = [
                {"coordinate_system": "mediapipe_relative"},
                {"coordinate_system": "camera_xyz_metres"},
                {"format_version": 2},
                {"frame_count": 2},
                {"pose_frame_count": 2},
                {"format_version": 3, "coordinate_system": "rgb_pixels_stereo_depth"},
            ]
            for extra in cases:
                with self.subTest(metadata=extra):
                    path.write_text(
                        json.dumps({"subject_id": "S01", "data": [measured_frame()], **extra})
                    )
                    with self.assertRaises(ValueError):
                        load_sequence(path)
            frame = measured_frame()
            frame["hand"][0]["z_relative"] = frame["hand"][0].pop("depth_m")
            path.write_text(json.dumps({"subject_id": "S01", "data": [frame]}))
            with self.assertRaisesRegex(ValueError, "Relative MediaPipe"):
                load_sequence(path)

    def test_capture_uses_stereo_points_and_closes_camera_and_models(self):
        with tempfile.TemporaryDirectory() as temporary:
            cv2 = Mock()
            cv2.waitKey.side_effect = [32, 32]
            cv2.getWindowProperty.return_value = 1
            mp = MagicMock()
            pipeline, video, depth = Mock(), Mock(), Mock()
            frame = measured_frame()
            with (
                patch.dict("sys.modules", {"cv2": cv2, "mediapipe": mp, "depthai": Mock()}),
                patch.object(capture_movement, "configure_depthai_runtime"),
                patch.object(
                    capture_movement, "build_pipeline", return_value=(pipeline, video, depth)
                ) as build,
                patch.object(capture_movement, "ArmHandTracker") as tracker,
                patch.object(
                    capture_movement,
                    "analyze_frame",
                    return_value=(Mock(), "", frame, {"hand_visible": True}),
                ),
            ):
                capture_movement.main(["--subject-id", "S01", "--output", temporary])
            self.assertTrue(build.call_args.kwargs["use_depth"])
            saved = list((Path(temporary) / "S01").glob("*.json"))
            self.assertEqual(len(saved), 1)
            payload = json.loads(saved[0].read_text())
            self.assertEqual(payload["coordinate_system"], "rgb_pixels_stereo_depth")
            self.assertEqual(payload["data"][0]["hand"], frame["hand"])
            pipeline.stop.assert_called_once()
            tracker.return_value.close.assert_called_once()
            cv2.destroyAllWindows.assert_called_once()


if __name__ == "__main__":
    unittest.main()
