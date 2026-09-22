"""Check that camera and serial previews do not create recording sessions."""

from __future__ import annotations

import argparse
from contextlib import redirect_stdout
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "recording"))

import record_oak_pose
import record_serial_sensors


class SensorPreviewTests(unittest.TestCase):
    def test_left_arm_points_keep_camera_orientation_before_recording(self) -> None:
        point = lambda x, y: types.SimpleNamespace(x=x, y=y, visibility=0.9)
        body = types.SimpleNamespace(landmark=[
            point(0.2, 0.2), point(0.3, 0.3), point(0.4, 0.4),
            point(0.7, 0.2), point(0.2, 0.7), point(0.7, 0.7),
        ])
        pose = Mock()
        pose.process.return_value = types.SimpleNamespace(
            pose_landmarks=body, pose_world_landmarks=None
        )
        mp = types.SimpleNamespace(solutions=types.SimpleNamespace(
            pose=types.SimpleNamespace(
                PoseLandmark=types.SimpleNamespace(LEFT_SHOULDER=0, LEFT_ELBOW=1,
                                                   LEFT_WRIST=2, RIGHT_SHOULDER=3,
                                                   LEFT_HIP=4, RIGHT_HIP=5)
            ),
        ))
        cv2 = types.SimpleNamespace(COLOR_BGR2RGB=1, FONT_HERSHEY_SIMPLEX=0,
                                    cvtColor=Mock(side_effect=lambda frame, _: frame),
                                    line=Mock(), circle=Mock(), putText=Mock())
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[0, 0, 0] = 99
        rendered, status, sample, observation = record_oak_pose.analyze_frame(
            image, None, pose, mp, cv2, np, "left"
        )
        self.assertIn("Left arm tracked", status)
        self.assertIsNone(sample)
        self.assertEqual(len(observation["keypoints_2d"]), 6)
        self.assertEqual(rendered[0, 0, 0], 99)
        self.assertTrue(observation["arm_visible"])
        self.assertEqual(cv2.line.call_count, 2)
        self.assertEqual(cv2.line.call_args_list[0].args[1], (128, 96))
        self.assertEqual(cv2.line.call_args_list[0].args[2], (192, 144))
        self.assertEqual(cv2.putText.call_count, 3)
        world_point = lambda x, y, z: types.SimpleNamespace(x=x, y=y, z=z)
        pose.process.return_value.pose_world_landmarks = types.SimpleNamespace(
            landmark=[
                world_point(1, -1, 0), world_point(1, 0, 0), world_point(1, 0, -1),
                world_point(-1, -1, 0), world_point(1, 0, 0), world_point(-1, 0, 0),
            ]
        )
        _, _, _, estimated = record_oak_pose.analyze_frame(
            image, None, pose, mp, cv2, np, "left"
        )
        self.assertEqual(estimated["angle_source"], "mediapipe_world_estimate")
        self.assertAlmostEqual(estimated["angles_deg"]["elbow_flexion"], 90)
        intrinsics = [[600, 0, 320], [0, 600, 240], [0, 0, 1]]
        depth = np.full((480, 640), 1000, dtype=np.uint16)
        _, _, sample, measured = record_oak_pose.analyze_frame(
            image, depth, pose, mp, cv2, np, "left", intrinsics
        )
        self.assertEqual(measured["angle_source"], "stereo")
        self.assertEqual(len(sample["keypoints_camera_m"]), 6)
        self.assertIn("elbow_flexion", sample["angles_deg"])

    def test_oscar_six_point_angles_use_trunk_frame(self) -> None:
        points = {
            "left_shoulder": [1, 1, 0], "right_shoulder": [-1, 1, 0],
            "left_hip": [1, 0, 0], "right_hip": [-1, 0, 0],
            "left_elbow": [1, 0, 0], "left_wrist": [1, 0, 1],
        }
        angles = record_oak_pose.arm_angles_deg(points, "left", np)
        self.assertAlmostEqual(angles["elbow_flexion"], 90)
        self.assertAlmostEqual(angles["shoulder_flexion"], 0)
        self.assertAlmostEqual(angles["shoulder_abduction"], 0)
        self.assertAlmostEqual(angles["shoulder_rotation_proxy"], 0)
        points["left_wrist"] = [1, -1, 0]
        straight = record_oak_pose.arm_angles_deg(points, "left", np)
        self.assertAlmostEqual(straight["elbow_flexion"], 0)
        self.assertIsNone(straight["shoulder_rotation_proxy"])

    def test_hand_nearest_selected_wrist_is_saved_with_finger_angles(self) -> None:
        pose_point = lambda x, y: types.SimpleNamespace(x=x, y=y, visibility=0.95)
        body = types.SimpleNamespace(landmark=[
            pose_point(0.2, 0.2), pose_point(0.3, 0.3), pose_point(0.4, 0.4),
            pose_point(0.7, 0.2), pose_point(0.2, 0.7), pose_point(0.7, 0.7),
        ])
        pose = Mock()
        pose.process.return_value = types.SimpleNamespace(
            pose_landmarks=body, pose_world_landmarks=None,
        )
        hand_point = lambda x, y, z=0.0: types.SimpleNamespace(x=x, y=y, z=z)

        def make_hand(wrist_x, wrist_y):
            points = [hand_point(wrist_x, wrist_y)]
            for index in range(1, 21):
                points.append(hand_point(
                    wrist_x + (index % 4 + 1) * 0.012,
                    wrist_y - (index // 4 + 1) * 0.012,
                    index * 0.001,
                ))
            return types.SimpleNamespace(landmark=points)

        far_hand = make_hand(0.8, 0.8)
        selected_hand = make_hand(0.41, 0.41)
        hands = Mock()
        hands.process.return_value = types.SimpleNamespace(
            multi_hand_landmarks=[far_hand, selected_hand],
            multi_hand_world_landmarks=[far_hand, selected_hand],
        )
        mp = types.SimpleNamespace(solutions=types.SimpleNamespace(
            pose=types.SimpleNamespace(PoseLandmark=types.SimpleNamespace(
                LEFT_SHOULDER=0, LEFT_ELBOW=1, LEFT_WRIST=2,
                RIGHT_SHOULDER=3, LEFT_HIP=4, RIGHT_HIP=5,
            )),
        ))
        cv2 = types.SimpleNamespace(
            COLOR_BGR2RGB=1, FONT_HERSHEY_SIMPLEX=0,
            cvtColor=Mock(side_effect=lambda frame, _: frame),
            line=Mock(), circle=Mock(), putText=Mock(),
        )
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        _, status, _, observation = record_oak_pose.analyze_frame(
            image, None, pose, mp, cv2, np, "left", hands=hands,
        )
        self.assertIn("hand/fingers tracked", status)
        self.assertTrue(observation["hand_visible"])
        self.assertEqual(len(observation["hand_keypoints_2d"]), 21)
        self.assertEqual(observation["hand_keypoints_2d"]["wrist"][:2], [0.41, 0.41])
        self.assertEqual(len(observation["finger_angles_deg"]), 15)
        self.assertEqual(set(observation["finger_curl_deg"]),
                         {"thumb", "index", "middle", "ring", "pinky"})

    def test_oak_live_tracking_closes_without_creating_session(self) -> None:
        pipeline = Mock()
        pipeline.isRunning.return_value = True
        video_queue = Mock()
        video_queue.tryGet.return_value.getCvFrame.return_value = np.zeros(
            (480, 640, 3), dtype=np.uint8
        )
        depth_queue = Mock()
        depth_queue.tryGet.return_value = None
        model = Mock()
        model.process.return_value = types.SimpleNamespace(pose_landmarks=None)
        model.close = Mock()
        hand_model = Mock()
        hand_model.close = Mock()
        mp = types.SimpleNamespace(solutions=types.SimpleNamespace(
            pose=types.SimpleNamespace(Pose=Mock(return_value=model)),
            hands=types.SimpleNamespace(Hands=Mock(return_value=hand_model)),
        ))
        cv2 = types.SimpleNamespace(
            FONT_HERSHEY_SIMPLEX=0, COLOR_BGR2RGB=1, WINDOW_NORMAL=0,
            EVENT_LBUTTONUP=4,
            putText=Mock(), rectangle=Mock(), circle=Mock(),
            cvtColor=Mock(side_effect=lambda frame, _: frame),
            imshow=Mock(), namedWindow=Mock(), setMouseCallback=Mock(),
            waitKey=Mock(return_value=ord("q")), destroyAllWindows=Mock(),
        )
        args = argparse.Namespace(preview=True, duration=0.0, subject="S01", side="left",
                                  depth=False, pose_model=1)
        with patch.object(record_oak_pose, "parse_args", return_value=args), \
             patch.object(record_oak_pose, "build_pipeline", return_value=(pipeline, video_queue, depth_queue)), \
             patch.object(record_oak_pose, "create_session_directory", side_effect=AssertionError("recorded")), \
             patch.dict(sys.modules, {"cv2": cv2, "depthai": Mock(), "mediapipe": mp}), \
             redirect_stdout(io.StringIO()):
            self.assertEqual(record_oak_pose.main(), 0)
        pipeline.stop.assert_called_once()
        hand_model.close.assert_called_once()
        cv2.imshow.assert_called_once()
        self.assertEqual(cv2.imshow.call_args.args[1].shape, (644, 640, 3))

    def test_camera_session_starts_only_after_record_pressed(self) -> None:
        writer = Mock()
        writer.isOpened.return_value = True
        cv2 = types.SimpleNamespace(VideoWriter=Mock(return_value=writer),
                                    VideoWriter_fourcc=Mock(return_value=123))
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary) / "capture"
            session.mkdir()
            camera = record_oak_pose.CameraSession(cv2, "S01", "left")
            camera.add_frame(frame, None, {})
            self.assertFalse(camera.recording)
            with patch.object(record_oak_pose, "create_session_directory", return_value=session):
                camera.start()
                camera.add_frame(frame, {"shoulder": [1, 2, 0.5],
                                         "elbow": [3, 4, 0.5], "hand": []},
                                 {"person_visible": True})
                camera.stop()
            self.assertFalse(camera.recording)
            writer.write.assert_called()
            self.assertEqual((session / "pose.json").is_file(), True)
            self.assertEqual((session / "meta.json").is_file(), True)
            self.assertEqual(__import__("json").loads((session / "pose.json").read_text())["side"], "left")
            self.assertFalse(__import__("json").loads((session / "meta.json").read_text())["video_mirrored"])

    def test_serial_preview_prints_without_creating_session(self) -> None:
        class FakeSerial:
            def __init__(self, *_args, **_kwargs):
                self.calls = 0

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return None

            def readline(self):
                self.calls += 1
                if self.calls > 1:
                    raise KeyboardInterrupt
                return b'{"emg": 42}\n'

        args = argparse.Namespace(preview=True, port="COM5", baud=115200, duration=0.0, subject="S01")
        output = io.StringIO()
        serial = types.SimpleNamespace(Serial=FakeSerial, SerialException=Exception)
        with patch.object(record_serial_sensors, "parse_args", return_value=args), \
             patch.object(record_serial_sensors, "create_session_directory", side_effect=AssertionError("recorded")), \
             patch.dict(sys.modules, {"serial": serial}), redirect_stdout(output):
            self.assertEqual(record_serial_sensors.main(), 0)
        self.assertIn('"emg": 42', output.getvalue())


if __name__ == "__main__":
    unittest.main()
