# Christian, Majd
"""Check that camera and serial previews do not create recording sessions."""

from __future__ import annotations

import argparse
import io
import sys
import tempfile
import time
import types
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

import aurora.recording.record_oak_pose as record_oak_pose
import aurora.recording.record_serial_sensors as record_serial_sensors
from aurora.gui.sensor_graphs import LiveSignalGraphs
from aurora.gui.sensor_plot import draw_traces
from aurora.recording.ble_preview import imu_tilt_degrees, relative_angle
from aurora.recording.serial_preview import numeric_values
from aurora.sensors.ble_input import BleSensorReader
from aurora.sensors.ble_protocol import CURRENT_EMG_PACKET, HIGH_RATE_IMU_PACKET


class SensorPreviewTests(unittest.TestCase):
    def test_packet_totals_do_not_describe_sensor_availability(self):
        now = [100.0]
        reader = BleSensorReader(clock=lambda: now[0])
        for sequence in (1, 4):
            reader.notification("emg", CURRENT_EMG_PACKET.pack(0x0101, sequence, 1000, 2048))
        reader.notification("imu", HIGH_RATE_IMU_PACKET.pack(0x0103, 1, 1000, 1000, *([0] * 12)))
        graphs = LiveSignalGraphs.__new__(LiveSignalGraphs)
        graphs.window, graphs.status, graphs.zoom_emg, graphs.emg = Mock(), Mock(), Mock(), Mock()
        graphs.last_draw, graphs.imu = 0, {}
        graphs.show_activation = False
        with patch("aurora.gui.sensor_graphs.draw_traces"):
            graphs.update(reader)
            text = graphs.status.set.call_args.args[0]
            self.assertIn("IMUs 2/2 | EMG receiving", text)
            self.assertIn("missed packets (total): IMU 0, EMG 2", text)
            self.assertNotIn("grip activation", text)
            now[0] += 1
            graphs.last_draw = 0
            graphs.update(reader)
            text = graphs.status.set.call_args.args[0]
            self.assertIn("IMUs 0/2 | EMG waiting for samples", text)
            self.assertIn("missed packets (total): IMU 0, EMG 2", text)

    def test_raw_emg_graph_keeps_small_noise_small_on_the_full_adc_scale(self):
        canvas = Mock()
        canvas.winfo_width.return_value = 600
        canvas.winfo_height.return_value = 250
        draw_traces(canvas, [[0, 20, 72]], unit="ADC", sample_period=0.00025, limits=(0, 4095))
        points = canvas.create_line.call_args.args
        heights = points[1::2]
        self.assertLess(max(heights) - min(heights), 5)
        labels = [call.kwargs["text"] for call in canvas.create_text.call_args_list]
        self.assertIn("4095.00 ADC", labels)
        self.assertIn("0 ms of received samples", labels)

    def test_serial_preview_finds_numeric_values_in_json(self) -> None:
        self.assertEqual(
            numeric_values('{"emg": 42, "imu": {"x": -1.5}, "ready": true}'),
            {"emg": 42.0, "imu.x": -1.5},
        )
        self.assertEqual(numeric_values("3.25"), {"value": 3.25})
        self.assertEqual(numeric_values("device ready"), {})

    def test_imu_tilt_exposes_arm_rotation_without_integrating_acceleration(self) -> None:
        self.assertEqual(imu_tilt_degrees([0.0, 0.0, 1.0]), (0.0, 0.0))
        pitch, roll = imu_tilt_degrees([0.0, 1.0, 0.0])
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(roll, 90.0)
        pitch, roll = imu_tilt_degrees([1.0, 0.0, 0.0])
        self.assertAlmostEqual(pitch, -90.0)
        self.assertAlmostEqual(roll, 0.0)
        self.assertIsNone(imu_tilt_degrees([0.0, 0.0, 0.0]))
        self.assertAlmostEqual(relative_angle(-179.0, 179.0), 2.0)

    def test_left_arm_points_keep_camera_orientation_before_recording(self) -> None:
        def point(x, y):
            return types.SimpleNamespace(x=x, y=y, visibility=0.9)

        body = types.SimpleNamespace(
            landmark=[
                point(0.2, 0.2),
                point(0.3, 0.3),
                point(0.4, 0.4),
                point(0.7, 0.2),
                point(0.2, 0.7),
                point(0.7, 0.7),
            ]
        )
        pose = Mock()
        pose.process.return_value = types.SimpleNamespace(
            pose_landmarks=body, pose_world_landmarks=None
        )
        mp = types.SimpleNamespace(
            solutions=types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    PoseLandmark=types.SimpleNamespace(
                        LEFT_SHOULDER=0,
                        LEFT_ELBOW=1,
                        LEFT_WRIST=2,
                        RIGHT_SHOULDER=3,
                        LEFT_HIP=4,
                        RIGHT_HIP=5,
                    )
                ),
            )
        )
        cv2 = types.SimpleNamespace(
            COLOR_BGR2RGB=1,
            FONT_HERSHEY_SIMPLEX=0,
            cvtColor=Mock(side_effect=lambda frame, _: frame),
            line=Mock(),
            circle=Mock(),
            putText=Mock(),
        )
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[0, 0, 0] = 99
        rendered, status, sample, observation = record_oak_pose.analyze_frame(
            image, None, pose, mp, cv2, np, "left"
        )
        self.assertIn("Left arm tracked", status)
        self.assertIsNone(sample)
        self.assertEqual(len(observation["keypoints_2d"]), 6)
        self.assertEqual(rendered[0, -1, 0], 99)
        self.assertEqual(rendered[0, 0, 0], 0)
        self.assertTrue(observation["arm_visible"])
        self.assertEqual(cv2.line.call_count, 2)
        self.assertEqual(cv2.line.call_args_list[0].args[1], (511, 96))
        self.assertEqual(cv2.line.call_args_list[0].args[2], (447, 144))
        self.assertEqual(cv2.putText.call_count, 3)

        def world_point(x, y, z):
            return types.SimpleNamespace(x=x, y=y, z=z)

        pose.process.return_value.pose_world_landmarks = types.SimpleNamespace(
            landmark=[
                world_point(1, -1, 0),
                world_point(1, 0, 0),
                world_point(1, 0, -1),
                world_point(-1, -1, 0),
                world_point(1, 0, 0),
                world_point(-1, 0, 0),
            ]
        )
        _, _, _, estimated = record_oak_pose.analyze_frame(image, None, pose, mp, cv2, np, "left")
        self.assertEqual(estimated["angle_source"], "mediapipe_world_estimate")
        self.assertAlmostEqual(estimated["angles_deg"]["elbow_flexion"], 90)
        for index in (4, 5):
            body.landmark[index].visibility = 0.0
        _, status, _, hidden = record_oak_pose.analyze_frame(image, None, pose, mp, cv2, np, "left")
        self.assertTrue(hidden["arm_visible"])
        self.assertFalse(hidden["reference_visible"])
        self.assertAlmostEqual(hidden["angles_deg"]["elbow_flexion"], 90)
        self.assertNotIn("shoulder_flexion", hidden["angles_deg"])
        self.assertEqual(len(hidden["pose_landmarks"]), len(body.landmark))
        self.assertEqual(len(hidden["pose_world_landmarks"]), 6)
        self.assertIn("shoulder angles unavailable", status)
        _, _, hidden_sample, stereo_hidden = record_oak_pose.analyze_frame(
            image,
            np.full((480, 640), 1000, dtype=np.uint16),
            pose,
            mp,
            cv2,
            np,
            "left",
            [[600, 0, 320], [0, 600, 240], [0, 0, 1]],
        )
        self.assertEqual(stereo_hidden["angle_source"], "stereo")
        self.assertIn("elbow_flexion", hidden_sample["angles_deg"])
        self.assertNotIn("shoulder_flexion", stereo_hidden["angles_deg"])
        for index in (4, 5):
            body.landmark[index].visibility = 0.9
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
            "left_shoulder": [1, 1, 0],
            "right_shoulder": [-1, 1, 0],
            "left_hip": [1, 0, 0],
            "right_hip": [-1, 0, 0],
            "left_elbow": [1, 0, 0],
            "left_wrist": [1, 0, 1],
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

    def test_left_and_right_camera_tracking_share_forward_and_outward_axes(self) -> None:
        trunk = {
            "left_shoulder": [1, 1, 0],
            "right_shoulder": [-1, 1, 0],
            "left_hip": [1, 0, 0],
            "right_hip": [-1, 0, 0],
        }
        left_forward = {
            **trunk,
            "left_elbow": [1, 1, 1],
            "left_wrist": [1, 1, 2],
        }
        right_forward = {
            **trunk,
            "right_elbow": [-1, 1, 1],
            "right_wrist": [-1, 1, 2],
        }
        left = record_oak_pose.arm_angles_deg(left_forward, "left", np)
        right = record_oak_pose.arm_angles_deg(right_forward, "right", np)
        self.assertAlmostEqual(left["shoulder_flexion"], 90.0)
        self.assertAlmostEqual(right["shoulder_flexion"], 90.0)

        left_out = {
            **trunk,
            "left_elbow": [2, 1, 0],
            "left_wrist": [3, 1, 0],
        }
        right_out = {
            **trunk,
            "right_elbow": [-2, 1, 0],
            "right_wrist": [-3, 1, 0],
        }
        left = record_oak_pose.arm_angles_deg(left_out, "left", np)
        right = record_oak_pose.arm_angles_deg(right_out, "right", np)
        self.assertAlmostEqual(left["shoulder_abduction"], 90.0)
        self.assertAlmostEqual(right["shoulder_abduction"], 90.0)

    def test_left_axial_proxy_keeps_right_and_left_direction(self) -> None:
        trunk_and_upper_arm = {
            "left_shoulder": [1, 1, 0],
            "right_shoulder": [-1, 1, 0],
            "left_hip": [1, 0, 0],
            "right_hip": [-1, 0, 0],
            "left_elbow": [1, 0, 0],
        }
        hand_right = record_oak_pose.arm_angles_deg(
            {**trunk_and_upper_arm, "left_wrist": [0, 0, 0]},
            "left",
            np,
        )
        hand_left = record_oak_pose.arm_angles_deg(
            {**trunk_and_upper_arm, "left_wrist": [2, 0, 0]},
            "left",
            np,
        )
        self.assertGreater(hand_right["shoulder_rotation_proxy"], 0.0)
        self.assertLess(hand_left["shoulder_rotation_proxy"], 0.0)

    def test_forward_reach_does_not_turn_small_lateral_noise_into_abduction(self) -> None:
        points = {
            "left_shoulder": [1, 1, 0],
            "right_shoulder": [-1, 1, 0],
            "left_hip": [1, 0, 0],
            "right_hip": [-1, 0, 0],
            "left_elbow": [1.05, 1, 1],
            "left_wrist": [1.10, 1, 2],
        }
        angles = record_oak_pose.arm_angles_deg(points, "left", np)
        self.assertGreater(angles["shoulder_flexion"], 85.0)
        self.assertLess(angles["shoulder_abduction"], 5.0)

    def test_stereo_deprojection_uses_mediapipe_forward_axis(self) -> None:
        intrinsics = [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]]
        center = record_oak_pose.camera_xyz([50.0, 40.0, 1.25], intrinsics)
        self.assertEqual(center, [0.0, 0.0, -1.25])
        # A point closer to the camera has a larger (more positive) forward Z.
        closer = record_oak_pose.camera_xyz([50.0, 40.0, 0.75], intrinsics)
        self.assertGreater(closer[2], center[2])

    def test_hand_nearest_selected_wrist_is_saved_with_finger_angles(self) -> None:
        def pose_point(x, y):
            return types.SimpleNamespace(x=x, y=y, visibility=0.95)

        body = types.SimpleNamespace(
            landmark=[
                pose_point(0.2, 0.2),
                pose_point(0.3, 0.3),
                pose_point(0.4, 0.4),
                pose_point(0.7, 0.2),
                pose_point(0.2, 0.7),
                pose_point(0.7, 0.7),
            ]
        )
        pose = Mock()
        pose.process.return_value = types.SimpleNamespace(
            pose_landmarks=body,
            pose_world_landmarks=None,
        )

        def hand_point(x, y, z=0.0):
            return types.SimpleNamespace(x=x, y=y, z=z)

        def make_hand(wrist_x, wrist_y):
            points = [hand_point(wrist_x, wrist_y)]
            for index in range(1, 21):
                points.append(
                    hand_point(
                        wrist_x + (index % 4 + 1) * 0.012,
                        wrist_y - (index // 4 + 1) * 0.012,
                        index * 0.001,
                    )
                )
            return types.SimpleNamespace(landmark=points)

        far_hand = make_hand(0.8, 0.8)
        selected_hand = make_hand(0.41, 0.41)
        hands = Mock()
        hands.process.return_value = types.SimpleNamespace(
            multi_hand_landmarks=[far_hand, selected_hand],
            multi_hand_world_landmarks=[far_hand, selected_hand],
        )
        mp = types.SimpleNamespace(
            solutions=types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    PoseLandmark=types.SimpleNamespace(
                        LEFT_SHOULDER=0,
                        LEFT_ELBOW=1,
                        LEFT_WRIST=2,
                        RIGHT_SHOULDER=3,
                        LEFT_HIP=4,
                        RIGHT_HIP=5,
                    )
                ),
            )
        )
        cv2 = types.SimpleNamespace(
            COLOR_BGR2RGB=1,
            FONT_HERSHEY_SIMPLEX=0,
            cvtColor=Mock(side_effect=lambda frame, _: frame),
            line=Mock(),
            circle=Mock(),
            putText=Mock(),
        )
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        _, status, _, observation = record_oak_pose.analyze_frame(
            image,
            None,
            pose,
            mp,
            cv2,
            np,
            "left",
            hands=hands,
        )
        self.assertIn("hand/fingers tracked", status)
        self.assertTrue(observation["hand_visible"])
        self.assertEqual(len(observation["hand_keypoints_2d"]), 21)
        self.assertEqual(observation["hand_keypoints_2d"]["wrist"][:2], [0.41, 0.41])
        self.assertEqual(len(observation["finger_angles_deg"]), 15)
        self.assertEqual(
            set(observation["finger_curl_deg"]), {"thumb", "index", "middle", "ring", "pinky"}
        )

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
        mp = types.SimpleNamespace(
            solutions=types.SimpleNamespace(
                pose=types.SimpleNamespace(Pose=Mock(return_value=model)),
                hands=types.SimpleNamespace(Hands=Mock(return_value=hand_model)),
            )
        )
        cv2 = types.SimpleNamespace(
            FONT_HERSHEY_SIMPLEX=0,
            COLOR_BGR2RGB=1,
            WINDOW_NORMAL=0,
            EVENT_LBUTTONUP=4,
            putText=Mock(),
            rectangle=Mock(),
            circle=Mock(),
            cvtColor=Mock(side_effect=lambda frame, _: frame),
            imshow=Mock(),
            namedWindow=Mock(),
            setMouseCallback=Mock(),
            waitKey=Mock(return_value=ord("q")),
            destroyAllWindows=Mock(),
        )
        args = argparse.Namespace(
            preview=True, duration=0.0, subject="S01", side="left", depth=False, pose_model=1
        )
        cup_detector = Mock(name="test_cups", status="ready", model_sha256="test", threshold=0.25)
        cup_detector.detect.return_value = []
        with (
            patch.object(record_oak_pose, "parse_args", return_value=args),
            patch.object(record_oak_pose, "require_pose_model"),
            patch.object(record_oak_pose, "CupDetector", return_value=cup_detector),
            patch.object(
                record_oak_pose,
                "receive_frame",
                return_value=(np.zeros((480, 640, 3), dtype=np.uint8), 123),
            ),
            patch.object(record_oak_pose.CameraSession, "add_frame", autospec=True) as add_frame,
            patch.object(
                record_oak_pose, "build_pipeline", return_value=(pipeline, video_queue, depth_queue)
            ),
            patch(
                "aurora.recording.oak_session.create_session_directory",
                side_effect=AssertionError("recorded"),
            ),
            patch.dict(sys.modules, {"cv2": cv2, "depthai": Mock(), "mediapipe": mp}),
            redirect_stdout(io.StringIO()),
        ):
            self.assertEqual(record_oak_pose.main(), 0)
        pipeline.stop.assert_called_once()
        hand_model.close.assert_called_once()
        cup_detector.close.assert_called_once()
        cv2.imshow.assert_called_once()
        self.assertEqual(add_frame.call_args.kwargs["host_monotonic_ns"], 123)
        self.assertEqual(cv2.imshow.call_args.args[1].shape, (644, 640, 3))

    def test_camera_session_starts_only_after_record_pressed(self) -> None:
        writer = Mock()
        writer.isOpened.return_value = True
        cv2 = types.SimpleNamespace(
            VideoWriter=Mock(return_value=writer), VideoWriter_fourcc=Mock(return_value=123)
        )
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary) / "capture"
            session.mkdir()
            camera = record_oak_pose.CameraSession(cv2, "S01", "left")
            camera.add_frame(frame, None, {}, host_monotonic_ns=0)
            self.assertFalse(camera.recording)
            with patch(
                "aurora.recording.oak_session.create_session_directory", return_value=session
            ):
                camera.start()
                camera.add_frame(
                    frame,
                    {"shoulder": [1, 2, 0.5], "elbow": [3, 4, 0.5], "hand": []},
                    {"person_visible": True},
                    host_monotonic_ns=time.monotonic_ns(),
                )
                camera.stop()
            self.assertFalse(camera.recording)
            writer.write.assert_called()
            self.assertEqual((session / "pose.json").is_file(), True)
            self.assertEqual((session / "meta.json").is_file(), True)
            pose = __import__("json").loads((session / "pose.json").read_text())
            metadata = __import__("json").loads((session / "meta.json").read_text())
            self.assertEqual(pose["side"], "left")
            self.assertIsInstance(pose["data"][0]["host_monotonic_ns"], int)
            self.assertEqual(
                pose["data"][0]["host_monotonic_ns"],
                pose["observations"][0]["host_monotonic_ns"],
            )
            self.assertEqual(metadata["clock_domain"], "host_monotonic_ns")
            self.assertTrue(metadata["video_mirrored"])

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

        args = argparse.Namespace(
            preview=True, port="COM5", baud=115200, duration=0.0, subject="S01"
        )
        output = io.StringIO()
        serial = types.SimpleNamespace(Serial=FakeSerial, SerialException=Exception)
        with (
            patch.object(record_serial_sensors, "parse_args", return_value=args),
            patch.object(
                record_serial_sensors,
                "create_session_directory",
                side_effect=AssertionError("recorded"),
            ),
            patch.dict(sys.modules, {"serial": serial}),
            redirect_stdout(output),
        ):
            self.assertEqual(record_serial_sensors.main(), 0)
        self.assertIn('"emg": 42', output.getvalue())


if __name__ == "__main__":
    unittest.main()
