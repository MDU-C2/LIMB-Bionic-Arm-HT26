# Majd
"""Check that tracking time does not move saved frame timestamps."""

from __future__ import annotations

import json
import tempfile
import threading
import unittest
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from aurora.recording.cup_input import CupCameraFrame, CupDepthFrame, CupSensorInput
from aurora.recording.oak_session import CameraSession
from aurora.sensors.camera.oak_camera import (
    FRAME_TIMESTAMP_REFERENCE,
    receive_frame,
    require_pose_model,
)
from aurora.sensors.live_input import LiveSensorInput


class CameraTimingTests(unittest.TestCase):
    def test_tracking_waits_for_delayed_depth_and_keeps_both_receipt_times(self):
        live = CupSensorInput(
            side="left", device="test", shoulder_twist_axis="y", wrist_twist_axis="y"
        )
        first = CupCameraFrame(1, 1_000_000_000, object(), device_time_ns=100_000_000)
        newer = CupCameraFrame(2, 1_040_000_000, object(), device_time_ns=140_000_000)
        with patch("aurora.recording.cup_input.time.monotonic_ns", return_value=1_050_000_000):
            live._publish_frame(first)
            live._publish_frame(newer)
            self.assertEqual(live.next_tracking_frame(0), (None, None))
            depth = CupDepthFrame(10, 1_055_000_000, 101_000_000, object())
            live._publish_depth(depth)
            frame, matched = live.next_tracking_frame(0)
            self.assertIs(frame, first)
            self.assertIs(matched, depth)
            self.assertIs(live.camera_frame, newer)
            self.assertEqual(frame.frame_time_ns, 1_000_000_000)
            self.assertEqual(matched.frame_time_ns, 1_055_000_000)
            self.assertEqual(live.next_tracking_frame(1), (None, None))
        self.assertEqual(live.drain_frames(), [first, newer])

    def test_missing_depth_still_allows_rgb_tracking_after_a_short_wait(self):
        live = CupSensorInput(
            side="left", device="test", shoulder_twist_axis="y", wrist_twist_axis="y"
        )
        first = CupCameraFrame(1, 1_000_000_000, object(), device_time_ns=100_000_000)
        live._publish_frame(first)
        live._publish_frame(CupCameraFrame(2, 1_240_000_000, object(), device_time_ns=340_000_000))
        with patch("aurora.recording.cup_input.time.monotonic_ns", return_value=1_250_000_000):
            frame, depth = live.next_tracking_frame(0)
        self.assertIs(frame, first)
        self.assertIsNone(depth)

    def test_stereo_readiness_rejects_zero_depth_missing_intrinsics_and_old_tracking(self):
        import time

        import numpy as np

        now = time.monotonic_ns()
        live = CupSensorInput(
            side="left", device="test", shoulder_twist_axis="y", wrist_twist_axis="y"
        )
        points = {
            name: [0.1, 0.2, -1.5]
            for name in ("left_shoulder", "right_shoulder", "left_elbow", "left_wrist")
        }
        live.tracking_frame = CupCameraFrame(
            1,
            now,
            object(),
            camera_sample={"keypoints_camera_m": points},
            camera_observation={"depth_time_difference_ns": 0},
            device_time_ns=100,
        )
        live._publish_depth(CupDepthFrame(1, now, 100, np.zeros((4, 4), dtype=np.uint16)))
        live.intrinsics = [[500, 0, 320], [0, 500, 240], [0, 0, 1]]
        self.assertTrue(live.stereo_status()["receiving"])
        self.assertFalse(live.stereo_status()["ready"])
        live._publish_depth(CupDepthFrame(2, now, 100, np.full((4, 4), 1500, dtype=np.uint16)))
        self.assertTrue(live.stereo_status()["ready"])
        live.intrinsics = None
        self.assertFalse(live.stereo_status()["ready"])
        live.intrinsics = [[500, 0, 320], [0, 500, 240], [0, 0, 1]]
        live.tracking_frame = CupCameraFrame(
            1,
            now - 2_000_000_000,
            object(),
            camera_sample={"keypoints_camera_m": points},
            camera_observation={"depth_time_difference_ns": 0},
            device_time_ns=100,
        )
        self.assertFalse(live.stereo_status()["ready"])

    def test_depth_matching_uses_device_time_and_rejects_stale_maps(self):
        live = CupSensorInput(
            side="left", device="test", shoulder_twist_axis="y", wrist_twist_axis="y"
        )
        first = CupDepthFrame(10, 500, 1_000_000_000, object())
        latest = CupDepthFrame(11, 600, 1_040_000_000, object())
        live._publish_depth(first)
        live._publish_depth(latest)
        rgb = CupCameraFrame(20, 900, object(), device_time_ns=1_005_000_000)
        self.assertIs(live.matching_depth(rgb), first)
        self.assertIsNone(
            live.matching_depth(CupCameraFrame(21, 1000, object(), device_time_ns=1_100_000_000))
        )
        self.assertIsNone(live.matching_depth(CupCameraFrame(22, 1100, object())))
        self.assertEqual(live.drain_depth(), [first, latest])
        self.assertEqual(live.drain_depth(), [])

    def test_received_depth_maps_save_losslessly_with_separate_timestamps(self):
        import cv2
        import numpy as np

        with tempfile.TemporaryDirectory() as temporary:
            camera = CameraSession(
                cv2,
                "S01",
                "left",
                depth_enabled=True,
                directory=Path(temporary) / "År5 Mälardalen" / "camera",
                raw_video=True,
            )
            camera.start()
            depth = np.array([[0, 320, 1500], [3001, 65535, 1]], dtype=np.uint16)
            stamp = camera.started_host_monotonic_ns + 1_000_000
            camera.add_depth(CupDepthFrame(88, stamp, 2_000_000_000, depth))
            camera.stop()
            saved = cv2.imdecode(
                np.frombuffer((camera.session / "depth/000000.png").read_bytes(), dtype=np.uint8),
                cv2.IMREAD_UNCHANGED,
            )
            np.testing.assert_array_equal(saved, depth)
            frame = json.loads((camera.session / "depth_times.jsonl").read_text())
            self.assertEqual(frame["host_monotonic_ns"], stamp)
            self.assertEqual(frame["device_time_ns"], 2_000_000_000)
            self.assertEqual(frame["source_frame_sequence"], 88)
            self.assertEqual(frame["units"], "mm")
            self.assertEqual(
                json.loads((camera.session / "meta.json").read_text())["depth_frame_count"], 1
            )

    def test_camera_acquisition_continues_while_tracking_waits(self):
        live = CupSensorInput(
            side="left", device="test", shoulder_twist_axis="y", wrist_twist_axis="y"
        )
        tracking_started, captured = threading.Event(), threading.Event()
        live.pipeline = Mock()
        message = Mock()
        message.getSequenceNum.return_value = 15
        message.getTimestampDevice.return_value = timedelta(seconds=0.5)

        def blocked_tracking():
            tracking_started.set()
            live.stop_event.wait(2)

        def next_message():
            if not tracking_started.wait(2):
                raise RuntimeError("Tracking thread did not start")
            if live.received_frames:
                captured.set()
                return None
            return message

        live._track = blocked_tracking
        live.video_queue = Mock(tryGet=next_message)
        with patch("aurora.recording.cup_input.receive_frame", return_value=(object(), 123)):
            try:
                live.start_background()
                self.assertTrue(captured.wait(2), "Raw acquisition waited for tracking")
                self.assertTrue(live.threads[1].is_alive())
                self.assertEqual(live.drain_frames()[0].frame_time_ns, 123)
            finally:
                live.close()

    def test_raw_video_and_delayed_tracking_keep_separate_frame_times(self):
        with tempfile.TemporaryDirectory() as temporary:
            camera = CameraSession(
                Mock(), "S01", "left", directory=Path(temporary) / "camera", raw_video=True
            )
            camera.start()
            start = camera.started_host_monotonic_ns
            for sequence in range(1, 5):
                camera.add_frame(
                    object(),
                    None,
                    {},
                    host_monotonic_ns=start + sequence * 40_000_000,
                    frame_sequence=sequence,
                    save_observation=False,
                )
            camera.add_observation(
                None,
                {"source_frame_sequence": 1, "arm_visible": False},
                host_monotonic_ns=start + 40_000_000,
            )
            camera.stop()
            payload = json.loads((camera.session / "pose.json").read_text())
            times = [
                json.loads(line)
                for line in (camera.session / "frame_times.jsonl").read_text().splitlines()
            ]
            self.assertEqual(payload["source_frame_count"], 4)
            self.assertEqual(payload["observation_frame_count"], 1)
            self.assertFalse(payload["video_mirrored"])
            self.assertEqual(payload["video_kind"], "raw_rgb")
            self.assertEqual([item["source_frame_sequence"] for item in times], [1, 2, 3, 4])
            self.assertEqual(
                payload["observations"][0]["host_monotonic_ns"], times[0]["host_monotonic_ns"]
            )
            self.assertEqual(
                sum(item["encoded_copies"] for item in times), payload["video_frame_count"]
            )

    def test_raw_frames_continue_and_queue_loss_is_counted_without_tracking(self):
        live = CupSensorInput(
            side="left", device="test", shoulder_twist_axis="y", wrist_twist_axis="y"
        )
        live.tracking_error = "Tracking unavailable"
        for sequence in range(1, 81):
            live._publish_frame(CupCameraFrame(sequence, sequence * 40_000_000, object()))
        self.assertEqual(live.capture_health()["received_rgb_frames"], 80)
        self.assertEqual(live.capture_health()["raw_frame_queue_overruns"], 5)
        frames = live.drain_frames()
        self.assertEqual([frame.frame_sequence for frame in frames], list(range(6, 81)))
        self.assertEqual(live.drain_frames(), [])
        self.assertEqual(live.drain_tracking(), [])
        self.assertEqual(live.camera_frame.frame_time_ns, 80 * 40_000_000)

    def test_missing_optional_pose_model_fails_before_mediapipe_can_download_it(self):
        with tempfile.TemporaryDirectory() as temporary:
            package = Path(temporary)
            model = package / "modules/pose_landmark/pose_landmark_full.tflite"
            model.parent.mkdir(parents=True)
            model.touch()
            mp = SimpleNamespace(__file__=str(package / "__init__.py"))
            require_pose_model(mp, 1)
            with self.assertRaisesRegex(RuntimeError, "not installed"):
                require_pose_model(mp, 2)

    def test_camera_files_keep_the_receipt_time_after_slow_tracking(self):
        with tempfile.TemporaryDirectory() as temporary:
            camera = CameraSession(Mock(), "S01", "left")
            camera.writer = Mock()
            camera.session = Path(temporary)
            camera.started_host_monotonic_ns = 1_000_000_000
            camera.started = 1.0
            with patch(
                "aurora.sensors.camera.oak_camera.time.monotonic_ns", return_value=1_100_000_000
            ):
                frame, stamp = receive_frame(Mock())
            # Tracking finishes much later; the saved timestamp must stay at receipt.
            with patch("aurora.recording.oak_session.time.monotonic", return_value=1.9):
                camera.add_frame(frame, None, {}, host_monotonic_ns=stamp)
                camera.stop()
            payload = json.loads((Path(temporary) / "pose.json").read_text())
            self.assertEqual(payload["observations"][0]["host_monotonic_ns"], stamp)
            self.assertEqual(payload["observations"][0]["time_s"], 0.1)
            self.assertEqual(payload["timestamp_reference"], FRAME_TIMESTAMP_REFERENCE)

    def test_live_input_keeps_the_same_receipt_time_after_tracking(self):
        live = LiveSensorInput()
        live.video_queue = Mock()
        live.pose = live.mp = live.cv2 = live.np = None
        live.accept_camera_observation = Mock()
        live._snapshot = Mock()
        with (
            patch("aurora.sensors.live_input.receive_frame", return_value=(Mock(), 123)),
            patch(
                "aurora.sensors.live_input.analyze_frame", return_value=(Mock(), "ready", None, {})
            ),
        ):
            live._poll_sources()
        self.assertEqual(live.frame_time_ns, 123)
        self.assertEqual(live.camera_frame.frame_time_ns, 123)

    def test_slow_tracking_cannot_publish_a_frame_with_the_next_frames_timestamp(self):
        live = LiveSensorInput()
        live.video_queue = Mock()
        live.pose = live.mp = live.cv2 = live.np = None
        live.accept_camera_observation = live._snapshot = Mock()
        first, second = object(), object()
        with (
            patch("aurora.sensors.live_input.receive_frame", return_value=(first, 123)),
            patch(
                "aurora.sensors.live_input.analyze_frame",
                return_value=(first, "ready", None, {"id": 1}),
            ),
        ):
            live._poll_sources()
        saved = live.camera_frame

        def delayed_tracking(*_args, **_kwargs):
            self.assertIs(live.camera_frame, saved)
            self.assertEqual(live.camera_frame.frame_time_ns, 123)
            return second, "ready", None, {"id": 2}

        with (
            patch("aurora.sensors.live_input.receive_frame", return_value=(second, 456)),
            patch("aurora.sensors.live_input.analyze_frame", side_effect=delayed_tracking),
        ):
            live._poll_sources()
        self.assertEqual(live.camera_frame.frame_time_ns, 456)
        self.assertIs(live.camera_frame.latest_frame, second)
        self.assertEqual(live.camera_frame.camera_observation, {"id": 2})
        self.assertIs(saved.latest_frame, first)

    def test_frames_received_before_trial_start_are_not_saved(self):
        camera = CameraSession(Mock(), "S01", "left")
        camera.writer = Mock()
        camera.started_host_monotonic_ns = 200
        camera.add_frame(Mock(), None, {}, host_monotonic_ns=199)
        self.assertEqual(camera.observations, [])
        camera.writer.write.assert_not_called()


if __name__ == "__main__":
    unittest.main()
