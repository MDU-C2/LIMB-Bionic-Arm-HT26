# Majd
"""Check that tracking time does not move saved frame timestamps."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from aurora.recording.oak_session import CameraSession
from aurora.sensors.camera.oak_camera import FRAME_TIMESTAMP_REFERENCE, receive_frame
from aurora.sensors.live_input import LiveSensorInput


class CameraTimingTests(unittest.TestCase):
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

    def test_frames_received_before_trial_start_are_not_saved(self):
        camera = CameraSession(Mock(), "S01", "left")
        camera.writer = Mock()
        camera.started_host_monotonic_ns = 200
        camera.add_frame(Mock(), None, {}, host_monotonic_ns=199)
        self.assertEqual(camera.observations, [])
        camera.writer.write.assert_not_called()


if __name__ == "__main__":
    unittest.main()
