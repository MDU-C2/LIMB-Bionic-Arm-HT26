# Majd
"""Drive the live mounting guide with known, arbitrarily mounted IMUs."""

from __future__ import annotations

import hashlib
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from test_arm_alignment import generated_reference

from aurora.sensors.ble_input import BleSensorReader
from aurora.sensors.ble_protocol import HIGH_RATE_IMU_PACKET
from aurora.sensors.camera.oak_camera import CameraFrame
from aurora.sensors.live_input import LiveFusionSnapshot, LiveSensorInput
from aurora.sensors.reference_capture import ReferenceCapture
from aurora.simulation.interactive.live_setup import LiveSetup


def wait_for_fit(capture):
    deadline = time.monotonic() + 5
    while not capture.finished and time.monotonic() < deadline:
        time.sleep(0.005)
    if not capture.finished:
        raise AssertionError("Calibration worker did not finish")


class ReferenceCaptureTests(unittest.TestCase):
    def test_slow_reference_save_can_finish_after_a_cleanup_timeout(self):
        capture = ReferenceCapture(
            subject_id="S01", side="left", connection_generation=1, mounting_session_id="one"
        )
        capture.begin_step("STILL", 0)
        capture.end_step(5_000_000_000)
        capture.error = "Preparation cancelled"
        entered, release = threading.Event(), threading.Event()
        write = Path.write_bytes

        def slow_write(path, payload):
            entered.set()
            release.wait(3)
            return write(path, payload)

        with (
            tempfile.TemporaryDirectory() as temporary,
            patch.object(Path, "write_bytes", slow_write),
        ):
            try:
                capture.finish(Path(temporary))
                self.assertTrue(entered.wait(1))
                with self.assertRaises(TimeoutError):
                    capture.close(timeout=0)
                self.assertFalse(capture.finished)
            finally:
                release.set()
                capture.close()
            self.assertTrue((capture.directory / "inputs.json").is_file())
            self.assertEqual(capture.result["status"], "failed")

    def test_live_guide_fits_the_existing_connection_and_preserves_inputs(self):
        for side, mismatched_depth in (("left", False), ("right", False), ("left", True)):
            with (
                self.subTest(side=side, mismatched_depth=mismatched_depth),
                tempfile.TemporaryDirectory() as temporary,
            ):
                clock = [100.0]
                live = LiveSensorInput(
                    depth=True, side=side, show_monitor=False, clock=lambda clock=clock: clock[0]
                )
                live.reader = BleSensorReader(side=side, clock=live.clock)
                live.raw_camera_angles = dict(
                    elbow_flexion=90, shoulder_flexion=0, shoulder_abduction=0
                )
                guide = LiveSetup(
                    live, {"elbow_flexion": 45}, subject_id="S07", output=Path(temporary)
                )
                state = LiveFusionSnapshot({}, {}, True, True, "Reference", emg_raw=2048)
                stages, camera, mounting, _ = generated_reference(side)
                sequence = 0
                for cue, values in stages.items():
                    guide.update(state, clock[0])
                    guide.action()
                    clock[0] += 3
                    guide.update(state, clock[0])
                    start = clock[0]
                    for index, stamp in enumerate(values["shoulder"].time_s):
                        clock[0] = start + stamp
                        host_ns = round(clock[0] * 1e9)
                        vectors = []
                        for role in ("shoulder", "wrist"):
                            vectors.extend(np.rint(values[role].accel_g[index] * 1000).astype(int))
                            vectors.extend(np.rint(values[role].gyro_dps[index] * 100).astype(int))
                        packet = HIGH_RATE_IMU_PACKET.pack(
                            0x103, sequence, host_ns // 1000, host_ns // 1000, *vectors
                        )
                        live.reader.decoder.handle("imu", packet, host_monotonic_ns=host_ns)
                        sequence += 1
                        if index % 20 == 0:
                            points = {
                                key: value.tolist()
                                for key, value in camera[cue][index // 20].items()
                            }
                            live.camera_frame = CameraFrame(
                                sequence, host_ns, None, device_time_ns=host_ns
                            )
                            live.camera_observation = dict(
                                keypoints_camera_m=points,
                                depth_time_difference_ns=50_000_000 if mismatched_depth else 0,
                            )
                            live.camera_time = clock[0]
                        guide.update(state, clock[0])
                    clock[0] = start + (5 if cue in ("STILL", "RETURN") else 10)
                    guide.update(state, clock[0])
                capture = guide.capture
                self.assertEqual(guide.stage, "fit")
                wait_for_fit(capture)
                if mismatched_depth:
                    self.assertEqual(capture.result["status"], "failed")
                    self.assertIn("stereo arm points", capture.result["message"])
                    guide.update(state, clock[0])
                    self.assertEqual(guide.stage, "mount")
                    self.assertIsNone(live.reader.alignment_profile)
                    continue
                self.assertEqual(capture.result["status"], "passed", capture.result)
                guide.update(state, clock[0])
                self.assertEqual(guide.stage, "imu")
                self.assertIsNone(live.reader.reference_capture)
                self.assertEqual(live.reader.alignment_profile["subject_id"], "S07")
                self.assertTrue(live._calibration_valid)
                for role in mounting:
                    np.testing.assert_allclose(
                        live.reader.imu_estimator.sensor_rotations[role], mounting[role], atol=0.001
                    )
                original = (capture.directory / "inputs.json").read_bytes()
                self.assertEqual(
                    capture.result["source_sha256"]["inputs.json"],
                    hashlib.sha256(original).hexdigest(),
                )
                source = json.loads(original)
                self.assertEqual(len(source["stages"]["BEND"]["imus"]["wrist"]), 1000)
                self.assertEqual(source["subject_id"], "S07")
                self.assertFalse(capture.result["accuracy_validated"])

    def test_held_measurements_are_not_new_samples_and_overrun_fails(self):
        capture = ReferenceCapture(
            subject_id="S01", side="left", connection_generation=1, mounting_session_id="one"
        )
        capture.begin_step("STILL", 0)
        sensor = dict(
            sample_time_us=10,
            host_monotonic_ns=10_000,
            accel_g=dict(x=0, y=0, z=1),
            gyro_dps=dict.fromkeys("xyz", 0),
        )
        for _ in range(10):
            capture.add_imus({"shoulder": sensor})
        self.assertEqual(len(capture.stages["STILL"]["imus"]["shoulder"]), 1)
        for index in range(2001):
            capture.add_imus({"shoulder": {**sensor, "sample_time_us": 20 + index}})
        capture.end_step(30_000_000_000)
        with tempfile.TemporaryDirectory() as temporary:
            capture.finish(Path(temporary))
            wait_for_fit(capture)
            self.assertEqual(capture.result["status"], "failed")
            self.assertIn("buffer full", capture.result["message"])
            self.assertEqual(len(capture.stages["STILL"]["imus"]["shoulder"]), 2000)

    def test_reconnect_cancels_the_guide_without_reusing_its_fit(self):
        with tempfile.TemporaryDirectory() as temporary:
            live = LiveSensorInput(depth=True, show_monitor=False, clock=lambda: 100.0)
            live.reader = BleSensorReader()
            guide = LiveSetup(live, {}, output=Path(temporary))
            state = LiveFusionSnapshot({}, {}, True, True, "Ready")
            guide.update(state, 100)
            guide.action()
            guide.update(state, 103)
            capture = guide.capture
            live.reader._start_connection()
            guide.update(state, 104)
            self.assertEqual(guide.stage, "mount")
            self.assertIsNone(guide.capture)
            self.assertIsNone(live.reader.reference_capture)
            self.assertIsNone(live.reader.alignment_profile)
            wait_for_fit(capture)
            self.assertEqual(capture.result["status"], "failed")
            self.assertIn("cancelled", capture.result["message"])


if __name__ == "__main__":
    unittest.main()
