# Majd
"""Check guided calibration and the live controller without hardware."""

from __future__ import annotations

import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from aurora.sensors.ble_input import BleSensorReader
from aurora.sensors.ble_protocol import HIGH_RATE_IMU_PACKET
from aurora.sensors.fusion import (
    DualImuArmEstimator,
    fuse_control_angles,
)
from aurora.sensors.live_input import LiveFusionSnapshot, LiveSensorInput
from aurora.simulation.interactive.live_setup import LiveSetup


def snapshot(**changes):
    values = dict(
        angles={"elbow_flexion": 45},
        camera_ready=True,
        imu_ready=True,
        sensors={name: {"gyro_dps": dict.fromkeys("xyz", 0)} for name in ("shoulder", "wrist")},
        status="ready",
        emg_raw=2048,
        emg_activation=0.0,
    )
    return LiveFusionSnapshot(**{**values, **changes})


class LiveSetupTests(unittest.TestCase):
    def test_delayed_camera_result_cannot_correct_current_imu_state(self):
        clock = [100.0]
        live = LiveSensorInput(show_monitor=False, clock=lambda: clock[0])
        live.reader = BleSensorReader(clock=live.clock)
        for index in range(60):
            live.reader.notification(
                "imu",
                HIGH_RATE_IMU_PACKET.pack(
                    0x103, index, index * 10_000, index * 10_000, *([0, 0, 1000, 0, 0, 0] * 2)
                ),
            )
        before = dict(live.reader.imu_estimator.last_angles)
        live.accept_camera_observation(
            {"angles_deg": {"elbow_flexion": 130, "shoulder_flexion": 60}}, 99.0
        )
        state = live.poll()
        self.assertFalse(state.camera_ready)
        self.assertEqual(live.reader.imu_estimator.last_angles, before)
        self.assertIn("too old", state.status)
        live.close()

    def setUp(self):
        self.live = SimpleNamespace(
            reader=Mock(connection_generation=1),
            grip_source="emg",
            raw_camera_angles={"elbow_flexion": 95},
            camera_origin={"elbow_flexion": 95},
            calibrate=Mock(),
            calibrate_emg=Mock(),
            calibrate_emg_strength=Mock(return_value=True),
        )
        self.guide = LiveSetup(self.live, {"elbow_flexion": 45})

    def test_alignment_waits_for_camera_and_still_arm(self):
        self.guide.action()
        self.guide.update(snapshot(camera_ready=False), 10)
        self.guide.update(snapshot(), 11)
        self.guide.update(snapshot(), 12.9)
        self.live.calibrate.assert_not_called()
        self.guide.update(snapshot(), 13.1)
        self.live.calibrate.assert_called_once_with({"elbow_flexion": 45})
        self.assertEqual(self.guide.stage, "imu")

    def test_movement_restarts_hold(self):
        self.guide.action()
        self.guide.update(snapshot(), 10)
        moving = snapshot()
        moving.sensors["shoulder"]["gyro_dps"]["x"] = 30
        self.guide.update(moving, 11.9)
        self.guide.update(snapshot(), 12)
        self.guide.update(snapshot(), 13)
        self.live.calibrate.assert_not_called()

    def test_repeated_shortcut_does_not_restart_a_pending_step(self):
        self.guide.action()
        self.guide.update(snapshot(), 10)
        self.guide.action()
        self.guide.update(snapshot(), 12.1)
        self.live.calibrate.assert_called_once_with({"elbow_flexion": 45})

    def test_baseline_and_grip_require_separate_user_actions(self):
        self.guide.stage, self.guide.pending = "imu", True
        self.live.reader.latest.return_value = {"imu_calibrated": True}
        self.guide.update(snapshot(), 10)
        self.assertEqual(self.guide.stage, "rest")
        self.live.calibrate_emg.assert_not_called()
        self.live.reader.graph_data.return_value = {
            "emg_calibration_state": {"phase": "rest", "message": "Collecting relaxed baseline"}
        }
        self.guide.action()
        self.live.calibrate_emg.assert_not_called()
        self.guide.update(snapshot(), 12.9)
        self.live.calibrate_emg.assert_not_called()
        self.guide.update(snapshot(), 13.1)
        self.live.calibrate_emg.assert_called_once()
        self.live.reader.graph_data.return_value = {
            "emg_calibration_state": {"phase": "grip_reference_needed", "message": "Rest saved"}
        }
        self.guide.update(snapshot(), 14)
        self.assertEqual(self.guide.stage, "grip")
        self.live.calibrate_emg_strength.assert_not_called()
        self.guide.action()
        self.live.reader.graph_data.return_value = {
            "emg_calibration_state": {"phase": "failed", "message": "Grip too weak; repeat"}
        }
        self.guide.update(snapshot(), 17.1)
        self.assertEqual(self.guide.stage, "grip")
        self.assertFalse(self.guide.pending)
        self.guide.action()
        self.live.reader.graph_data.return_value = {
            "emg_calibration_state": {"phase": "ready", "message": "Grip reference saved"}
        }
        self.guide.update(snapshot(), 20.2)
        self.assertEqual(self.guide.stage, "ready")
        self.assertTrue(self.guide.action())

    def test_control_waits_for_return_to_start_and_relaxation(self):
        self.guide.stage = "ready"
        self.live.raw_camera_angles["elbow_flexion"] = 50
        self.guide.update(snapshot(), 10)
        self.assertFalse(self.guide.action())
        self.live.raw_camera_angles["elbow_flexion"] = 95
        self.guide.update(snapshot(emg_activation=0.7), 11)
        self.assertFalse(self.guide.action())
        self.guide.update(snapshot(emg_activation=0), 12)
        self.assertTrue(self.guide.action())

    def test_reconnect_and_lost_tracking_pause_control(self):
        self.guide.stage = "running"
        self.live.reader.connection_generation = 2
        self.guide.update(snapshot(), 10)
        self.assertEqual(self.guide.stage, "align")
        self.guide.stage = "running"
        self.guide.update(snapshot(camera_ready=False, imu_ready=False, emg_raw=None), 11)
        self.assertEqual(self.guide.stage, "align")

    def test_pause_retains_only_same_connection_calibration_and_mounting_repeat_invalidates(self):
        self.guide.stage = "running"
        self.guide.update(snapshot(), 10)
        self.guide.pause()
        self.assertEqual(self.guide.stage, "ready")
        self.live.calibrate.assert_not_called()
        self.assertTrue(self.guide.action())
        self.live.reader.connection_generation = 2
        self.guide.pause()
        self.assertEqual(self.guide.stage, "align")
        self.live.invalidate_calibration = Mock()
        self.guide.repeat()
        self.live.invalidate_calibration.assert_called_once()
        self.assertIn("Mounting changed", self.guide.message)

    def test_explicit_invalidation_removes_live_validity_without_stopping_raw_sources(self):
        live = LiveSensorInput(show_monitor=False, clock=lambda: 1000)
        live.reader = BleSensorReader(clock=live.clock)
        live.calibrate()
        self.assertTrue(live.poll().calibration_valid)
        live.invalidate_calibration()
        self.assertFalse(live.poll().calibration_valid)
        self.assertIsNone(live.reader.alignment_profile)
        self.assertFalse(live.reader._stop.is_set())
        live.close()

    def test_real_elbow_change_above_robot_limit_survives_alignment(self):
        clock = [1000.0]
        live = LiveSensorInput(show_monitor=False, clock=lambda: clock[0])
        live.accept_camera_observation({"angles_deg": {"elbow_flexion": 95}}, clock[0])
        live.calibrate({"elbow_flexion": 45})
        clock[0] += 0.4
        live.accept_camera_observation({"angles_deg": {"elbow_flexion": 85}}, clock[0])
        self.assertEqual(live.camera_angles["elbow_flexion"], 85)
        self.assertEqual(live.poll().robot_targets["elbow_x"], 35)

    def test_camera_correction_persists_in_later_imu_predictions(self):
        estimator = DualImuArmEstimator(alpha=0)
        sensors = {
            name: {
                "connected": True,
                "accel_g": {"x": 0, "y": 0, "z": 1},
                "gyro_dps": dict.fromkeys("xyz", 0),
            }
            for name in ("shoulder", "wrist")
        }
        estimator.update(sensors, 0.01)
        estimator.correct_angles({"shoulder_flexion": 30, "elbow_flexion": 45}, 1)
        later = estimator.update(sensors, 0.01)
        self.assertEqual(later["shoulder_flexion"], 30)
        self.assertEqual(later["elbow_flexion"], 45)

    def test_corrected_imu_motion_is_not_blended_with_a_held_camera_frame_twice(self):
        imu = {"elbow_flexion": 55}
        camera = {"elbow_flexion": 45}
        self.assertEqual(fuse_control_angles(imu, camera, 0.25, camera_corrected=True), imu)
        self.assertEqual(fuse_control_angles(imu, camera, 1.0, camera_corrected=True), camera)

    def test_slow_camera_does_not_block_control_polling(self):
        live = LiveSensorInput(show_monitor=False)
        entered, release = threading.Event(), threading.Event()

        def slow_camera():
            entered.set()
            release.wait(2)
            return snapshot()

        live._poll_camera = slow_camera
        try:
            live.start_background()
            self.assertTrue(entered.wait(1))
            start = time.monotonic()
            for _ in range(50):
                live.poll()
            self.assertLess(time.monotonic() - start, 0.1)
        finally:
            release.set()
            live.close()

    def test_new_ble_angles_reach_control_while_camera_is_processing(self):
        live = LiveSensorInput(show_monitor=False, clock=lambda: 1000.0)
        live.reader = BleSensorReader(clock=live.clock)
        live.reader.imu_estimator = DualImuArmEstimator(alpha=0)
        entered, release = threading.Event(), threading.Event()

        def packet(y, z, timestamp):
            return HIGH_RATE_IMU_PACKET.pack(
                0x103,
                timestamp // 10_000,
                timestamp,
                timestamp,
                0,
                y,
                z,
                0,
                0,
                0,
                0,
                y,
                z,
                0,
                0,
                0,
            )

        live.reader.notification("imu", packet(0, 1000, 1_000_000))

        def slow_camera():
            entered.set()
            release.wait(2)
            return snapshot()

        live._poll_camera = slow_camera
        try:
            live.start_background()
            self.assertTrue(entered.wait(1))
            before = live.poll().relative_angles["shoulder_flexion"]
            live.reader.notification("imu", packet(707, 707, 1_010_000))
            after = live.poll().relative_angles["shoulder_flexion"]
            self.assertEqual(before, 0)
            self.assertAlmostEqual(after, 45)
        finally:
            release.set()
            live.close()


if __name__ == "__main__":
    unittest.main()
