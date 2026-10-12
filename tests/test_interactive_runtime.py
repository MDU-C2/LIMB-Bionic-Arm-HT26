# Majd, Christian
"""Exercise the interactive task modules with real headless physics."""

from __future__ import annotations

import csv
import io
import os
import sys
import tempfile
import unittest
from collections import defaultdict
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/simulation"))

import pybullet as p
import pygame
from ai.live_sensor_input import LiveFusionSnapshot
from interactive.arm_control import get_angles_deg
from interactive.runtime import InteractiveSimulator
from interactive.settings import LEFT_ARM_LIMITS_DEG, parse_args
from sim.joint_limits import FINGER_LIMITS_DEG, finger_joint_angles_rad


class InteractiveRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.output = io.StringIO()
        self.capture = redirect_stdout(self.output)
        self.capture.__enter__()
        self.addCleanup(self.capture.__exit__, None, None, None)
        self.environment = patch.dict(
            os.environ, {"SDL_VIDEODRIVER": "dummy", "SDL_AUDIODRIVER": "dummy"}
        )
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.simulator = InteractiveSimulator(parse_args([]))
        self.addCleanup(self.simulator.close)
        connect = p.connect
        with patch("interactive.scene.p.connect", side_effect=lambda _mode: connect(p.DIRECT)):
            self.simulator.initialize_scene()
        self.simulator.initialize_arm()
        self.simulator.initialize_hud()
        self.simulator.initialize_task()
        self.simulator.start_live_input()
        # Tk is represented by widgets that accept updates; physics and HUD are real.
        self.simulator.sensor_window = Mock()
        self.simulator.sensor_vars = defaultdict(Mock)
        self.simulator.torque_graph = Mock()
        self.simulator.joint_names_map = {name: name for name in LEFT_ARM_LIMITS_DEG}
        self.frame()

    def frame(self, held_keys=()):
        simulator = self.simulator
        self.assertTrue(simulator.prepare_frame())
        simulator.process_events()
        simulator.keys = defaultdict(bool, dict.fromkeys(held_keys, True))
        simulator.update_grasp()
        simulator.update_arm()
        self.assertTrue(simulator.advance_physics())
        simulator.update_camera()
        simulator.draw_hud()
        simulator.update_dashboard()

    def test_keyboard_motion_reaches_physics_and_both_displays(self):
        before = self.simulator.arm.elbow.angle_x
        self.frame((pygame.K_UP,))
        self.assertGreater(self.simulator.arm.elbow.angle_x, before)
        for name, requested in get_angles_deg(self.simulator.arm).items():
            self.assertAlmostEqual(self.simulator.measured_angles_deg[name], requested, places=4)
        self.simulator.sensor_window.update.assert_called()

    def test_manual_grasp_release_and_reset_restore_the_scene(self):
        simulator = self.simulator
        near_hand = tuple(value + 0.01 for value in simulator.hand_position)
        p.resetBasePositionAndOrientation(simulator.target_body, near_hand, (0, 0, 0, 1))
        self.frame((pygame.K_f,))
        self.assertIsNotNone(simulator.grasp_transform)
        self.assertIsNone(simulator.target_anchor_constraint)
        self.frame((pygame.K_g,))
        self.assertIsNone(simulator.grasp_transform)
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_r))
        self.frame()
        position, _orientation = p.getBasePositionAndOrientation(simulator.target_body)
        self.assertEqual(tuple(position), tuple(simulator.target_start_position))
        self.assertIsNotNone(simulator.target_anchor_constraint)
        self.assertFalse(simulator.auto_grasp_pending)

    def test_live_targets_and_grip_follow_existing_limits(self):
        simulator = self.simulator
        simulator.args.control = "camera-imu"
        simulator.live_feedback_active = True
        simulator.live_sensor_input = Mock()
        simulator.live_sensor_input.poll.return_value = LiveFusionSnapshot(
            angles={"shoulder_flexion": 60.0, "elbow_flexion": 50.0},
            sensors={},
            camera_ready=True,
            imu_ready=False,
            status="Camera OK",
            hand_curl=0.8,
            grip_source="EMG",
        )
        before = simulator.arm.elbow.angle_x
        self.frame()
        self.assertGreater(simulator.arm.elbow.angle_x, before)
        self.assertGreater(simulator.arm.hand.curl, 0)
        self.assertLess(simulator.arm.hand.curl, 0.8 * 1.5)
        for name, value in get_angles_deg(simulator.arm).items():
            self.assertEqual(LEFT_ARM_LIMITS_DEG[name].clamp(value), value)

    def test_live_alignment_waits_for_a_new_snapshot_after_calibration(self):
        simulator = self.simulator
        simulator.args.control = "camera-imu"
        simulator.live_feedback_active = False
        simulator.live_sensor_input = Mock()
        simulator.live_sensor_input.poll.return_value = LiveFusionSnapshot(
            angles={"elbow_flexion": 50.0},
            sensors={},
            camera_ready=True,
            imu_ready=False,
            status="Camera OK",
        )
        before = get_angles_deg(simulator.arm)
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_l))
        self.frame()
        simulator.live_sensor_input.calibrate.assert_called_once()
        self.assertTrue(simulator.live_feedback_active)
        self.assertIsNone(simulator.live_snapshot)
        self.assertEqual(get_angles_deg(simulator.arm), before)

    def test_live_rotation_fallback_moves_real_joints_and_stops_at_limits(self):
        simulator = self.simulator
        simulator.args.control = "camera-imu"
        simulator.live_feedback_active = True
        simulator.live_snapshot = LiveFusionSnapshot(
            angles={"shoulder_flexion": 60.0, "elbow_flexion": 0.0},
            sensors={}, camera_ready=True, imu_ready=False, status="Camera OK",
        )
        simulator.loop_dt_s = 0.1
        simulator.keys = defaultdict(bool)
        # The PyBullet scene can have focus instead of the controller window.
        simulator.bullet_events = {
            ord("e"): p.KEY_IS_DOWN, p.B3G_RIGHT_ARROW: p.KEY_IS_DOWN,
        }
        for _ in range(80):
            simulator.update_arm()
        self.assertEqual(simulator.arm.shoulder.angle_x, 60.0)
        self.assertEqual(simulator.arm.wrist.rotation, 140.0)
        simulator.advance_physics()
        self.assertAlmostEqual(simulator.measured_angles_deg["shoulder_x"], 60.0, places=4)
        self.assertAlmostEqual(simulator.measured_angles_deg["wrist_rotation"], 140.0, places=4)

        # A visible bent elbow restores the camera's axial target, despite E held.
        simulator.live_snapshot.angles["shoulder_rotation_proxy"] = 30.0
        simulator.live_snapshot.angles["elbow_flexion"] = 50.0
        simulator.update_arm()
        self.assertLess(simulator.arm.shoulder.angle_x, 60.0)
        self.assertEqual(simulator.manual_motion.velocity_deg_s["shoulder_x"], 0.0)

        simulator.live_snapshot.angles.pop("shoulder_rotation_proxy")
        simulator.bullet_events = {
            ord("q"): p.KEY_IS_DOWN, p.B3G_LEFT_ARROW: p.KEY_IS_DOWN,
        }
        for _ in range(80):
            simulator.update_arm()
        self.assertEqual(simulator.arm.shoulder.angle_x, -60.0)
        self.assertEqual(simulator.arm.wrist.rotation, 0.0)
        simulator.advance_physics()
        self.assertAlmostEqual(simulator.measured_angles_deg["shoulder_x"], -60.0, places=4)
        self.assertAlmostEqual(simulator.measured_angles_deg["wrist_rotation"], 0.0, places=4)

    def test_live_camera_grip_opens_and_closes_inside_urdf_limits(self):
        simulator = self.simulator
        simulator.args.control = "camera-imu"
        simulator.live_feedback_active = True
        simulator.live_snapshot = LiveFusionSnapshot(
            angles=None, sensors={}, camera_ready=True, imu_ready=False,
            status="Camera OK", hand_curl=1.0, grip_source="Camera hand",
            emg_activation=0.0,
        )
        simulator.loop_dt_s = 0.1
        simulator.keys = defaultdict(bool)
        simulator.bullet_events = {}
        for target in (1.0, 0.0):
            simulator.live_snapshot.hand_curl = target
            for _ in range(24):
                before = simulator.arm.hand.curl
                simulator.update_grasp()
                progress_speed = abs(simulator.arm.hand.curl - before) / 1.5 / 0.1
                for limit in FINGER_LIMITS_DEG.values():
                    servo_speed = progress_speed * (limit.upper - limit.lower)
                    self.assertLessEqual(servo_speed, limit.speed(target - before) + 1e-8)
                simulator.update_arm()
                simulator.advance_physics()
            self.assertAlmostEqual(simulator.arm.hand.curl, 1.5 * target)
            expected = finger_joint_angles_rad(simulator.arm.hand.curl)
            for name, angle in expected.items():
                joint = simulator.joint_indices[name]
                lower, upper = p.getJointInfo(simulator.robot_body, joint)[8:10]
                measured = p.getJointState(simulator.robot_body, joint)[0]
                self.assertAlmostEqual(measured, angle, places=6)
                self.assertLessEqual(lower, measured)
                self.assertLessEqual(measured, upper)

    def test_ble_twist_reaches_real_upper_arm_and_wrist_joints(self):
        from ai.ble_sensor_input import BleSensorReader
        from ai.live_sensor_input import LiveSensorInput
        from ble_packets import HIGH_RATE_IMU_PACKET

        reader = BleSensorReader()
        for sample in range(150):
            arm_rate, wrist_rate = (0, 0) if sample < 50 else (3000, 7000)
            timestamp = 1_000_000 + sample * 10_000
            packet = HIGH_RATE_IMU_PACKET.pack(
                0x103, sample, timestamp, timestamp,
                0, 1000, 0, 0, arm_rate, 0, 0, 1000, 0, 0, wrist_rate, 0,
            )
            reader.notification("imu", packet)
        controller = LiveSensorInput(grip_source="camera")
        controller.reader = reader
        controller.cv2 = SimpleNamespace(waitKey=lambda _delay: -1)
        simulator = self.simulator
        simulator.args.control = "camera-imu"
        simulator.live_feedback_active = True
        simulator.live_snapshot = controller.poll()
        simulator.loop_dt_s = 0.1
        simulator.keys = defaultdict(bool)
        simulator.bullet_events = {}
        for _ in range(40):
            simulator.update_arm()
            simulator.advance_physics()
        self.assertAlmostEqual(simulator.measured_angles_deg["shoulder_x"], 30.0, places=4)
        self.assertAlmostEqual(simulator.measured_angles_deg["wrist_rotation"], 40.0, places=4)

    def test_dynamic_mode_writes_real_torque_measurements(self):
        simulator = self.simulator
        simulator.args = parse_args(
            ["--mode", "dynamic", "--torque-out", str(Path(self.temporary.name) / "torque.csv")]
        )
        simulator.dynamic_mode = True
        simulator.initialize_task()
        for _frame in range(12):
            self.frame()
        simulator.torque_csv_file.flush()
        with simulator.args.torque_out.open(newline="", encoding="utf-8") as file:
            rows = list(csv.DictReader(file))
        self.assertEqual(len(rows), 5)
        self.assertEqual({row["logical_joint"] for row in rows}, set(LEFT_ARM_LIMITS_DEG))
        self.assertTrue(all(float(row["time_s"]) > 0 for row in rows))
        self.assertIsNotNone(simulator.physics_snapshot)


if __name__ == "__main__":
    unittest.main()
