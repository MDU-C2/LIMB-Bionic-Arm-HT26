# Christian och Majd
"""Tests for Human Mimic robot mapping and synthetic PyBullet playback."""

from __future__ import annotations

import math
import unittest

import pybullet as p

from aurora.gui.tool_catalog import arguments_for
from aurora.simulation.dmp.mimic import MimicState, MimicTarget
from aurora.simulation.dmp.simulator import MimicRobotAdapter, mimic_robot_targets
from aurora.simulation.interactive.arm_control import AuroraArm, get_angles_deg
from aurora.simulation.mimic_simulation import parse_args, run_synthetic_mimic
from aurora.simulation.model.joint_limits import LEFT_ARM_LIMITS_DEG


def target(degrees, state=MimicState.RUNNING):
    return MimicTarget(
        0.5,
        [math.radians(value) for value in degrees],
        state,
    )


class MimicRobotAdapterTests(unittest.TestCase):
    def test_maps_human_radians_to_existing_limited_robot_degrees(self):
        mapped = mimic_robot_targets(target([30.0, 40.0, 15.0, -20.0]))

        expected = {
            "elbow_x": 30.0,
            "shoulder_y": 50.0,
            "shoulder_z": -15.0,
            "shoulder_x": 20.0,
        }
        self.assertEqual(set(mapped), set(expected))
        for name, value in expected.items():
            self.assertAlmostEqual(mapped[name], value)

    def test_mapping_clamps_every_command_to_robot_position_limits(self):
        mapped = mimic_robot_targets(target([120.0, -30.0, 100.0, 100.0]))

        self.assertEqual(mapped["elbow_x"], LEFT_ARM_LIMITS_DEG["elbow_x"].upper)
        self.assertEqual(mapped["shoulder_y"], LEFT_ARM_LIMITS_DEG["shoulder_y"].upper)
        self.assertEqual(mapped["shoulder_z"], LEFT_ARM_LIMITS_DEG["shoulder_z"].lower)
        self.assertEqual(mapped["shoulder_x"], LEFT_ARM_LIMITS_DEG["shoulder_x"].lower)

    def test_waiting_and_lost_targets_leave_the_arm_unchanged(self):
        arm = AuroraArm()
        adapter = MimicRobotAdapter()
        before = get_angles_deg(arm)

        waiting = MimicTarget(0.0, None, MimicState.WAITING)
        lost = MimicTarget(0.5, None, MimicState.LOST)

        self.assertIsNone(adapter.apply(arm, waiting, elapsed_s=0.02))
        self.assertIsNone(adapter.apply(arm, lost, elapsed_s=0.02))
        self.assertEqual(get_angles_deg(arm), before)

    def test_speed_limited_application_uses_existing_actuator_speeds(self):
        arm = AuroraArm()
        adapter = MimicRobotAdapter()
        adapter.apply(
            arm,
            target([60.0, 30.0, 35.0, -50.0]),
            elapsed_s=0.1,
            speed_limited=True,
        )

        self.assertAlmostEqual(arm.elbow.angle_x, 4.0)
        self.assertAlmostEqual(arm.shoulder.angle_y, 1.0)
        self.assertAlmostEqual(arm.shoulder.angle_z, -7.0)
        self.assertAlmostEqual(arm.shoulder.angle_x, 4.0)

    def test_direct_preview_applies_all_four_mapped_targets(self):
        arm = AuroraArm()
        adapter = MimicRobotAdapter()
        mapped = adapter.apply(
            arm,
            target([30.0, 40.0, 15.0, -20.0], MimicState.HOLDING),
            elapsed_s=0.02,
            speed_limited=False,
        )

        commanded = get_angles_deg(arm)
        for name, value in mapped.items():
            self.assertAlmostEqual(commanded[name], value)
        self.assertEqual(adapter.last_robot_targets, mapped)

    def test_rejects_invalid_adapter_inputs(self):
        adapter = MimicRobotAdapter()
        arm = AuroraArm()
        with self.assertRaises(TypeError):
            mimic_robot_targets(object())
        with self.assertRaises(TypeError):
            adapter.apply(object(), target([0, 0, 0, 0]), elapsed_s=0.1)
        for elapsed in (0, -1, float("nan")):
            with self.subTest(elapsed=elapsed), self.assertRaises(ValueError):
                adapter.apply(arm, target([0, 0, 0, 0]), elapsed_s=elapsed)


class MimicPyBulletTests(unittest.TestCase):
    def tearDown(self):
        if p.isConnected():
            p.disconnect()

    def test_complete_pipeline_runs_headless_in_pybullet(self):
        result = run_synthetic_mimic(duration_s=0.5, mode="kinematic", headless=True)

        self.assertEqual(result.state, MimicState.COMPLETE.value)
        self.assertEqual(result.progress, 1.0)
        self.assertEqual(result.steps, 30)
        self.assertFalse(p.isConnected())
        for name, angle in result.commanded_angles_deg.items():
            self.assertGreaterEqual(angle, LEFT_ARM_LIMITS_DEG[name].lower)
            self.assertLessEqual(angle, LEFT_ARM_LIMITS_DEG[name].upper)

    def test_desktop_tool_arguments_use_the_same_backend_contract(self):
        values = {
            "mode": "Motor physics",
            "duration": 2.5,
            "headless": True,
        }
        args = parse_args(arguments_for("mimic_sim", values))

        self.assertEqual(args.mode, "dynamic")
        self.assertEqual(args.duration, 2.5)
        self.assertTrue(args.headless)


if __name__ == "__main__":
    unittest.main()
