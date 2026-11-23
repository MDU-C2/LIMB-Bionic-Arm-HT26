# Christian och Majd
"""Tests for progress-driven online DMP target selection."""

from __future__ import annotations

import unittest

import numpy as np

from aurora.simulation.dmp.dmp import DMPModel, fit
from aurora.simulation.dmp.mimic import HumanMotionSample, MimicState, ProgressState
from aurora.simulation.dmp.online import PhaseDrivenDMP
from aurora.simulation.dmp.progress import ProgressTracker


def synthetic_reference(sample_count: int = 81) -> np.ndarray:
    phase = np.linspace(0.0, 1.0, sample_count)
    return np.column_stack(
        (
            1.0 * phase,
            0.55 * phase + 0.10 * np.sin(np.pi * phase),
            -0.35 * phase,
            0.25 * phase**2,
        )
    )


class PhaseDrivenDMPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = synthetic_reference()
        cls.tau = 1.6
        cls.model = fit(
            [cls.reference],
            tau=cls.tau,
            dt=cls.tau / (len(cls.reference) - 1),
            n_basis_functions=20,
            alpha_canonical=4.0,
            alpha_transformation=25.0,
            beta_transformation=6.25,
        )

    def make_online(self, *, sample_count=101):
        return PhaseDrivenDMP(
            self.model,
            self.reference[0],
            self.reference[-1],
            sample_count=sample_count,
        )

    def test_running_progress_selects_the_corresponding_dmp_pose(self):
        online = self.make_online(sample_count=101)
        target = online.update(ProgressState(0.40, 32, 0.95, MimicState.RUNNING))

        self.assertEqual(target.state, MimicState.RUNNING)
        self.assertEqual(target.progress, 0.40)
        np.testing.assert_allclose(target.angles_rad, online.trajectory_rad[40])
        self.assertFalse(target.angles_rad.flags.writeable)

    def test_fractional_progress_interpolates_between_rollout_samples(self):
        online = self.make_online(sample_count=11)
        angles = online.angles_at(0.25)
        expected = 0.5 * online.trajectory_rad[2] + 0.5 * online.trajectory_rad[3]

        np.testing.assert_allclose(angles, expected)
        self.assertFalse(angles.flags.writeable)

    def test_equal_progress_produces_equal_targets_independent_of_human_speed(self):
        slow = self.make_online()
        fast = self.make_online()

        slow_target = slow.update(ProgressState(0.60, 48, 1.0, MimicState.RUNNING))
        fast_target = fast.update(ProgressState(0.60, 48, 1.0, MimicState.RUNNING))

        np.testing.assert_array_equal(slow_target.angles_rad, fast_target.angles_rad)

    def test_holding_reuses_the_previous_target(self):
        online = self.make_online()
        running = online.update(ProgressState(0.35, 28, 1.0, MimicState.RUNNING))
        holding = online.update(ProgressState(0.35, 28, 0.8, MimicState.HOLDING))

        self.assertEqual(holding.state, MimicState.HOLDING)
        self.assertEqual(holding.progress, running.progress)
        np.testing.assert_array_equal(holding.angles_rad, running.angles_rad)
        self.assertIn("holding", holding.status)

    def test_holding_without_an_earlier_target_selects_its_current_progress(self):
        online = self.make_online(sample_count=101)
        holding = online.update(ProgressState(0.20, 16, 0.8, MimicState.HOLDING))

        np.testing.assert_allclose(holding.angles_rad, online.trajectory_rad[20])
        self.assertEqual(holding.state, MimicState.HOLDING)

    def test_waiting_and_lost_states_do_not_produce_joint_targets(self):
        online = self.make_online()
        waiting = online.update(ProgressState(0.0, 0, 0.0, MimicState.WAITING))
        online.update(ProgressState(0.40, 32, 0.9, MimicState.RUNNING))
        lost = online.update(ProgressState(0.40, 32, 0.0, MimicState.LOST))

        self.assertIsNone(waiting.angles_rad)
        self.assertEqual(waiting.state, MimicState.WAITING)
        self.assertIsNone(lost.angles_rad)
        self.assertEqual(lost.state, MimicState.LOST)

    def test_complete_uses_the_final_dmp_pose(self):
        online = self.make_online()
        complete = online.update(ProgressState(1.0, 80, 1.0, MimicState.COMPLETE))

        self.assertEqual(complete.progress, 1.0)
        self.assertEqual(complete.state, MimicState.COMPLETE)
        np.testing.assert_allclose(complete.angles_rad, online.trajectory_rad[-1])

    def test_backward_progress_is_rejected_until_reset(self):
        online = self.make_online()
        online.update(ProgressState(0.60, 48, 1.0, MimicState.RUNNING))

        with self.assertRaisesRegex(ValueError, "must not move backwards"):
            online.update(ProgressState(0.40, 32, 1.0, MimicState.RUNNING))
        reset = online.reset()
        resumed = online.update(ProgressState(0.40, 32, 1.0, MimicState.RUNNING))

        self.assertEqual(reset.state, MimicState.WAITING)
        self.assertIsNone(reset.angles_rad)
        self.assertEqual(resumed.progress, 0.40)

    def test_progress_tracker_and_phase_driven_dmp_work_end_to_end(self):
        reference = synthetic_reference(11)
        model = fit(
            [reference],
            tau=1.0,
            dt=0.1,
            n_basis_functions=10,
            alpha_canonical=4.0,
            alpha_transformation=25.0,
            beta_transformation=6.25,
        )
        tracker = ProgressTracker(reference, stationary_timeout_s=0.25)
        online = PhaseDrivenDMP(model, reference[0], reference[-1], sample_count=101)

        for index in range(5):
            progress = tracker.update(
                HumanMotionSample(index * 0.1, reference[index], source="synthetic")
            )
            target = online.update(progress)
        paused_progress = tracker.update(
            HumanMotionSample(0.8, reference[4], source="synthetic")
        )
        paused_target = online.update(paused_progress)

        self.assertAlmostEqual(progress.progress, 0.40)
        self.assertEqual(target.state, MimicState.RUNNING)
        self.assertEqual(paused_progress.state, MimicState.HOLDING)
        self.assertEqual(paused_target.state, MimicState.HOLDING)
        np.testing.assert_array_equal(paused_target.angles_rad, target.angles_rad)

    def test_validates_model_vectors_sample_count_and_progress_input(self):
        bad_model = DMPModel(
            weights=np.zeros((3, 1)),
            centers=np.ones(1),
            widths=np.ones(1),
            alpha_canonical=4.0,
            alpha_transformation=25.0,
            beta_transformation=6.25,
            tau=1.0,
            n_joints=3,
        )
        for arguments in (
            (bad_model, np.zeros(4), np.ones(4), 10),
            (self.model, np.zeros(3), np.ones(4), 10),
            (self.model, np.zeros(4), np.ones(3), 10),
            (self.model, np.zeros(4), np.ones(4), 1),
            (self.model, np.zeros(4), np.ones(4), 2.5),
        ):
            with self.subTest(arguments=arguments), self.assertRaises(
                (TypeError, ValueError)
            ):
                PhaseDrivenDMP(
                    arguments[0],
                    arguments[1],
                    arguments[2],
                    sample_count=arguments[3],
                )

        online = self.make_online()
        for progress in (-0.1, 1.1, float("nan"), True):
            with self.subTest(progress=progress), self.assertRaises(ValueError):
                online.angles_at(progress)
        with self.assertRaises(TypeError):
            online.update(0.5)


if __name__ == "__main__":
    unittest.main()
