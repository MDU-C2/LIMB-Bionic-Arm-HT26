# Christian och Majd
"""Tests for the complete synthetic Human Mimic controller pipeline."""

from __future__ import annotations

import unittest

import numpy as np

from aurora.simulation.dmp.controller import HumanMimicController
from aurora.simulation.dmp.dmp import fit
from aurora.simulation.dmp.mimic import HumanMotionSample, MimicState
from aurora.simulation.dmp.online import PhaseDrivenDMP
from aurora.simulation.dmp.progress import ProgressTracker


def synthetic_reference(sample_count: int = 11) -> np.ndarray:
    phase = np.linspace(0.0, 1.0, sample_count)
    return np.column_stack(
        (
            1.0 * phase,
            0.55 * phase + 0.10 * np.sin(np.pi * phase),
            -0.35 * phase,
            0.25 * phase**2,
        )
    )


class HumanMimicControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = synthetic_reference()
        cls.model = fit(
            [cls.reference],
            tau=1.0,
            dt=0.1,
            n_basis_functions=10,
            alpha_canonical=4.0,
            alpha_transformation=25.0,
            beta_transformation=6.25,
        )

    def make_controller(self):
        tracker = ProgressTracker(
            self.reference,
            stationary_timeout_s=0.25,
            lost_timeout_s=0.50,
        )
        phase_dmp = PhaseDrivenDMP(
            self.model,
            self.reference[0],
            self.reference[-1],
            sample_count=101,
        )
        return HumanMimicController(tracker, phase_dmp)

    def advance_to_index(self, controller, final_index, *, start_time=0.0):
        target = None
        for index in range(final_index + 1):
            target = controller.update(
                HumanMotionSample(
                    start_time + index * 0.1,
                    self.reference[index],
                    source="synthetic",
                )
            )
        return target

    def test_initial_state_is_synchronized_and_waiting(self):
        controller = self.make_controller()

        self.assertEqual(controller.last_progress.state, MimicState.WAITING)
        self.assertEqual(controller.last_target.state, MimicState.WAITING)
        self.assertEqual(controller.last_progress.progress, 0.0)
        self.assertEqual(controller.last_target.progress, 0.0)
        self.assertIsNone(controller.last_target.angles_rad)

    def test_single_update_entry_point_returns_progress_selected_dmp_target(self):
        controller = self.make_controller()
        target = self.advance_to_index(controller, 4)

        self.assertAlmostEqual(controller.last_progress.progress, 0.40)
        self.assertAlmostEqual(target.progress, 0.40)
        self.assertEqual(target.state, MimicState.RUNNING)
        self.assertEqual(controller.last_target, target)
        np.testing.assert_allclose(
            target.angles_rad,
            controller.phase_dmp.trajectory_rad[40],
        )

    def test_pause_and_short_dropout_hold_exactly_the_previous_target(self):
        controller = self.make_controller()
        running = self.advance_to_index(controller, 4)
        paused = controller.update(
            HumanMotionSample(0.8, self.reference[4], source="synthetic")
        )
        short_dropout = controller.update(
            HumanMotionSample(0.9, None, valid=False, source="synthetic")
        )

        self.assertEqual(paused.state, MimicState.HOLDING)
        self.assertEqual(short_dropout.state, MimicState.HOLDING)
        np.testing.assert_array_equal(paused.angles_rad, running.angles_rad)
        np.testing.assert_array_equal(short_dropout.angles_rad, running.angles_rad)

    def test_lost_input_suppresses_target_and_valid_input_recovers(self):
        controller = self.make_controller()
        self.advance_to_index(controller, 4)
        controller.update(HumanMotionSample(0.5, None, valid=False, source="synthetic"))
        lost = controller.update(
            HumanMotionSample(1.1, None, valid=False, source="synthetic")
        )
        recovered = controller.update(
            HumanMotionSample(1.2, self.reference[6], source="synthetic")
        )

        self.assertEqual(lost.state, MimicState.LOST)
        self.assertIsNone(lost.angles_rad)
        self.assertEqual(recovered.state, MimicState.RUNNING)
        self.assertAlmostEqual(recovered.progress, 0.60)
        self.assertIsNotNone(recovered.angles_rad)

    def test_complete_then_reset_starts_a_new_synchronized_movement(self):
        controller = self.make_controller()
        self.advance_to_index(controller, 5)
        complete = controller.update(
            HumanMotionSample(0.7, self.reference[-1], source="synthetic")
        )
        reset = controller.reset()
        restarted = controller.update(
            HumanMotionSample(0.0, self.reference[0], source="synthetic")
        )

        self.assertEqual(complete.state, MimicState.COMPLETE)
        self.assertEqual(complete.progress, 1.0)
        self.assertEqual(reset.state, MimicState.WAITING)
        self.assertIsNone(reset.angles_rad)
        self.assertEqual(controller.last_progress.state, MimicState.RUNNING)
        self.assertEqual(restarted.state, MimicState.RUNNING)
        self.assertEqual(restarted.progress, 0.0)

    def test_debug_log_contains_runtime_state_progress_confidence_and_target(self):
        controller = self.make_controller()

        with self.assertLogs("aurora.simulation.dmp.controller", level="DEBUG") as captured:
            controller.update(
                HumanMotionSample(0.0, self.reference[0], source="synthetic")
            )

        message = captured.output[-1]
        self.assertIn("state=running", message)
        self.assertIn("progress=0.000000", message)
        self.assertIn("confidence=1.000000", message)
        self.assertIn("target_rad=", message)

    def test_rejects_wrong_component_and_sample_types(self):
        tracker = ProgressTracker(self.reference)
        phase_dmp = PhaseDrivenDMP(
            self.model,
            self.reference[0],
            self.reference[-1],
        )

        for arguments in ((object(), phase_dmp), (tracker, object())):
            with self.subTest(arguments=arguments), self.assertRaises(TypeError):
                HumanMimicController(*arguments)
        controller = HumanMimicController(tracker, phase_dmp)
        with self.assertRaises(TypeError):
            controller.update(self.reference[0])


if __name__ == "__main__":
    unittest.main()
