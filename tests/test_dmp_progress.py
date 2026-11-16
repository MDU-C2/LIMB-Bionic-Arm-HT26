# Christian och Majd
"""Tests for synthetic Human Mimic movement progress tracking."""

from __future__ import annotations

import unittest

import numpy as np

from aurora.simulation.dmp.mimic import HumanMotionSample, MimicState
from aurora.simulation.dmp.progress import ProgressTracker


def synthetic_reference(sample_count: int = 101) -> np.ndarray:
    phase = np.linspace(0.0, 1.0, sample_count)
    return np.column_stack(
        (
            1.1 * phase,
            0.65 * phase + 0.12 * np.sin(np.pi * phase),
            -0.45 * phase,
            0.30 * phase**2,
        )
    )


class ProgressTrackerTests(unittest.TestCase):
    def test_tracks_the_same_movement_at_normal_half_and_double_speed(self):
        reference = synthetic_reference()

        for speed in (0.5, 1.0, 2.0):
            with self.subTest(speed=speed):
                tracker = ProgressTracker(reference)
                observed_progress = []
                for index, angles in enumerate(reference):
                    state = tracker.update(
                        HumanMotionSample(index * 0.02 / speed, angles, source="synthetic")
                    )
                    observed_progress.append(state.progress)

                self.assertEqual(state.state, MimicState.COMPLETE)
                self.assertEqual(state.progress, 1.0)
                self.assertTrue(
                    np.all(np.diff(np.asarray(observed_progress)) >= 0),
                    "progress must never move backwards",
                )

    def test_reports_geometric_progress_not_elapsed_time(self):
        reference = synthetic_reference(11)
        tracker = ProgressTracker(reference)

        start = tracker.update(HumanMotionSample(0.0, reference[0], source="synthetic"))
        middle = tracker.update(HumanMotionSample(8.0, reference[4], source="synthetic"))

        self.assertEqual(start.progress, 0.0)
        self.assertAlmostEqual(middle.progress, 0.40, places=6)
        self.assertEqual(middle.state, MimicState.RUNNING)

    def test_pauses_at_the_last_progress_and_resumes_when_the_human_moves(self):
        reference = synthetic_reference(21)
        tracker = ProgressTracker(reference, stationary_timeout_s=0.25)

        moving = tracker.update(HumanMotionSample(0.10, reference[5], source="synthetic"))
        paused = tracker.update(HumanMotionSample(0.40, reference[5], source="synthetic"))
        resumed = tracker.update(HumanMotionSample(0.50, reference[7], source="synthetic"))

        self.assertAlmostEqual(moving.progress, 0.25)
        self.assertEqual(tracker.snapshot(), resumed)
        self.assertEqual(paused.progress, moving.progress)
        self.assertEqual(paused.state, MimicState.HOLDING)
        self.assertGreater(resumed.progress, paused.progress)
        self.assertEqual(resumed.state, MimicState.RUNNING)

    def test_noise_and_backward_motion_cannot_reduce_progress(self):
        reference = synthetic_reference(51)
        tracker = ProgressTracker(reference)
        generator = np.random.default_rng(2026)
        progress = []

        for index in range(0, 31, 2):
            noisy_angles = reference[index] + generator.normal(0.0, 0.008, 4)
            progress.append(
                tracker.update(
                    HumanMotionSample(index * 0.02, noisy_angles, source="synthetic")
                ).progress
            )
        before_backward_sample = progress[-1]
        backward = tracker.update(
            HumanMotionSample(1.0, reference[20], source="synthetic")
        )

        self.assertTrue(np.all(np.diff(np.asarray(progress)) >= 0))
        self.assertGreater(progress[-1], 0.55)
        self.assertEqual(backward.progress, before_backward_sample)

    def test_short_sensor_dropout_holds_and_long_dropout_becomes_lost(self):
        reference = synthetic_reference(21)
        tracker = ProgressTracker(reference, lost_timeout_s=0.50)
        running = tracker.update(HumanMotionSample(0.0, reference[6], source="synthetic"))

        short_dropout = tracker.update(
            HumanMotionSample(0.20, None, valid=False, source="synthetic")
        )
        long_dropout = tracker.update(
            HumanMotionSample(0.71, None, valid=False, source="synthetic")
        )

        self.assertEqual(short_dropout.state, MimicState.HOLDING)
        self.assertEqual(short_dropout.progress, running.progress)
        self.assertEqual(long_dropout.state, MimicState.LOST)
        self.assertEqual(long_dropout.progress, running.progress)

    def test_tracking_can_recover_after_input_was_lost(self):
        reference = synthetic_reference(21)
        tracker = ProgressTracker(reference, lost_timeout_s=0.20)
        tracker.update(HumanMotionSample(0.0, reference[4], source="synthetic"))
        tracker.update(HumanMotionSample(0.1, None, valid=False, source="synthetic"))
        lost = tracker.update(HumanMotionSample(0.31, None, valid=False, source="synthetic"))
        recovered = tracker.update(HumanMotionSample(0.4, reference[7], source="synthetic"))

        self.assertEqual(lost.state, MimicState.LOST)
        self.assertEqual(recovered.state, MimicState.RUNNING)
        self.assertAlmostEqual(recovered.progress, 0.35)

    def test_pose_far_from_reference_is_treated_as_missing(self):
        reference = synthetic_reference(21)
        tracker = ProgressTracker(reference, lost_timeout_s=0.25)
        tracker.update(HumanMotionSample(0.0, reference[3], source="synthetic"))

        holding = tracker.update(HumanMotionSample(0.1, np.full(4, 5.0), source="synthetic"))
        lost = tracker.update(HumanMotionSample(0.4, np.full(4, 5.0), source="synthetic"))

        self.assertEqual(holding.state, MimicState.HOLDING)
        self.assertEqual(lost.state, MimicState.LOST)
        self.assertLess(lost.confidence, tracker.minimum_confidence)

    def test_reset_and_completion_are_stable(self):
        reference = synthetic_reference(11)
        tracker = ProgressTracker(reference)
        complete = tracker.update(HumanMotionSample(0.0, reference[-1], source="synthetic"))
        still_complete = tracker.update(
            HumanMotionSample(2.0, None, valid=False, source="synthetic")
        )
        reset = tracker.reset()

        self.assertEqual(complete.state, MimicState.COMPLETE)
        self.assertEqual(still_complete.state, MimicState.COMPLETE)
        self.assertEqual(reset.state, MimicState.WAITING)
        self.assertEqual(reset.progress, 0.0)
        self.assertEqual(reset.active_segment, 0)

    def test_duplicate_waypoints_are_removed_but_static_reference_is_rejected(self):
        reference = synthetic_reference(5)
        repeated = np.insert(reference, 2, reference[1], axis=0)
        tracker = ProgressTracker(repeated)

        self.assertEqual(tracker.reference_rad.shape, reference.shape)
        self.assertFalse(tracker.reference_rad.flags.writeable)
        with self.assertRaisesRegex(ValueError, "contain movement"):
            ProgressTracker(np.zeros((3, 4)))

    def test_rejects_invalid_configuration_samples_and_timestamp_order(self):
        reference = synthetic_reference(5)
        invalid_options = (
            {"look_ahead_segments": 0},
            {"look_ahead_segments": 1.5},
            {"stationary_timeout_s": -1},
            {"lost_timeout_s": 0},
            {"confidence_scale_rad": 0},
            {"minimum_confidence": 1.1},
            {"progress_epsilon": 1},
        )
        for options in invalid_options:
            with self.subTest(options=options), self.assertRaises(ValueError):
                ProgressTracker(reference, **options)

        tracker = ProgressTracker(reference)
        with self.assertRaises(TypeError):
            tracker.update(reference[0])
        tracker.update(HumanMotionSample(1.0, reference[0], source="synthetic"))
        with self.assertRaisesRegex(ValueError, "timestamps"):
            tracker.update(HumanMotionSample(0.9, reference[1], source="synthetic"))


if __name__ == "__main__":
    unittest.main()
