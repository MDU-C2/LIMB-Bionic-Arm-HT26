# Majd
"""Check that training optimizations keep the original preprocessing and ranking."""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "ml"))

TORCH_AVAILABLE = importlib.util.find_spec("torch") is not None
if TORCH_AVAILABLE:
    import torch
    from train_gru import BATCH_SIZE, MovementDataset, TripletDataset, compute_top1_accuracy


@unittest.skipUnless(TORCH_AVAILABLE, "Install ML dependencies to check GRU training")
class MovementTrainingTests(unittest.TestCase):
    def test_triplet_training_rejects_a_split_without_a_different_participant(self) -> None:
        samples = [(np.zeros((60, 69)), "a"), (np.ones((60, 69)), "a")]
        for is_val in (False, True):
            with self.subTest(validation=is_val):
                with self.assertRaisesRegex(ValueError, "at least two participants"):
                    TripletDataset(SimpleNamespace(samples=samples), is_val=is_val)

    def test_triplets_use_matching_and_different_participants(self) -> None:
        samples = [(np.full((60, 69), value), user)
                   for value, user in ((0, "a"), (1, "a"), (2, "b"), (3, "b"))]
        dataset = TripletDataset(SimpleNamespace(samples=samples), is_val=True)
        for index, (anchor, positive, negative) in enumerate(dataset):
            anchor_user = samples[index][1]
            self.assertTrue(any(user == anchor_user and np.array_equal(positive.numpy(), sample)
                                for sample, user in samples))
            self.assertTrue(any(user != anchor_user and np.array_equal(negative.numpy(), sample)
                                for sample, user in samples))

    def test_centering_matches_the_original_loop(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            frame = {"shoulder": [1, 2, 3], "elbow": [4, 6, 8], "hand": []}
            path = Path(temporary) / "movement.json"
            path.write_text(json.dumps({"user_id": "1", "data": [frame]}), encoding="utf-8")
            dataset = MovementDataset(temporary, [path.name])
            original = np.array([1, 2, 3, 4, 6, 8] + [0] * 63, dtype=np.float32)
            base = original[:3].copy()
            for index in range(0, 69, 3):
                original[index : index + 3] -= base
            np.testing.assert_array_equal(dataset[0][0], np.repeat(original[None, :], 60, axis=0))

    def test_accuracy_embeds_each_recording_once_and_excludes_identical_sequences(self) -> None:
        class Model(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.sample_count = 0
                self.call_count = 0

            def forward(self, inputs):
                self.sample_count += len(inputs)
                self.call_count += 1
                return inputs[:, 0, :2]

        samples = [(np.full((60, 69), index / 100), "a") for index in range(10)]
        samples += [(np.full((60, 69), 10 + index / 100), "b") for index in range(10)]
        samples.append((samples[0][0].copy(), "other"))
        model = Model()
        dataset = SimpleNamespace(data=samples)
        actual = compute_top1_accuracy(model, dataset, "cpu")
        expected_correct = 0
        for index, (anchor, user) in enumerate(samples):
            candidates = [
                (np.linalg.norm(anchor[0, :2] - reference[0, :2]), other_user)
                for other, (reference, other_user) in enumerate(samples)
                if other != index and not np.array_equal(anchor, reference)
            ]
            expected_correct += min(candidates, key=lambda candidate: candidate[0])[1] == user
        self.assertEqual(actual, expected_correct / len(samples))
        self.assertEqual(model.sample_count, len(samples))
        self.assertEqual(model.call_count, (len(samples) + BATCH_SIZE - 1) // BATCH_SIZE)

    def test_empty_or_all_identical_recordings_have_no_reference_match(self) -> None:
        model = torch.nn.Flatten()
        self.assertEqual(compute_top1_accuracy(model, SimpleNamespace(data=[]), "cpu"), 0.0)
        sample = np.zeros((60, 69))
        data = [(sample, "a"), (sample.copy(), "b")]
        self.assertEqual(compute_top1_accuracy(model, SimpleNamespace(data=data), "cpu"), 0.0)


if __name__ == "__main__":
    unittest.main()
