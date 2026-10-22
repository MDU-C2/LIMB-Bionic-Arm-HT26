# Majd
"""Check participant isolation, label integrity and live feature parity."""

from __future__ import annotations

import csv
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from aurora.json_utils import write_json
from aurora.ml.emg_dataset import load_trials, make_dataset, participant_splits, trial_windows
from aurora.ml.emg_metrics import classification_scores, grip_scores
from aurora.ml.emg_model import EmgModel, LiveEmgClassifier
from aurora.ml.train_emg import run_experiment
from aurora.recording.emg_labels import digest, load_labels, save_labels
from aurora.sensors.ble_input import BleSensorReader
from aurora.sensors.emg_window import EmgWindowSettings, EmgWindowStream, features


def recording(root, subject="S01", trial="T01", seed=42):
    directory = root / subject / trial
    directory.mkdir(parents=True)
    random = np.random.default_rng(seed)
    time = np.arange(5001) / 1000
    phase = np.minimum((time // 1).astype(int), 4)
    amplitude = np.asarray([3, 25, 55, 15, 3])[phase]
    raw = np.round(
        2048 + amplitude * (np.sin(2 * np.pi * 83 * time) + random.normal(0, 0.2, len(time)))
    )
    source = directory / "emg.csv"
    with source.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("device_time", "channel", "value"))
        writer.writerows(zip(time * 1e6, np.zeros(len(time), dtype=int), raw, strict=True))
    segments = [
        {"start_s": index, "end_s": index + 1, "phase": name}
        for index, name in enumerate(("rest", "grip", "hold", "release", "rest"))
    ]
    labels = save_labels(source, subject, trial, segments, reviewed=True)
    return source, labels


class EmgLearningTests(unittest.TestCase):
    def test_reviewed_labels_are_separate_and_bind_to_raw_samples(self):
        with tempfile.TemporaryDirectory() as temp:
            source, labels = recording(Path(temp))
            original = source.read_bytes()
            document, _, _, _ = load_labels(labels)
            self.assertTrue(document["reviewed"])
            self.assertEqual(source.read_bytes(), original)
            document["reviewed"] = False
            write_json(labels, document)
            with self.assertRaisesRegex(ValueError, "Review"):
                load_labels(labels)
            document["reviewed"] = True
            write_json(labels, document)
            source.write_bytes(original + b"\n")
            with self.assertRaisesRegex(ValueError, "match the raw"):
                load_labels(labels)

    def test_invalid_overlapping_labels_and_source_overwrite_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            source, _labels = recording(Path(temp))
            with self.assertRaises(ValueError):
                save_labels(
                    source,
                    "S01",
                    "T01",
                    [{"phase": "rest", "start_s": 0, "end_s": 6}],
                    reviewed=True,
                )
            with self.assertRaises(ValueError):
                save_labels(
                    source,
                    "S01",
                    "T01",
                    [{"phase": "rest", "start_s": 0, "end_s": 1}],
                    reviewed=True,
                    destination=source,
                )

    def test_participants_and_calibration_trials_are_split_before_windows(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for subject in ("S01", "S02", "S03"):
                for trial in ("T01", "T02"):
                    recording(root, subject, trial, seed=len(list(root.rglob("emg.csv"))))
            trials = load_trials(root)
            for fold in participant_splits(trials):
                self.assertNotIn(fold["subject"], {trial.subject for trial in fold["train"]})
                self.assertEqual({trial.subject for trial in fold["test"]}, {fold["subject"]})
            for fold in participant_splits(trials, 1):
                self.assertFalse({t.key for t in fold["train"]} & {t.key for t in fold["test"]})
                self.assertEqual(len(fold["calibration"]), 1)
            with self.assertRaises(ValueError):
                participant_splits(trials, 2)

    def test_causal_features_match_streaming_and_reset_after_gaps(self):
        with tempfile.TemporaryDirectory() as temp:
            source, _ = recording(Path(temp))
            trial = load_trials(source.parent)[0]
            settings = EmgWindowSettings(rate_hz=1000)
            x, _y, records = trial_windows(trial, settings, 2)
            stream = EmgWindowStream(settings)
            history, found = [], {}
            for index, raw in enumerate(trial.raw):
                emitted = stream.update(raw)
                if emitted is not None:
                    history.append(emitted[1])
                    if len(history) >= 2:
                        found[index] = np.asarray(history[-2:])
            for vector, record in zip(x, records, strict=True):
                np.testing.assert_array_equal(vector, found[record["sample_index"]])
            trial.times[2500:] += 100000
            _x, _y, records = trial_windows(trial, settings, 2)
            self.assertFalse(any(2500 <= r["sample_index"] < 3099 for r in records))
            with self.assertRaisesRegex(ValueError, "sample times"):
                trial_windows(trial, EmgWindowSettings(), 2)

    def test_clipping_restarts_prediction_history(self):
        stream = EmgWindowStream(EmgWindowSettings(rate_hz=1000))
        emissions = [stream.update(4095) for _ in range(800)]
        self.assertTrue(any(value is not None and value[1] is None for value in emissions))
        stream.reset()
        self.assertTrue(all(stream.update(2048) is None for _ in range(449)))

    def test_feature_values_and_noise_threshold(self):
        result = features(np.asarray([-2, 2, -2, 2], dtype=float), 1)
        np.testing.assert_allclose(result, [2, 2, 12, 3, 2, 16 / 3])
        self.assertEqual(features(np.asarray([-2, 2, -2, 2]), 5)[3], 0)

    def test_lda_experiment_freezes_train_scaling_and_requires_explicit_review(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "data"
            for number in range(3):
                recording(root, f"S0{number + 1}", seed=number)
            output = Path(temp) / "run"
            manifest = run_experiment(
                root,
                output,
                kinds=("lda",),
                feature_sets=("basic", "full"),
                settings=EmgWindowSettings(rate_hz=1000),
                sequence_length=2,
                epochs=1,
            )
            self.assertEqual(manifest["status"], "complete")
            self.assertEqual(len(manifest["folds"]), 3)
            self.assertFalse(manifest["folds"][0]["calibrated_amplitude_baseline"]["available"])
            train = participant_splits(load_trials(root))[0]["train"]
            x, _y, _records = make_dataset(train, EmgWindowSettings(rate_hz=1000), 2)
            with np.load(output / "fold-1/lda-full/weights.npz") as weights:
                np.testing.assert_allclose(weights["mean"], x.reshape(-1, 6).mean(axis=0))
            bundle = output / "bundles/lda-full"
            with self.assertRaisesRegex(ValueError, "reviewed"):
                EmgModel(bundle)
            info = json.loads((bundle / "model.json").read_text(encoding="utf-8"))
            self.assertEqual(
                info["evaluation"]["training_sha256"], digest(output / "training.json")
            )
            info["reviewed_for_live"] = True
            write_json(bundle / "model.json", info)
            model = EmgModel(bundle)
            for changes in (
                {"feature_indices": [0.5, 1]},
                {"sequence_length": True},
                {"artifacts": []},
            ):
                write_json(bundle / "model.json", {**info, **changes})
                with self.assertRaises(ValueError):
                    EmgModel(bundle)
            write_json(bundle / "model.json", info)
            live = LiveEmgClassifier(bundle)
            trial = load_trials(root)[0]
            for raw in trial.raw:
                live.update(raw)
            self.assertIn(live.phase, info["classes"])
            self.assertAlmostEqual(float(model.predict(x[:1]).sum()), 1)
            live.update(float("nan"))
            self.assertIsNone(live.phase)
            live.reset()
            self.assertIsNone(live.grip(0.7))
            weights = bundle / "weights.npz"
            weights.write_bytes(weights.read_bytes() + b"changed")
            with self.assertRaisesRegex(ValueError, "hash"):
                EmgModel(bundle)

    def test_metrics_keep_missing_classes_and_count_false_state_changes(self):
        score = classification_scores([0, 0], [0, 0], 4)
        self.assertEqual(score["support"], [2, 0, 0, 0])
        self.assertEqual(score["macro_f1"], 0.25)
        records = [
            {"subject_id": "S01", "trial_id": "T01", "end_s": i * 0.15, "segment_start_s": 0}
            for i in range(4)
        ]
        score = grip_scores([0, 0, 2, 2], [0, 1, 2, 0], records)
        self.assertEqual(score["false_closures"], 1)
        self.assertEqual(score["false_releases"], 1)

    def test_event_counts_use_the_configured_step_and_ignore_new_streams(self):
        records = [
            {
                "subject_id": "S01",
                "trial_id": "T01",
                "end_s": i * 0.3,
                "segment_start_s": 0,
                "stream_start_index": 0,
            }
            for i in range(3)
        ]
        score = grip_scores([0, 0, 2], [0, 1, 0], records, step_s=0.3)
        self.assertEqual(score["false_closures"], 1)
        self.assertEqual(score["false_releases"], 1)
        records[1]["stream_start_index"] = records[2]["stream_start_index"] = 1000
        score = grip_scores([0, 0, 2], [0, 1, 0], records, step_s=0.3)
        self.assertEqual(score["false_closures"], 0)
        self.assertEqual(score["false_releases"], 1)

    def test_score_validation_rejects_invalid_or_missing_predictions(self):
        for actual, predicted in (([], []), ([0, 1], [0]), ([0], [-1]), ([4], [0])):
            with self.subTest(actual=actual, predicted=predicted), self.assertRaises(ValueError):
                classification_scores(actual, predicted, 4)

    def test_labels_reject_invalid_objects_sources_and_timestamps(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, label_file = recording(Path(temporary))
            original = json.loads(label_file.read_text(encoding="utf-8"))
            for name in ("../emg.csv", str(source.resolve()), "folder\\emg.csv", None):
                write_json(label_file, {**original, "source": name})
                with self.assertRaisesRegex(ValueError, "source beside"):
                    load_labels(label_file)
            write_json(label_file, [])
            with self.assertRaisesRegex(ValueError, "JSON object"):
                load_labels(label_file)
            source.write_text(
                "device_time,channel,value\n1000,0,2048\n0,0,2048\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(ValueError, "increasing"):
                save_labels(source, "S01", "T01", original["segments"], reviewed=True)

    def test_subject_and_trial_case_aliases_do_not_enter_different_partitions(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            recording(root, "S01", "T01", seed=1)
            _source, labels = recording(root, "S01", "T02", seed=2)
            document = json.loads(labels.read_text(encoding="utf-8"))
            write_json(labels, {**document, "subject_id": "s01"})
            with self.assertRaisesRegex(ValueError, "same subject ID"):
                load_trials(root)
            write_json(labels, {**document, "trial_id": "t01"})
            with self.assertRaisesRegex(ValueError, "only once"):
                load_trials(root)

    def test_interrupted_training_saves_status_without_touching_sources(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            sources = [recording(root / "data", f"S0{i + 1}", seed=i)[0] for i in range(2)]
            hashes = [digest(source) for source in sources]
            with (
                patch("aurora.ml.train_emg.make_dataset", side_effect=KeyboardInterrupt),
                self.assertRaises(KeyboardInterrupt),
            ):
                run_experiment(
                    root / "data",
                    root / "run",
                    kinds=("lda",),
                    feature_sets=("full",),
                    settings=EmgWindowSettings(rate_hz=1000),
                )
            document = json.loads((root / "run/training.json").read_text(encoding="utf-8"))
            self.assertEqual(document["status"], "interrupted")
            self.assertIn("finished_utc", document)
            self.assertEqual(hashes, [digest(source) for source in sources])

    def test_incomplete_model_finalisation_cannot_leave_a_complete_run(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for index in range(2):
                recording(root / "data", f"S0{index + 1}", seed=index)

            def save(path, document):
                if path.name == "model.json" and "training_sha256" in document.get(
                    "evaluation", {}
                ):
                    raise OSError("Disk write interrupted")
                write_json(path, document)

            with (
                patch("aurora.ml.train_emg.write_json", side_effect=save),
                self.assertRaises(OSError),
            ):
                run_experiment(
                    root / "data",
                    root / "run",
                    kinds=("lda",),
                    feature_sets=("full",),
                    settings=EmgWindowSettings(rate_hz=1000),
                    sequence_length=2,
                )
            document = json.loads((root / "run/training.json").read_text(encoding="utf-8"))
            self.assertEqual(document["status"], "failed")

    def test_live_model_waits_for_prediction_and_expires_with_emg(self):
        from aurora.sensors.live_input import LiveSensorInput

        clock = [10.0]
        source = LiveSensorInput(emg_model=Path("selected-bundle"), clock=lambda: clock[0])
        source.emg_time, source.emg_activation = clock[0], 0.9
        self.assertIsNone(source._snapshot().hand_curl)
        source.emg_phase, source.model_grip = "hold", 0.9
        self.assertEqual(source._snapshot().hand_curl, 0.9)
        source.emg_phase, source.model_grip = "rest", 0.0
        self.assertEqual(source._snapshot().hand_curl, 0)
        clock[0] += 0.6
        self.assertIsNone(source._snapshot().hand_curl)

    def test_ble_gap_and_recalibration_clear_model_predictions(self):
        from unittest.mock import Mock

        reader = BleSensorReader()
        reader.emg_classifier = Mock()
        reader.emg_classifier.model.settings = EmgWindowSettings(rate_hz=1000)
        reader._emg_period = 0.001
        reader._samples("emg", 1, [[2048] * 10])
        reader._samples("emg", 3, [[2048] * 10])
        reader.emg_classifier.reset.assert_called()
        reader.emg_classifier.reset.reset_mock()
        reader.calibrate_emg()
        reader.emg_classifier.reset.assert_called_once()
        reader.emg_classifier.reset.reset_mock()
        reader._clear_state()
        reader.emg_classifier.reset.assert_called_once()


if __name__ == "__main__":
    unittest.main()
