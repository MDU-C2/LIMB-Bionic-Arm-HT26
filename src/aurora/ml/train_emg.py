# Majd
"""Compare EMG classifiers on held-out participants and save a model bundle."""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import platform
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

from aurora.json_utils import write_json
from aurora.ml.emg_dataset import load_trials, make_dataset, participant_splits
from aurora.ml.emg_metrics import classification_scores, grip_scores
from aurora.ml.emg_model import EmgModel, fit_lda
from aurora.paths import CHECKOUT_DIRECTORY, REPOSITORY_ROOT
from aurora.recording.emg_features import PHASE_NAMES
from aurora.recording.emg_labels import digest
from aurora.sensors.emg_protocol import EmgActivationEstimator
from aurora.sensors.emg_window import EmgWindowSettings, preprocessing
from aurora.time_utils import utc_now


def fit_model(
    x: np.ndarray,
    y: np.ndarray,
    kind: str,
    indices: list[int],
    settings: EmgWindowSettings,
    directory: Path,
    *,
    epochs: int,
    seed: int,
) -> EmgModel:
    if set(y.tolist()) != set(range(4)):
        raise ValueError(
            "Every training partition needs reviewed rest, grip, hold and release windows."
        )
    directory.mkdir(parents=True, exist_ok=False)
    selected = x[:, :, indices]
    mean = selected.reshape(-1, len(indices)).mean(axis=0)
    scale = selected.reshape(-1, len(indices)).std(axis=0)
    scale[scale < 1e-8] = 1.0
    scaled = ((selected - mean) / scale).astype(np.float32)
    weights_path = directory / "weights.npz"
    artifacts = {}
    if kind == "lda":
        weights, bias = fit_lda(scaled[:, -1], y)
        np.savez(weights_path, mean=mean, scale=scale, weights=weights, bias=bias)
    else:
        model_path = directory / "model.onnx"
        # NumPy covariance solves and PyTorch load different OpenMP runtimes on Windows.
        with tempfile.TemporaryDirectory(prefix="aurora-emg-network-") as temporary:
            dataset = Path(temporary) / "training.npz"
            np.savez(dataset, features=scaled, labels=y)
            result = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "aurora.ml.emg_network",
                    "--data",
                    str(dataset),
                    "--output",
                    str(model_path.resolve()),
                    "--kind",
                    kind,
                    "--epochs",
                    str(epochs),
                    "--seed",
                    str(seed),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            if result.returncode:
                raise RuntimeError(
                    f"{kind.upper()} training failed:\n{result.stdout}{result.stderr}"
                )
        np.savez(weights_path, mean=mean, scale=scale)
        artifacts[model_path.name] = digest(model_path)
    artifacts[weights_path.name] = digest(weights_path)
    write_json(
        directory / "model.json",
        {
            "format_version": 1,
            "task": "emg_phase_classification",
            "kind": kind,
            "reviewed_for_live": False,
            "classes": list(PHASE_NAMES),
            "sequence_length": x.shape[1],
            "feature_indices": indices,
            "preprocessing": preprocessing(settings),
            "artifacts": artifacts,
            "training": {
                "seed": seed,
                "epochs": epochs if kind != "lda" else None,
                "scaler_fit": "training_partition_only",
                "lda_shrinkage": 0.1,
                "class_weighting": "equal_priors" if kind == "lda" else "inverse_frequency",
            },
        },
    )
    return EmgModel(directory, require_review=False)


def amplitude_baseline(trials, records, rate_hz: float) -> dict:
    predictions = {}
    for trial in trials:
        metadata = trial.source.parent.parent / "trial.json"
        if not metadata.is_file():
            return {"available": False, "reason": "No saved pre-trial calibration."}
        profile = json.loads(metadata.read_text(encoding="utf-8")).get("calibration", {})
        required = ("emg_baseline_adc", "emg_rest", "emg_reference", "emg_strong")
        if any(profile.get(key) is None for key in required):
            return {"available": False, "reason": "Saved rest/strong calibration is incomplete."}
        estimator = EmgActivationEstimator()
        estimator.baseline, estimator.rest_envelope = profile[required[0]], profile[required[1]]
        estimator.reference, estimator.strong_envelope = profile[required[2]], profile[required[3]]
        estimator._energy = estimator.rest_envelope**2
        estimator._gap_settling_s = 0.25
        selected = {
            r["sample_index"] for r in records if (r["subject_id"], r["trial_id"]) == trial.key
        }
        for index, raw in enumerate(trial.raw):
            if index and not np.isclose(
                trial.times[index] - trial.times[index - 1], 1e6 / rate_hz, rtol=0.01, atol=1
            ):
                estimator.sample_gap()
            value = estimator.update(raw, 1 / rate_hz)
            if index in selected:
                if value is None:
                    return {
                        "available": False,
                        "reason": "Amplitude baseline not ready at a scored window.",
                    }
                predictions[(*trial.key, index)] = value >= 0.5
    return {
        "available": True,
        "threshold": 0.5,
        "predictions": [
            predictions[(r["subject_id"], r["trial_id"], r["sample_index"])] for r in records
        ],
    }


def code_info() -> dict:
    revision = None
    dirty = None
    if CHECKOUT_DIRECTORY:
        try:
            revision = subprocess.check_output(
                ["git", "rev-parse", "HEAD"],
                cwd=CHECKOUT_DIRECTORY,
                stderr=subprocess.DEVNULL,
                text=True,
            ).strip()
            dirty = bool(
                subprocess.check_output(
                    ["git", "status", "--porcelain"],
                    cwd=CHECKOUT_DIRECTORY,
                    stderr=subprocess.DEVNULL,
                ).strip()
            )
        except (OSError, subprocess.CalledProcessError):
            pass
    from aurora.ml import emg_dataset, emg_metrics, emg_model, emg_network
    from aurora.sensors import emg_filter, emg_window

    modules = (
        Path(__file__),
        *(
            Path(m.__file__)
            for m in (emg_dataset, emg_metrics, emg_model, emg_network, emg_filter, emg_window)
        ),
    )
    return {
        "revision": revision,
        "uncommitted_changes": dirty,
        "source_sha256": {p.name: digest(p) for p in modules},
    }


def label_coverage(trials, records) -> list[dict]:
    coverage = []
    for trial in trials:
        for segment in trial.labels["segments"]:
            count = sum(
                1
                for record in records
                if (record["subject_id"], record["trial_id"]) == trial.key
                and record["segment_start_s"] == segment["start_s"]
            )
            coverage.append(
                {
                    "subject_id": trial.subject,
                    "trial_id": trial.key[1],
                    **segment,
                    "eligible_windows": count,
                }
            )
    return coverage


def run_experiment(
    data: Path,
    output: Path,
    *,
    kinds=("lda", "ann", "lstm"),
    feature_sets=("basic", "full"),
    settings=None,
    sequence_length=4,
    calibration_trials=0,
    epochs=30,
    seed=42,
) -> dict:
    settings = settings or EmgWindowSettings()
    if epochs < 1 or epochs > 10000 or not kinds or not set(kinds) <= {"lda", "ann", "lstm"}:
        raise ValueError("Choose LDA, ANN or LSTM and 1 to 10000 training epochs.")
    if not feature_sets or not set(feature_sets) <= {"basic", "full"}:
        raise ValueError("Feature sets are basic and full.")
    trials = load_trials(data)
    folds = participant_splits(trials, calibration_trials)
    output.mkdir(parents=True, exist_ok=False)
    versions = {"python": platform.python_version()}
    for package in ("numpy", "scipy", "torch", "onnx", "onnxruntime"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    manifest = {
        "format_version": 1,
        "task": "emg_phase_classification",
        "status": "running",
        "created_utc": utc_now(),
        "split_method": "leave_one_subject_out",
        "personal_calibration_trials": calibration_trials,
        "preprocessing": preprocessing(settings),
        "settings": {
            "kinds": list(kinds),
            "feature_sets": list(feature_sets),
            "epochs": epochs,
            "seed": seed,
            "sequence_length": sequence_length,
        },
        "versions": versions,
        "code": code_info(),
        "files": [],
        "folds": [],
    }
    for trial in trials:
        manifest["files"].append(
            {
                "subject_id": trial.subject,
                "trial_id": trial.key[1],
                "raw_file": trial.source.relative_to(
                    data.parent if data.is_file() else data
                ).as_posix(),
                "raw_sha256": digest(trial.source),
                "labels_sha256": digest(trial.label_file),
                "labels_file": trial.label_file.relative_to(
                    data.parent if data.is_file() else data
                ).as_posix(),
            }
        )
    write_json(output / "training.json", manifest)
    try:
        for number, fold in enumerate(folds):
            print(f"Held-out subject {fold['subject']} ({number + 1}/{len(folds)})", flush=True)
            train_x, train_y, _train_records = make_dataset(
                fold["train"], settings, sequence_length
            )
            test_x, test_y, records = make_dataset(fold["test"], settings, sequence_length)
            result = {
                "subject_id": fold["subject"],
                "train_trials": [list(t.key) for t in fold["train"]],
                "test_trials": [list(t.key) for t in fold["test"]],
                "calibration_trials": [list(t.key) for t in fold["calibration"]],
                "train_windows": len(train_y),
                "test_windows": len(test_y),
                "test_label_coverage": label_coverage(fold["test"], records),
                "models": {},
            }
            baseline = amplitude_baseline(fold["test"], records, settings.rate_hz)
            if baseline["available"]:
                baseline["grip"] = grip_scores(
                    test_y, baseline.pop("predictions"), records, binary=True
                )
            result["calibrated_amplitude_baseline"] = baseline
            for feature_set in feature_sets:
                indices = [0, 1] if feature_set == "basic" else list(range(6))
                for kind in kinds:
                    name = f"{kind}-{feature_set}"
                    print(f"  Training {name}", flush=True)
                    directory = output / f"fold-{number + 1}" / name
                    model = fit_model(
                        train_x,
                        train_y,
                        kind,
                        indices,
                        settings,
                        directory,
                        epochs=epochs,
                        seed=seed,
                    )
                    predicted = np.argmax(model.predict(test_x), axis=1)
                    result["models"][name] = {
                        "phase": classification_scores(test_y, predicted, 4),
                        "grip": grip_scores(test_y, predicted, records),
                    }
            manifest["folds"].append(result)
            write_json(output / "training.json", manifest)
        names = list(manifest["folds"][0]["models"])
        manifest["mean_subject_macro_f1"] = {
            name: float(
                np.mean([f["models"][name]["phase"]["macro_f1"] for f in manifest["folds"]])
            )
            for name in names
        }
        # Every candidate is exported explicitly. There is no test-score-based automatic winner.
        all_x, all_y, _records = make_dataset(trials, settings, sequence_length)
        for feature_set in feature_sets:
            for kind in kinds:
                name = f"{kind}-{feature_set}"
                model = fit_model(
                    all_x,
                    all_y,
                    kind,
                    [0, 1] if feature_set == "basic" else list(range(6)),
                    settings,
                    output / "bundles" / name,
                    epochs=epochs,
                    seed=seed,
                )
                model.manifest["evaluation"] = {
                    "training_file": "../../training.json",
                    "candidate": name,
                    "fit": "all_reviewed_trials",
                }
                write_json(model.directory / "model.json", model.manifest)
        manifest["status"] = "completed"
        manifest["completed_utc"] = utc_now()
    except Exception as error:
        manifest["status"], manifest["error"] = "failed", str(error)
        raise
    finally:
        write_json(output / "training.json", manifest)
    for name in names:
        model_file = output / "bundles" / name / "model.json"
        document = json.loads(model_file.read_text(encoding="utf-8"))
        document["evaluation"]["training_sha256"] = digest(output / "training.json")
        write_json(model_file, document)
    print(f"Results: {output / 'training.json'}", flush=True)
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data", type=Path, required=True, help="Folder of reviewed .labels.json files"
    )
    parser.add_argument("--output", type=Path, help="New run folder; defaults to outputs/training")
    parser.add_argument(
        "--models", choices=("lda", "ann", "lstm"), nargs="+", default=["lda", "ann", "lstm"]
    )
    parser.add_argument(
        "--features", choices=("basic", "full"), nargs="+", default=["basic", "full"]
    )
    parser.add_argument("--rate", type=float, default=4000)
    parser.add_argument("--window-ms", type=float, default=200)
    parser.add_argument("--overlap-ms", type=float, default=50)
    parser.add_argument("--noise-threshold", type=float, default=5)
    parser.add_argument("--sequence-length", type=int, default=4)
    parser.add_argument("--calibration-trials", type=int, default=0)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    from datetime import datetime

    output = args.output or REPOSITORY_ROOT / "outputs/training" / datetime.now().strftime(
        "emg_%Y%m%d_%H%M%S_%f"
    )
    try:
        run_experiment(
            args.data.resolve(),
            output,
            kinds=args.models,
            feature_sets=args.features,
            settings=EmgWindowSettings(
                args.rate, args.window_ms, args.overlap_ms, noise_threshold_adc=args.noise_threshold
            ),
            sequence_length=args.sequence_length,
            calibration_trials=args.calibration_trials,
            epochs=args.epochs,
            seed=args.seed,
        )
    except (OSError, ValueError, ImportError, KeyError, RuntimeError) as error:
        parser.exit(1, f"EMG training: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
