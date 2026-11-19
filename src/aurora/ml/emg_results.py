# Majd
"""Plot held-out participant scores from a saved EMG training run."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from aurora.json_utils import read_json_object


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--training", type=Path, required=True, help="A completed EMG training.json"
    )
    parser.add_argument(
        "--save", type=Path, help="Image destination; defaults to the training folder"
    )
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    try:
        document = read_json_object(args.training)
        if document.get("task") != "emg_phase_classification" or document.get("status") not in (
            "complete",
            "completed",
        ):
            raise ValueError("Choose a completed EMG training run.")
        folds = document["folds"]
        if not isinstance(folds, list) or not folds:
            raise ValueError("The training run has no participant evaluation results.")
        candidates = list(folds[0]["models"])
        import matplotlib

        if not args.show:
            matplotlib.use("Agg")
        from matplotlib import pyplot as plt

        figure, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)
        position = np.arange(len(folds))
        for name in candidates:
            for axis, target in zip(axes, ("phase", "grip"), strict=True):
                axis.plot(
                    position,
                    [f["models"][name][target]["macro_f1"] for f in folds],
                    marker="o",
                    label=name,
                )
        baseline = [
            f["calibrated_amplitude_baseline"].get("grip", {}).get("macro_f1", np.nan)
            for f in folds
        ]
        if np.isfinite(baseline).any():
            axes[1].plot(
                position,
                baseline,
                marker="x",
                color="black",
                linestyle="--",
                label="Calibrated amplitude",
            )
        for axis, title in zip(axes, ("Four phases", "Open/closed grip"), strict=True):
            axis.set(
                xticks=position,
                xticklabels=[f["subject_id"] for f in folds],
                ylim=(0, 1.02),
                xlabel="Held-out subject",
                title=title,
            )
            axis.grid(alpha=0.2)
            axis.legend(fontsize=8)
        axes[0].set_ylabel("Macro F1")
        figure.suptitle(
            f"EMG participant evaluation | personal calibration trials: {document['personal_calibration_trials']}"
        )
        figure.tight_layout()
        output = args.save or args.training.parent / "comparison.png"
        output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output, dpi=160)
        if args.show:
            plt.show()
        plt.close(figure)
        print(f"Saved {output}")
    except (OSError, ValueError, KeyError, IndexError, TypeError) as error:
        parser.exit(1, f"EMG results: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
