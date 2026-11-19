# Majd
"""Plot saved training metrics in a process without PyTorch's native libraries."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from aurora.paths import REPOSITORY_ROOT

os.environ.setdefault("MPLCONFIGDIR", str(REPOSITORY_ROOT / "outputs" / "cache" / "matplotlib"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument(
        "--save", type=Path, help="Separate image export; defaults to the training folder"
    )
    args = parser.parse_args()
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    history = json.loads(args.manifest.read_text(encoding="utf-8"))["history"]
    losses, scores = history["validation_loss"], history["validation_top1"]
    epochs = range(1, len(losses) + 1)
    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].plot(epochs, losses, color="red", label="Validation loss")
    axes[1].plot(epochs, scores, color="blue", label="Validation top-1")
    axes[0].set(title="Validation loss", xlabel="Epoch", ylabel="Loss")
    axes[1].set(
        title="Movement-profile identification", xlabel="Epoch", ylabel="Accuracy", ylim=(0, 1.05)
    )
    for axis in axes:
        axis.grid(True, linestyle="--", alpha=0.6)
        axis.legend()
    figure.tight_layout()
    output = args.save or args.manifest.parent / "training_metrics.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output)
    print(f"Saved {output}")
    plt.close(figure)


if __name__ == "__main__":
    main()
