# Majd, Christian
import argparse
import os
import random
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from aurora.ml.experiment import create_manifest, sha256, write_manifest
from aurora.ml.features import FEATURE_COUNT, SEQUENCE_LENGTH, load_movement, load_sequence
from aurora.paths import REPOSITORY_ROOT
from aurora.time_utils import utc_now

# Keep Matplotlib's cache inside the repository on restricted/shared machines.
os.environ.setdefault(
    "MPLCONFIGDIR",
    str(REPOSITORY_ROOT / "artifacts" / ".matplotlib"),
)

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import Dataset

SEED = 42
# Model and training settings.
INPUT_SIZE = FEATURE_COUNT
HIDDEN_SIZE = 128
EMBED_SIZE = 128
MAX_SEQ_LEN = SEQUENCE_LENGTH

BATCH_SIZE = 16
EPOCHS = int(os.environ.get("AURORA_EPOCHS", "100"))
LEARNING_RATE = 1e-4

VAL_SPLIT = 0.25
MIN_VAL_FILES = 2
PATIENCE = 75


class MovementDataset(Dataset):
    """Resample movements and center their points on the shoulder."""

    def __init__(self, folder, files):
        self.samples = []

        for file in files:
            if not file.endswith(".json"):
                continue

            path = os.path.join(folder, file)
            sequence, subject_id = load_sequence(Path(path))
            self.samples.append((sequence, subject_id))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        return self.samples[idx]


class TripletDataset(Dataset):
    """Pick matching and different users for triplet training."""

    def __init__(self, base_dataset, is_val=False):
        self.base_dataset = base_dataset
        self.data = base_dataset.samples
        self.is_val = is_val
        if len({user for _sample, user in self.data}) < 2:
            raise ValueError(
                "Triplet training needs recordings from at least two participants in each split."
            )

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        anchor, user = self.data[idx]

        positives = [x for x in self.data if x[1] == user and not np.array_equal(x[0], anchor)]
        negatives = [x for x in self.data if x[1] != user]

        # Reuse the recording when no other positive sample is available.
        if len(positives) == 0:
            positives = [x for x in self.data if x[1] == user]

        if self.is_val:
            pos_sample = positives[idx % len(positives)]
            neg_sample = negatives[idx % len(negatives)]
        else:
            pos_sample = random.choice(positives)
            neg_sample = random.choice(negatives)

        return (
            torch.tensor(anchor, dtype=torch.float32),
            torch.tensor(pos_sample[0], dtype=torch.float32),
            torch.tensor(neg_sample[0], dtype=torch.float32),
        )


class MovementGRU(nn.Module):
    """The original GRU with mean pooling and normalized embeddings."""

    def __init__(self):
        super().__init__()

        self.gru = nn.GRU(INPUT_SIZE, HIDDEN_SIZE, batch_first=True, num_layers=2, dropout=0.3)
        self.dropout = nn.Dropout(0.4)
        self.fc = nn.Linear(HIDDEN_SIZE, EMBED_SIZE)

    def forward(self, x):
        out, _ = self.gru(x)
        out = out.mean(dim=1)
        out = self.dropout(out)

        emb = self.fc(out)
        return F.normalize(emb, dim=1)


def export_to_onnx(model, save_path="movement_gru.onnx"):
    model.eval()
    dummy_input = torch.randn(1, MAX_SEQ_LEN, INPUT_SIZE).to(next(model.parameters()).device)

    torch.onnx.export(
        model,
        dummy_input,
        save_path,
        export_params=True,
        opset_version=12,
        do_constant_folding=True,
        input_names=["input"],
        output_names=["output"],
        dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}},
    )
    print(f" Successfully exported best model to ONNX: {save_path}")


def run_epoch(model, dataset, optimizer, criterion, device, training=True):
    if training:
        model.train()
    else:
        model.eval()

    total_loss = 0
    total_pos = 0
    total_neg = 0
    num_batches = 0

    with torch.set_grad_enabled(training):
        for i in range(0, len(dataset), BATCH_SIZE):
            batch = [dataset[j] for j in range(i, min(i + BATCH_SIZE, len(dataset)))]

            anchor = torch.stack([b[0] for b in batch]).to(device)
            positive = torch.stack([b[1] for b in batch]).to(device)
            negative = torch.stack([b[2] for b in batch]).to(device)

            a_emb = model(anchor)
            p_emb = model(positive)
            n_emb = model(negative)

            pos_dist = torch.norm(a_emb - p_emb, dim=1).mean().item()
            neg_dist = torch.norm(a_emb - n_emb, dim=1).mean().item()

            loss = criterion(a_emb, p_emb, n_emb)

            if training:
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
                optimizer.step()

            total_loss += loss.item()
            total_pos += pos_dist
            total_neg += neg_dist
            num_batches += 1

    return total_loss / num_batches, total_pos / num_batches, total_neg / num_batches


def compute_top1_accuracy(model, dataset, device):
    """Compare recordings after calculating each embedding once."""
    model.eval()
    correct = 0
    total = 0
    all_samples = dataset.data
    if not all_samples:
        return 0.0

    with torch.no_grad():
        batches = []
        for start in range(0, len(all_samples), BATCH_SIZE):
            batch = np.stack([sample for sample, _user in all_samples[start : start + BATCH_SIZE]])
            inputs = torch.as_tensor(batch, dtype=torch.float32, device=device)
            batches.append(model(inputs).cpu().numpy())
        embeddings = np.concatenate(batches)

    for index, (anchor, true_user) in enumerate(all_samples):
        candidates = [
            other
            for other, (reference, _user) in enumerate(all_samples)
            if other != index and not np.array_equal(anchor, reference)
        ]
        if not candidates:
            continue
        distances = np.linalg.norm(embeddings[candidates] - embeddings[index], axis=1)
        nearest = candidates[int(np.argmin(distances))]
        total += 1
        correct += all_samples[nearest][1] == true_user

    return correct / total if total > 0 else 0.0


def main(argv=None):
    parser = argparse.ArgumentParser(description="Train the camera movement-profile GRU.")
    parser.add_argument(
        "--data",
        default=os.path.join(REPOSITORY_ROOT, "outputs", "recordings", "movement"),
        help="Folder containing reviewed camera movement JSON files",
    )
    parser.add_argument("--epochs", type=int, default=EPOCHS)
    parser.add_argument("--seed", type=int, default=SEED)
    parser.add_argument(
        "--output",
        type=Path,
        default=REPOSITORY_ROOT
        / "outputs"
        / "training"
        / datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f"),
        help="New run folder for models, training.json and the metrics plot",
    )
    args = parser.parse_args(argv)
    if args.epochs < 1 or args.seed < 0 or args.seed > 2**32 - 1:
        parser.error("Use at least one epoch and a seed between 0 and 4294967295")
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Using device:", device)

    folder = os.path.abspath(args.data)
    if not os.path.isdir(folder):
        print(f"Error: Folder not found -> {folder}")
        return 1

    files = sorted(str(path.relative_to(folder)) for path in Path(folder).rglob("*.json"))

    user_files = {}
    for file in files:
        path = os.path.join(folder, file)
        try:
            _sequence, subject_id = load_movement(Path(path))
        except (ValueError, TypeError) as error:
            print(f"Invalid movement file {file}: {error}")
            return 1
        if subject_id not in user_files:
            user_files[subject_id] = []
        user_files[subject_id].append(file)

    train_files = []
    val_files = []

    for user_list in user_files.values():
        random.shuffle(user_list)
        val_count = max(MIN_VAL_FILES, int(len(user_list) * VAL_SPLIT))
        val = user_list[:val_count]
        train = user_list[val_count:]
        train_files.extend(train)
        val_files.extend(val)

    try:
        train_dataset = TripletDataset(MovementDataset(folder, train_files), is_val=False)
        val_dataset = TripletDataset(MovementDataset(folder, val_files), is_val=True)
    except ValueError as error:
        print(f"Training could not start: {error}")
        return 1

    output = args.output.resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        print("Choose a new or empty output folder; existing runs are kept unchanged.")
        return 1
    output.mkdir(parents=True, exist_ok=True)
    model_path = output / "movement_gru_best.pth"
    onnx_path = output / "movement_gru.onnx"
    manifest = create_manifest(
        Path(folder),
        {"training": train_files, "validation": val_files},
        {
            "seed": args.seed,
            "epochs_requested": args.epochs,
            "batch_size": BATCH_SIZE,
            "learning_rate": LEARNING_RATE,
            "weight_decay": 5e-4,
            "hidden_size": HIDDEN_SIZE,
            "embedding_size": EMBED_SIZE,
            "layers": 2,
            "gru_dropout": 0.3,
            "output_dropout": 0.4,
            "optimizer": "Adam",
            "loss": "TripletMarginLoss",
            "margin": 0.3,
            "swap": True,
            "patience": PATIENCE,
            "gradient_clip": 1.0,
            "validation_fraction": VAL_SPLIT,
            "minimum_validation_files_per_subject": MIN_VAL_FILES,
            "scheduler": {"name": "ReduceLROnPlateau", "factor": 0.5, "patience": 15},
            "device": str(device),
        },
    )
    write_manifest(output, manifest)
    history_val_loss = []
    history_val_acc = []
    try:
        model = MovementGRU().to(device)

        optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE, weight_decay=5e-4)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(
            optimizer, mode="min", factor=0.5, patience=15
        )
        criterion = nn.TripletMarginLoss(margin=0.3, swap=True)

        best_val_loss = float("inf")
        patience_counter = 0
        best_pos, best_neg, best_acc = 0.0, 0.0, 0.0

        print("Starting training...")

        for epoch in range(args.epochs):
            run_epoch(model, train_dataset, optimizer, criterion, device, training=True)

            val_loss, val_pos, val_neg = run_epoch(
                model, val_dataset, optimizer, criterion, device, training=False
            )

            scheduler.step(val_loss)
            val_acc = compute_top1_accuracy(model, val_dataset, device)

            history_val_loss.append(val_loss)
            history_val_acc.append(val_acc)

            print(
                f"Epoch {epoch + 1}/{args.epochs} | "
                f"Val Loss: {val_loss:.4f} | "
                f"Val Acc: {val_acc:.4f} | "
                f"Val Pos Dist: {val_pos:.4f} | "
                f"Val Neg Dist: {val_neg:.4f} | "
                f"Val Ratio: {val_pos / (val_neg + 1e-8):.4f}"
            )

            if val_loss < best_val_loss:
                best_val_loss = val_loss
                best_pos = val_pos
                best_neg = val_neg
                best_acc = val_acc
                patience_counter = 0

                torch.save(model.state_dict(), model_path)
                manifest["best_epoch"] = epoch + 1
                print("Saved best model.")
            else:
                patience_counter += 1
                print(f"No validation improvement ({patience_counter}/{PATIENCE})")

            if patience_counter >= PATIENCE:
                print("Early stopping triggered.")
                break

        print("\n===== FINAL BEST MODEL =====")
        print(f"Best validation loss: {best_val_loss:.4f}")
        print(f"Best positive distance: {best_pos:.4f}")
        print(f"Best negative distance: {best_neg:.4f}")
        print(f"Best ratio: {best_pos / (best_neg + 1e-8):.4f}")
        print(f"Best accuracy: {best_acc:.4f}")

        manifest["epochs_completed"] = len(history_val_loss)
        try:
            model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
            export_to_onnx(model, onnx_path)
        except Exception as e:
            manifest.update(status="export_failed", error=str(e), finished_utc=utc_now())
            write_manifest(output, manifest)
            print(f"Error loading best model or exporting to ONNX: {e}")
            return 1

        manifest.update(
            status="exported",
            epochs_completed=len(history_val_loss),
            results={
                "best_validation_loss": best_val_loss,
                "matching_distance": best_pos,
                "different_distance": best_neg,
                "validation_top1": best_acc,
            },
            history={"validation_loss": history_val_loss, "validation_top1": history_val_acc},
        )
        write_manifest(output, manifest)
        try:
            subprocess.run(
                [sys.executable, "-m", "aurora.ml.training_plot", str(output / "training.json")],
                cwd=REPOSITORY_ROOT,
                check=True,
            )
        except (OSError, subprocess.CalledProcessError) as error:
            manifest.update(status="plot_failed", error=str(error), finished_utc=utc_now())
            write_manifest(output, manifest)
            print(f"Could not plot training metrics: {error}")
            return 1
        manifest.update(
            status="complete",
            finished_utc=utc_now(),
            artifacts={
                path.name: sha256(path)
                for path in (model_path, onnx_path, output / "training_metrics.png")
            },
        )
        write_manifest(output, manifest)
        print(f"Run saved: {output}")
        return 0
    except KeyboardInterrupt:
        manifest.update(
            status="interrupted", finished_utc=utc_now(), epochs_completed=len(history_val_loss)
        )
        write_manifest(output, manifest)
        print(f"Training interrupted. Run files kept in {output}")
        return 130
    except Exception as error:
        manifest.update(
            status="failed",
            error=f"{type(error).__name__}: {error}",
            finished_utc=utc_now(),
            epochs_completed=len(history_val_loss),
        )
        write_manifest(output, manifest)
        print(f"Training failed: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
