import os
import json
import random
import numpy as np

# Keep Matplotlib's cache inside the repository on restricted/shared machines.
os.environ.setdefault(
    "MPLCONFIGDIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "artifacts", ".matplotlib"),
)

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import Dataset

# Importera matplotlib för att kunna rita graferna
import matplotlib.pyplot as plt

SEED = 42
random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED)
if torch.cuda.is_available():
    torch.cuda.manual_seed_all(SEED)

# =========================
# CONFIG
# =========================
INPUT_SIZE = 69
HIDDEN_SIZE = 128
EMBED_SIZE = 128
MAX_SEQ_LEN = 60

BATCH_SIZE = 16
EPOCHS = int(os.environ.get("LIMB_EPOCHS", "100"))
LEARNING_RATE = 1e-4

VAL_SPLIT = 0.25
MIN_VAL_FILES = 2
PATIENCE = 75

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPOSITORY_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
OUTPUT_DIR = os.path.join(REPOSITORY_ROOT, "artifacts", "models")
ONNX_PATH = os.path.join(OUTPUT_DIR, "movement_gru.onnx")
MODEL_PATH = os.path.join(OUTPUT_DIR, "movement_gru_best.pth")


# =========================
# FRAME -> VECTOR
# =========================
def frame_to_vector(frame):
    vec = []
    for joint in ["shoulder", "elbow"]:
        vec.extend(frame.get(joint, [0.0, 0.0, 0.0]))

    hand = frame.get("hand", [])
    hand_map = {p["id"]: p for p in hand}

    for i in range(21):
        if i in hand_map:
            p = hand_map[i]
            vec.extend([p["x"], p["y"], p["depth_m"]])
        else:
            vec.extend([0.0, 0.0, 0.0])

    return np.array(vec, dtype=np.float32)


# =========================
# LOAD JSON
# =========================
def load_sequence(path):
    with open(path, "r") as f:
        raw = json.load(f)

    user_id = raw["user_id"]
    sequence_num = raw["sequence"]
    data = raw["data"]

    seq = [frame_to_vector(frame) for frame in data]
    seq = np.stack(seq)

    return seq, user_id, sequence_num


# =========================
# DATASET
# =========================
class MovementDataset(Dataset):
    def __init__(self, folder, files):
        self.samples = []

        for file in files:
            if not file.endswith(".json"):
                continue

            path = os.path.join(folder, file)
            seq, user_id, sequence_num = load_sequence(path)
            
            # Tids-interpolation till exakt MAX_SEQ_LEN
            num_frames = seq.shape[0]
            num_features = seq.shape[1]
            
            if num_frames > 1:
                current_indices = np.linspace(0, num_frames - 1, num_frames)
                target_indices = np.linspace(0, num_frames - 1, MAX_SEQ_LEN)
                
                resampled_chunk = np.zeros((MAX_SEQ_LEN, num_features))
                for f in range(num_features):
                    resampled_chunk[:, f] = np.interp(target_indices, current_indices, seq[:, f])
            else:
                resampled_chunk = np.repeat(seq, MAX_SEQ_LEN, axis=0)

            # Rums-centrering utifrån axeln (index 0, 1, 2)
            centered_chunk = resampled_chunk.copy()
            for t in range(len(centered_chunk)):
                if np.all(centered_chunk[t] == 0): 
                    continue
                base_x, base_y, base_z = centered_chunk[t, 0], centered_chunk[t, 1], centered_chunk[t, 2]
                for p in range(0, 69, 3):
                    centered_chunk[t, p] -= base_x
                    centered_chunk[t, p+1] -= base_y
                    centered_chunk[t, p+2] -= base_z

            self.samples.append((centered_chunk, user_id))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        return self.samples[idx]


# =========================
# TRIPLET DATASET
# =========================
class TripletDataset(Dataset):
    def __init__(self, base_dataset, is_val=False):
        self.base_dataset = base_dataset
        self.data = base_dataset.samples
        self.is_val = is_val

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        anchor, user = self.data[idx]
        
        positives = [x for x in self.data if x[1] == user and not np.array_equal(x[0], anchor)]
        negatives = [x for x in self.data if x[1] != user]
        
        # Säkerhetsspärr om en användare bara har en sekvens
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
            torch.tensor(neg_sample[0], dtype=torch.float32)
        )


# =========================
# MODEL
# =========================
class MovementGRU(nn.Module):
    def __init__(self):
        super().__init__()

        self.gru = nn.GRU(
            INPUT_SIZE,
            HIDDEN_SIZE,
            batch_first=True,
            num_layers=2,
            dropout=0.3
        )
        self.dropout = nn.Dropout(0.4)
        self.fc = nn.Linear(HIDDEN_SIZE, EMBED_SIZE)

    def forward(self, x):
        out, _ = self.gru(x)
        out = out.mean(dim=1)  # Mean pooling över tidsaxeln
        out = self.dropout(out)
        
        emb = self.fc(out)
        return F.normalize(emb, dim=1)


# =========================
# EXPORT ONNX
# =========================
def export_to_onnx(model, save_path="movement_gru.onnx"):
    model.eval()
    dummy_input = torch.randn(1, MAX_SEQ_LEN, 69).to(next(model.parameters()).device)
    
    torch.onnx.export(
        model,
        dummy_input,
        save_path,
        export_params=True,
        opset_version=12,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={
            'input': {0: 'batch_size'},
            'output': {0: 'batch_size'}
        }
    )
    print(f" Successfully exported best model to ONNX: {save_path}")


# =========================
# RUN EPOCH
# =========================
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


# =========================
# COMPUTE ACCURACY (1-NN)
# =========================
def compute_top1_accuracy(model, dataset, device):
    model.eval()
    correct = 0
    total = 0
    all_samples = dataset.data

    with torch.no_grad():
        for i in range(len(all_samples)):
            anchor, true_user = all_samples[i]

            anchor_tensor = torch.tensor(anchor, dtype=torch.float32).unsqueeze(0).to(device)
            anchor_emb = model(anchor_tensor).cpu().numpy()[0]

            best_user = None
            best_dist = float("inf")

            for j in range(len(all_samples)):
                ref, ref_user = all_samples[j]
                
                if i == j or np.array_equal(anchor, ref):
                    continue

                ref_tensor = torch.tensor(ref, dtype=torch.float32).unsqueeze(0).to(device)
                ref_emb = model(ref_tensor).cpu().numpy()[0]

                dist = np.linalg.norm(anchor_emb - ref_emb)

                if dist < best_dist:
                    best_dist = dist
                    best_user = ref_user

            if best_user is not None:
                total += 1
                if best_user == true_user:
                    correct += 1

    return correct / total if total > 0 else 0.0


# =========================
# MAIN TRAINING LOOP
# =========================
def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Using device:", device)

    folder = os.path.join(REPOSITORY_ROOT, "data", "movement", "training")
    if not os.path.exists(folder):
        print(f"Error: Folder not found -> {folder}")
        return

    files = [f for f in os.listdir(folder) if f.endswith(".json")]

    user_files = {}
    for file in files:
        path = os.path.join(folder, file)
        with open(path, "r") as f:
            raw = json.load(f)
        user_id = raw["user_id"]
        if user_id not in user_files:
            user_files[user_id] = []
        user_files[user_id].append(file)

    train_files = []
    val_files = []

    for user_id, user_list in user_files.items():
        random.shuffle(user_list)
        val_count = max(MIN_VAL_FILES, int(len(user_list) * VAL_SPLIT))
        val = user_list[:val_count]
        train = user_list[val_count:]
        train_files.extend(train)
        val_files.extend(val)

    train_dataset = TripletDataset(MovementDataset(folder, train_files), is_val=False)
    val_dataset = TripletDataset(MovementDataset(folder, val_files), is_val=True)

    if len(train_dataset) == 0 or len(val_dataset) == 0:
        print("Error: No samples found in train or validation dataset.")
        return

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    model = MovementGRU().to(device)

    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE, weight_decay=5e-4)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=15)
    criterion = nn.TripletMarginLoss(margin=0.3, swap=True)

    best_val_loss = float("inf")
    patience_counter = 0
    best_pos, best_neg, best_acc = 0.0, 0.0, 0.0

    # -------------------------------------------------------------
    # NYTT: Listor för att lagra historiken till plotten
    # -------------------------------------------------------------
    history_val_loss = []
    history_val_acc = []

    print("Starting training...")

    for epoch in range(EPOCHS):
        train_loss, train_pos, train_neg = run_epoch(
            model, train_dataset, optimizer, criterion, device, training=True
        )

        val_loss, val_pos, val_neg = run_epoch(
            model, val_dataset, optimizer, criterion, device, training=False
        )
        
        scheduler.step(val_loss)
        val_acc = compute_top1_accuracy(model, val_dataset, device)
        
        # Spara undan värden för denna epok till historiken
        history_val_loss.append(val_loss)
        history_val_acc.append(val_acc)
        
        print(
            f"Epoch {epoch+1}/{EPOCHS} | "
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

            torch.save(model.state_dict(), MODEL_PATH)
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

    try:
        model.load_state_dict(torch.load(MODEL_PATH, map_location=device, weights_only=True))
        export_to_onnx(model, ONNX_PATH)
    except Exception as e:
        print(f"Error loading best model or exporting to ONNX: {e}")

    # =============================================================
    # NYTT: RITA UT OCH SPARA HISTORIKEN
    # =============================================================
    epochs_range = range(1, len(history_val_loss) + 1)

    plt.figure(figsize=(12, 5))

    # Graf 1: Validation Loss
    plt.subplot(1, 2, 1)
    plt.plot(epochs_range, history_val_loss, color='red', label='Val Loss', linewidth=2)
    plt.title('Validation Loss over Epochs')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()

    # Graf 2: Validation Accuracy
    plt.subplot(1, 2, 2)
    plt.plot(epochs_range, history_val_acc, color='blue', label='Val Accuracy', linewidth=2)
    plt.title('Validation Accuracy over Epochs')
    plt.xlabel('Epochs')
    plt.ylabel('Accuracy (0.0 - 1.0)')
    plt.ylim(0, 1.05)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.legend()

    plt.tight_layout()
    
    # Sparar ner diagrammet som en PNG-fil i samma mapp
    plot_path = os.path.join(OUTPUT_DIR, "training_metrics.png")
    plt.savefig(plot_path)
    print(f"\n Metrics plot saved to: {plot_path}")
    
    # Öppnar upp fönstret på skärmen
    plt.close()


if __name__ == "__main__":
    main()
