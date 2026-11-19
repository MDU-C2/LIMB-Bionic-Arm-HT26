# Majd
"""Train one neural EMG candidate in its own process."""

from __future__ import annotations

import argparse
from pathlib import Path

from aurora.recording.common import enable_ctrl_break_as_interrupt


def train(data: Path, output: Path, kind: str, epochs: int, seed: int) -> None:
    import numpy as np
    import torch
    from torch import nn

    with np.load(data, allow_pickle=False) as dataset:
        x, y = dataset["features"], dataset["labels"]
    if (
        x.ndim != 3
        or len(x) != len(y)
        or x.dtype != np.float32
        or y.dtype != np.int64
        or not np.isfinite(x).all()
        or set(y.tolist()) != set(range(4))
    ):
        raise ValueError(
            "Training needs finite scaled feature sequences and all four phase labels."
        )
    torch.manual_seed(seed)
    torch.set_num_threads(1)

    class Network(nn.Module):
        def __init__(self):
            super().__init__()
            if kind == "lstm":
                self.recurrent = nn.LSTM(x.shape[2], 32, batch_first=True)
                self.output = nn.Linear(32, 4)
            else:
                self.output = nn.Sequential(nn.Linear(x.shape[2], 32), nn.Tanh(), nn.Linear(32, 4))

        def forward(self, features):
            if kind == "lstm":
                hidden, _state = self.recurrent(features)
                return self.output(hidden[:, -1])
            return self.output(features[:, -1])

    network = Network()
    optimizer = torch.optim.Adam(network.parameters(), lr=0.001, weight_decay=0.0001)
    counts = np.bincount(y, minlength=4)
    loss = nn.CrossEntropyLoss(weight=torch.tensor(len(y) / (4 * counts), dtype=torch.float32))
    values, labels = torch.from_numpy(x), torch.from_numpy(y)
    generator = torch.Generator().manual_seed(seed)
    network.train()
    for _epoch in range(epochs):
        for batch in torch.randperm(len(y), generator=generator).split(64):
            optimizer.zero_grad()
            error = loss(network(values[batch]), labels[batch])
            error.backward()
            torch.nn.utils.clip_grad_norm_(network.parameters(), 1.0)
            optimizer.step()
    network.eval()
    torch.onnx.export(
        network,
        values[:1],
        str(output),
        input_names=["features"],
        output_names=["logits"],
        opset_version=17,
        dynamic_axes={"features": {0: "batch"}, "logits": {0: "batch"}},
    )
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(
        str(output), sess_options=options, providers=["CPUExecutionProvider"]
    )
    for count in (1, min(8, len(x))):
        with torch.no_grad():
            expected = network(values[:count]).numpy()
        actual = session.run(None, {"features": x[:count]})[0]
        np.testing.assert_allclose(actual, expected, rtol=1e-4, atol=1e-5)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True, help="Scaled feature and label NPZ")
    parser.add_argument("--output", type=Path, required=True, help="New ONNX model path")
    parser.add_argument("--kind", choices=("ann", "lstm"), required=True)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    enable_ctrl_break_as_interrupt()
    if args.output.exists() or not 1 <= args.epochs <= 10000:
        parser.error("Use a new model path and 1 to 10000 epochs.")
    try:
        train(args.data, args.output, args.kind, args.epochs, args.seed)
    except (OSError, ValueError, ImportError) as error:
        parser.exit(1, f"EMG neural training: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
