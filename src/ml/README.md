# Movement training

The training data is in `data/movement/training/`.
Run these commands from the project environment.

Capture a movement:

```powershell
python src/ml/capture_movement.py --user-id 6 --side right
```

Press Space to start and press it again to save.

Use recordings from at least two participants, with enough recordings from each
to keep both training and validation splits populated. Train the GRU model:

```powershell
python src/ml/train_gru.py
```

Training saves the best model and its ONNX export in `artifacts/models/`.
The app loads `movement_gru.onnx` from that folder.

For labelled EMG trials, use **Guided grip trial** under Recording.
These recordings are separate from the movement dataset.
