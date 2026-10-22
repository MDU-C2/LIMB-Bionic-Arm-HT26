# Movement training

New movement captures go in `outputs/recordings/movement/`, which is ignored by
Git. Run these commands from the project environment.

Capture a movement:

```powershell
python src/ml/capture_movement.py --user-id 1 --side right
```

Press Space to start and press it again to save.

Use recordings from at least two participants, with enough recordings from each
to keep both training and validation splits populated. Train the GRU model:

```powershell
python src/ml/train_gru.py
```

The inherited LIMB1 camera data is separate in `data/legacy/limb1/training/`.
To train with those files, use:

```powershell
python src/ml/train_gru.py --data data/legacy/limb1/training
```

Training saves the best model and its ONNX export in `artifacts/models/`.
The app loads `movement_gru.onnx` from that folder.

Cup transfers and guided EMG trials use the Recording tab. Their saved files
have a different format and are not automatically included in GRU training.
