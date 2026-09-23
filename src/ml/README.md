# Movement model

This folder contains the movement-data capture and GRU training code. It was
moved from the Applied Artificial Intelligence student project.

The current dataset is in `data/movement/training/`.

## Capture a movement

```powershell
micromamba run -n aurora-simulation python src/ml/capture_movement.py --user-id 6 --side right
```

Press `Space` to start and press it again to save.

## Train the model

```powershell
micromamba run -n aurora-simulation python -m pip install -r src/ml/requirements.txt
micromamba run -n aurora-simulation python src/ml/train_gru.py
```

Training output is written below `artifacts/models/`. The model and dataset are
experimental and are not a medical or identification result.
