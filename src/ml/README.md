# Movement model training

This is the minimal training subset migrated from `KarlFallman/LIMB1`. It trains
a GRU embedding model that learns which recorded body/hand movements belong to
the same person. The included dataset contains 100 JSON recordings from five
users; it contains landmarks and depth values, not images.

From the repository root, install the ML-only dependencies and start training:

```powershell
python -m pip install -r src/ml/requirements.txt
python src/ml/train_gru.py
```

The best checkpoint, an ONNX export, and the metrics plot are written under
`artifacts/models/`. Training automatically uses CUDA when it is available.
For a quick smoke test, set the epoch count through an environment variable:

```powershell
$env:LIMB_EPOCHS = 1
python src/ml/train_gru.py
```

Expected JSON fields are `user_id`, `sequence`, and `data`. Each frame in
`data` has `shoulder`, `elbow`, and up to 21 `hand` landmarks with `id`, `x`,
`y`, and `depth_m`.
