# Models and training

Use **Recordings → Review EMG labels** before **Compare EMG models**. This
compares LDA, ANN and LSTM with held-out participants and saves separate runs
under `outputs/training/`. The [EMG guide](../../../docs/emg-learning.md) covers
processing, evaluation, plots and explicit model selection for live use.

The camera GRU below identifies movement profiles. It is separate from EMG
phase classification. Both training workflows save `training.json`; successful
runs use status `complete`. Matplotlib caches go under `outputs/cache/`.

## Camera movement profiles

New movement captures go in `outputs/recordings/movement/`, which is ignored by
Git. Run these commands from the project environment.

Capture a movement:

```powershell
python -m aurora.recording.capture_movement --subject-id S01 --side left
```

Press Space to start and press it again to save. Files go under the subject ID,
with trial numbers `T001`, `T002` and so on. This profile recorder needs stereo
depth. Normal cup trials also use stereo for subject calibration and capture.

Profiles use RGB x/y pixels and stereo depth in metres, matching the inherited
LIMB1 format. Relative MediaPipe z is not measured depth. New files declare their
coordinate system, subject ID and frame count. Training and recognition share
the same preprocessing and reject incompatible coordinate systems.
New profiles use `format_version: 3` and `pose_frame_count`; older formats
remain readable.

Use recordings from at least two participants, with enough recordings from each
to keep both training and validation splits populated. Train the GRU model:

```powershell
python -m aurora.ml.train_gru
```

The inherited LIMB1 camera data is separate in `data/legacy/limb1/training/`.
To train with those files, use:

```powershell
python -m aurora.ml.train_gru --data data/legacy/limb1/training
```

Training saves each run in `outputs/training/DATE_TIME/`. It contains the best
checkpoint, ONNX model, metrics plot and `training.json`. The manifest keeps
input file hashes, subject IDs, the exact split, seed, settings, package versions,
code hashes and results. A run is complete only when its manifest says `complete`.
Use `--epochs`, `--seed` and `--output` to set these explicitly.
Interrupted or failed runs keep their files and record their final status.

The split keeps separate trials from each participant in training and validation.
It evaluates movement-profile identification, not EMG gesture recognition or
performance on unseen participants. The model architecture and preprocessing
remain the inherited baseline.

The app loads `artifacts/models/movement_gru.onnx`. Training leaves that model
intact. After reviewing a run, copy its ONNX model there and keep `training.json`
with the selected model as `movement_gru.json`.
Recognition checks the selected model hash and preprocessing settings before use.

Cup transfers and guided EMG trials use the Recording tab. Their saved files
have a different format and are not automatically included in GRU training.
