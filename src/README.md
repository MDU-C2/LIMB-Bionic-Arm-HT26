# Source code

Run the app with `Start.bat` from the repository root.

The Python package is `aurora/`:

- `gui/`: the app and its controls.
- `sensors/`: shared BLE, IMU, EMG, camera tracking and fusion.
- `recording/`: sensor recording, signal processing and grip trials.
- `simulation/`: the arm, task scene and movement playback.
- `ml/`: camera movement profiles and EMG training/evaluation.

Recordings go in `outputs/recordings/`. New model runs go in `outputs/training/`;
bundled models stay in `artifacts/models/`.
