# Source code

Run the app with `Start.bat` from the repository root.

The Python package is `aurora/`:

- `gui/`: the app and its controls.
- `sensors/`: shared BLE, IMU, EMG, camera tracking and fusion.
- `recording/`: sensor recording, signal processing and grip trials.
- `simulation/`: the arm, task scene and movement playback.
- `ml/`: movement preprocessing, training and recognition.

Recordings go in `outputs/recordings/`. Models go in `artifacts/models/`.
