# Recording

Use the app's Recording tab. Select Bluetooth for EMG and IMUs, and the OAK-D
camera when needed. The ESP32 only needs USB for power when using BLE.

Enter the subject and trial IDs, then press **Start selected sources**.
Each source saves its own folder under `outputs/recordings/`.
The [recording guide](../../docs/RECORDING_GUIDE.md) covers signal checks and saved files.

## Grip trials

Enable **Guided grip trial** and choose the number of repeats. Follow the cues:
relax, close the hand, hold, then open it. Five repeats take about 38 seconds
after the countdown. Use duration 0 to let the BLE trial finish on its own.
If the camera is also recording with duration 0, stop it after the trial.

The BLE folder contains raw samples, `emg_trial.json` cue times and
`emg_trial_features.csv` with RMS and mean absolute value. `meta.json` includes
the rest and hold levels. Labels describe the cues; check them against the signal
before training. Keep whole recording sessions separate for training and testing.

## Process EMG

```powershell
python src/recording/emg_processing.py --file outputs/recordings/SESSION/emg.csv
```

This removes the offset, applies a 20-450 Hz band-pass and a 50 Hz notch, then
calculates RMS and mean absolute value in 200 ms windows. The source stays intact.
Samples must be regularly spaced. Gaps need checking before processing.

To correct grip labels, edit the phase start/end seconds in `emg_trial.json`, then run:

```powershell
python src/recording/emg_processing.py --file outputs/recordings/SESSION/emg.csv --labels outputs/recordings/SESSION/emg_trial.json
```

## Code

- `record_*.py`: recording commands.
- `ble_packets.py`: BLE decoding and raw files.
- `emg_trials.py`: guided trials and labelled features.
- `emg_filter.py`, `emg_protocol.py`, `emg_processing.py`: live and saved EMG processing.
- `clock_sync.py`: match device sample times to the PC clock.
- `oak_*.py`: camera capture, tracking and saved files.

Use `synced_host_monotonic_ns` to match BLE samples to camera frames.
USB tools are kept for diagnostics and older firmware.
