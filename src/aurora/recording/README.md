# Recording

Use the app's Recording tab for repeated cup transfers. Mount the camera in front
of the subject, enter their ID and connect camera and BLE. Use **Calibrate subject**
before cup trials. The guided reference fits sensor-to-arm rotations and collects
relaxed/grip EMG. Each step waits for the operator, then starts after a countdown.
Trial start requires passed checks and current stereo arm points.
The ESP32 only needs USB for power when using BLE.

Use **Practice (no files)** for an unrecorded attempt. **Start T001** begins the
three-second countdown. The operator signals the subject at **GO**. The default
eight-second recording starts at GO and saves automatically. Choose a longer
window before the next trial if needed for the full movement. The subject does not
need to see the screen.
Follow the rest/reset cue before starting again. **Restart / repeat** keeps the
old attempt and prepares another without overwriting it. Trials go under
`outputs/recordings/SUBJECT/TRIAL/`, with camera
and BLE files together. Finish the subject before entering the next ID.
The [recording guide](../../../docs/recording-guide.md) covers signal checks and saved files.

The preview marks detected cups and tracks the hand near the selected wrist.
Cup boxes, scores and hand landmarks go in `camera/pose.json`. Raw unmirrored video
and `frame_times.jsonl` are saved independently of tracking. Stereo capture also
saves lossless depth maps, their timestamps and camera intrinsics.
The table can hide the hips. Fingers hidden behind the cup can still be missed.

Choose natural, slow or fast before each trial. Follow 1 (hand start), 2 (grasp),
3 (place and release), then 4 (empty hand finish). Requested pace and recorder
cues are saved in `trial.json`. Movement events need review afterwards; the timer
does not label grasp, release or task completion. Use the
printable [cup-transfer protocol](../../../docs/cup-transfer-protocol.md).

## Grip trials

Under **Advanced recording options**, enable **Guided grip trial** and choose
the number of repeats. The operator relays the cues:
subject relaxes, closes the hand, holds, then opens it. Five repeats take about 38 seconds
after the countdown. Use duration 0 to let the BLE trial finish on its own.
If the camera is also recording with duration 0, stop it after the trial.

The BLE folder contains raw samples, `emg_trial.json` cue times and
`emg_trial_features.csv` with six time-domain features. `meta.json` includes
the rest and hold levels. Labels describe the cues; check them against the signal
before training. Keep whole recording sessions separate for training and testing.

## Process EMG

First open **Recordings**, select `trial.json` or a sensor CSV and press
**Plot signals**. Inspect raw EMG, both IMUs and saved camera angles before
using **Process EMG**. The [inspection guide](../../../docs/signal-inspection.md)
covers gaps, clocks, spectra and filter settings.

```powershell
python -m aurora.recording.emg_processing --file outputs/recordings/SESSION/emg.csv
```

This removes the offset, applies a 20-450 Hz band-pass and a 50 Hz notch, then
calculates RMS, MAV, waveform length, zero crossings, slope-sign changes and
sample variance in 200 ms windows. The source stays intact.
Samples must be regularly spaced. Gaps need checking before processing.

Use **Review EMG labels** to save checked intervals in `emg.labels.json`, then
**Compare EMG models** for participant evaluation. Raw files and original cues
stay unchanged. See [EMG learning](../../../docs/emg-learning.md).

To rebuild features from the original grip cues, run:

```powershell
python -m aurora.recording.emg_processing --file outputs/recordings/SESSION/emg.csv --labels outputs/recordings/SESSION/emg_trial.json
```

## Code

- `record_*.py`: recording commands.
- `cup_trials.py`, `record_cup_trials.py`: repeated camera and BLE cup trials.
- `cup_protocol.py`: subject preparation, countdown, recording window, practice and rest.
- `cup_input.py`: BLE, raw RGB/depth acquisition and separate optional tracking.
- `subject_calibration.py`: fit saved references and export `imu_aligned.csv`.
- `../sensors/arm_alignment.py`: isolated movement axes, mounting rotations and checks.
- Shared BLE decoding and clock mapping are in `../sensors/`.
- `emg_trials.py`: guided trials and labelled features.
- `emg_labels.py`: reviewed intervals and source validation.
- `emg_processing.py`: saved EMG processing command.
- `emg_features.py`: offline filtering, window features and labelled exports.
- `sensor_csv.py`, `saved_signals.py`: saved samples, time bases and plot data.
- `plot_sensor_recording.py`: raw and processed plots, with optional EMG spectra.
- `../sensors/clock_sync.py`: match device sample times to the PC clock.
- `oak_session.py`: camera recording files and metadata.
- `capture_movement.py`: stereo camera movement profiles for the GRU.
- Camera capture and tracking are shared in `../sensors/camera/`.

Use `synced_host_monotonic_ns` to match BLE samples to camera frames.
USB tools are kept for diagnostics and older firmware.
