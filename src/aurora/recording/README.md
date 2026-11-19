# Recording

Follow the [workflow guide](../../../docs/workflow-guide.md) for collection, review
and processing. See [recording formats](../../../docs/recording-guide.md) for raw
files, calibration provenance, timing and completeness.

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
