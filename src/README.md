# Source code

Run the app with `Start.bat` from the repository root. For direct launching,
select the Python environment created by Setup and run `python -m aurora.gui.app`.
Setup installs the package; scripts do not depend on the checkout's folder name
or its location on a particular computer.

The Python package is `aurora/`:

- `gui/`: the app and its controls.
- `sensors/`: shared BLE, IMU, EMG, camera tracking and fusion.
- `recording/`: sensor recording, signal processing and grip trials.
- `simulation/`: the arm, task scene and movement playback.
- `ml/`: camera movement profiles and EMG training/evaluation.

Recordings go in `outputs/recordings/`. New model runs go in `outputs/training/`;
bundled models stay in `artifacts/models/`.

## Recording flow

1. `gui/cup_recording.py` starts one `recording/record_cup_trials.py` child
   process for the subject. Commands go through JSON lines on stdin; `[CUP]`
   events update the GUI. The GUI does not own recording files or timers.
2. `recording/cup_input.py` owns the BLE reader, camera acquisition thread and
   separate optional tracking thread. Slow tracking must not stop raw capture.
   Raw queues are bounded; overruns are counted for later quality checks.
3. `recording/cup_protocol.py` times preparation, countdown, capture and rest.
   `record_cup_trials.py` applies these transitions and opens trial files at GO.
4. `recording/cup_trials.py` groups one trial's BLE and camera files.
   `recording/oak_session.py` writes RGB video, depth maps and camera timestamps.
   `sensors/ble_protocol.py` decodes and saves BLE packets and sample CSV files.
5. Saved plots, EMG processing and reviewed labels are separate operations.
   Keep raw files unchanged. Repeats get new IDs and link to earlier attempts.

Preparation fits sensor-to-arm rotations using isolated movements, static
gravity and a stereo reference. `sensors/arm_alignment.py` contains the numerical
fit; `recording/subject_calibration.py` reads its saved inputs and exports
aligned IMU copies. Normal trial start requires passed calibration checks and
current stereo arm points. The fit assumes an upright static body reference;
it does not establish anatomical accuracy or measure trunk motion independently.
Recorder cues, requested pace and capture completion are not measured movement
labels. Actual grasp/release timing needs review.
See the [recording guide](../docs/recording-guide.md) for the file fields.

## Timing and coordinates

- BLE keeps device time, notification receipt time and, when available, sample
  time mapped to the host monotonic clock. A reconnect starts a new clock fit
  and invalidates the subject reference.
- Camera host timestamps mark receipt before tracking. OAK device time helps
  match RGB and depth and differs from the ESP32 clock. Video frame index alone
  is not an acquisition timestamp.
- Saved RGB and landmarks use the raw camera view. The operator preview is
  mirrored. Monocular world landmarks are estimates; metric stereo coordinates
  require valid depth and camera intrinsics.
- Human angles remain unclipped through calibration and fusion. Robot joint
  limits apply when mapping measurements to simulation commands.

## Working on the code

Keep interface controls in `gui/`, sensor decoding/estimators in `sensors/` and
recording files/timing in `recording/`. Live control uses
`sensors/live_input.py`; cup recording uses its own acquisition path so it can
save raw data without a simulator or successful pose detection.

Run `python scripts/check_project.py` from the project environment after changes.
It compiles the code and runs each test module separately. See
[tests](../tests/README.md) for targeted checks. Software tests use generated
signals and fake devices; a real setup still needs a pilot recording and signal
inspection before collecting participants.
