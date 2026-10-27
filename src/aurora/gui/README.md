# AURORA app

Run Setup once, then open Start. Ordinary launching works offline.

| Location | Use |
| --- | --- |
| Connect & inspect | Set subject and arm; inspect BLE, USB and camera signals |
| Record | Connect camera + BLE, calibrate, practise and record separate cup trials |
| Control | Keyboard control, guided live setup and recorded sensor replay |
| Review & export | Browse recordings, inspect signals, review EMG labels, process and export |
| Advanced Tools | Movement profiles, EMG models, DMP/manual joints, raw capture, diagnostics and firmware |
| Help & status | Refresh capability checks and open the guides |

Start with **Connect & inspect**. Monitors do not create recordings. Stop them or
close the camera preview before a capture or control session takes the devices.
The app hands sensor connections to the child program before opening another owner.
Subject and arm controls stay locked while recording or control runs.

Under **Record**, enter the subject ID, choose the arm and output folder, then
connect camera + BLE. Subject calibration guides mounting movements, a static
reference and EMG checks. Each step waits for **Start step**, counts down and
shows the result. Repeat the named movement when a check fails. Normal trials
require valid calibration and stereo depth. Use **Practice (no files)** first.
The default capture is eight seconds after GO with a three-second countdown.
Requested natural, slow and fast pace stay separate from verified movement labels.
Use **Finish subject** before changing participants.

Under **Control**, choose **Direct pose preview** or **Motor physics**. Follow the
mounting, relaxed EMG and deliberate grip steps before live control. Human anatomical
angles, changes since zeroing and limited robot commands are different quantities.
Direct preview follows mapped targets; motor physics adds actuator speed and dynamics.
Use R to reset. See the [simulation guide](../../../docs/simulation-guide.md) for keys.

Under **Review & export**, select a trial or raw file:

- **Plot / export image** selects signals, optional EMG spectrum and a PNG destination.
- **Process EMG** selects filter frequencies, feature windows and a new output folder.
- **Review EMG labels** opens raw intervals and available synchronized video. Save
  reviewed labels before training; operator cues alone are not reviewed labels.
- **Replay selected cup trial** uses the recorded inputs and calibration.
- **Trial / calibration summary** shows completeness, references, cues and frame counts.
- **Export recording bundle** writes a lossless ZIP with hashes. Selecting a trial
  manifest exports that trial folder; choose its subject folder to include separate
  reference sessions. Original files and incomplete status are preserved.

Processing and export destinations stay separate from the next recording's output.
**Browse last outputs in Review** opens derived results in the browser.

**Advanced Tools** contains optional workflows:

- **Movement profiles**: capture a camera profile, choose references and a reviewed
  movement model, set the recognition threshold, train a GRU and export result plots.
- **EMG models**: compare LDA/ANN/LSTM and feature sets with explicit window, sequence,
  participant calibration, epoch and seed settings. Review participant results before
  selecting a bundle for the next live session. Calibrated amplitude remains available.
- **DMP / manual joints**: fit/export trajectories, play saved or refitted DMPs, play
  compatible camera poses and open robot joint sliders. Exports retain unclipped radians;
  playback applies robot limits. The inherited angles-only DMP fit assumes one second.
- **Raw capture**: choose named BLE, camera or USB captures, guided grip repeats,
  depth/detail settings and explicit legacy compatibility where needed.
- **Diagnostics**: select BLE/USB/camera checks, live or recorded profiling, fusion
  comparisons, dynamics and offline benchmarks. Blank-image benchmarks do not measure
  camera acquisition or tracking accuracy. **USB live-control diagnostics** uses the
  earlier configured-axis align/start path; use BLE for guided functional mounting
  and deliberate grip calibration. **Control** can also save detailed physics telemetry.
- **Firmware**: build, monitor or explicitly confirm a flash. Flashing is separate
  from build and requires the supported toolchain and a selected port.

Forms check optional packages in the background and explain missing capabilities.
**Cancel** closes an unstarted form. Once running, use **Stop active program**.
Progress, errors and completion appear in Activity, with full logs in `outputs/logs`.
New training runs stay under `outputs/training`; selection never replaces bundled models.

See the [recording guide](../../../docs/recording-guide.md),
[signal inspection](../../../docs/signal-inspection.md) and
[EMG guide](../../../docs/emg-learning.md) for data and evaluation details.
