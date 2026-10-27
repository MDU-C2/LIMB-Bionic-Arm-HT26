# Recording guide

Use BLE for EMG and both IMUs. Connect the OAK-D camera to the PC over USB.
The ESP32 can use a separate USB power source. Recording does not use the live
simulator and does not need an internet connection after setup.

## Cup transfers

1. Mount the camera in front of the subject. Frame both shoulders, the selected
   arm, hand and the cup's path across the 60 x 60 cm table. Keep the camera,
   chair and table positions fixed. The table can hide the hips; recording
   continues, but trunk-relative shoulder angles may be unavailable.
2. Open **Record**. Enter a subject ID such as `S01`, choose the arm and
   set the recording window. The default is **8 seconds after GO**, with a
   **3-second countdown**. Use a longer window if a practice attempt, especially
   at slow pace, needs more time. `0` selects manual stopping.
3. Stereo depth is used for subject calibration and normal cup trials. The app
   checks depth receipt and camera intrinsics before calibration. The reference
   fit checks the saved 3D arm points; a brief tracking miss does not disable
   the calibration button.
4. Press **1. Connect camera + BLE**. Wait for camera frames, both IMUs and raw
   EMG. Check **Signals** and the preview before collecting a subject.
5. Press **2. Calibrate subject**. Each step waits while the operator reads the
   instruction and positions the subject. Press **Start step** when ready.
   A three-second countdown precedes each timed movement; begin on **GO**.
   Check the result and the signal graphs. Normal trial start stays disabled
   until IMU alignment, EMG references and current stereo checks pass. The
   status identifies the step to repeat; earlier attempts remain saved.
6. Use **Practice (no files)** for a cup transfer. Confirm the pace and recording
   window, then reset the cup to 2 and the open hand to 1.
7. Press **3. Start T001**. The operator sees **3, 2, 1, GO** and gives the
   subject the start signal at GO. Trial files open at GO and save automatically
   after the recording window. The subject does not need to see the screen.
8. Follow the rest/reset countdown. Camera and BLE stay connected between
   trials. **Restart / repeat** keeps a started attempt as incomplete and
   prepares another. Cancel before GO to avoid creating files or using a trial ID.
9. Press **Finish subject** before changing the subject ID.

The preview shows the latest camera image. Tracking runs separately and its age,
hand and cup status appear below the video. Saved tracking stays attached to its
source frame. Camera transfer uses NV12 to keep RGB and aligned depth within the
USB 2 bandwidth budget; the image size and requested frame rate stay 640 x 480
at 25 frames/s. A USB 3 data connection gives more transfer capacity.

Use the printable [subject instructions](cup-transfer-protocol.md). Collect
natural, slow and fast as separate trials. The initial collection setting is
three trials per pace with at least 20 seconds of rest. Rotate the pace order
between subjects: natural/slow/fast, slow/fast/natural, fast/natural/slow.
Keep the chosen order, duration and repetitions consistent and record any changes.
The [related work](../research/related-work.md#recording-protocols) explains the basis.

The movement is hand at 1 -> grasp at 2 -> place and release at 3 -> empty hand
at 4. Reaching 4 ends the subject's movement. The recording timer only ends the
capture window; it does not verify that the task was completed. If the hand
reaches 4 early, the subject stays there until the recording ends. Increase the
window before collecting if the full movement does not fit comfortably.

**Stop early** saves a partial timed trial as incomplete. In manual mode,
**Stop and save** ends the recording after the movement. Practice uses the same
countdown and window without saving files. There are no marker buttons in the
normal flow. Check actual movement events afterwards using the video and signals.

Normal recording needs fresh RGB, raw EMG, both IMUs, a passed subject
calibration and current stereo points for both shoulders, elbow and wrist at
start. Hand/finger and cup detection are not start requirements. Once recording
has begun, available raw streams keep saving through tracking loss. Missing
streams or depth maps flag the attempt for review. Keep unsuccessful attempts.

## Subject calibration

Fix one IMU on the outside of the upper arm and one on the forearm above the
wrist. Straps must prevent slipping. The fit estimates the sensor mounting
rotations; fixed printed-axis alignment is not assumed. IMU spacing is not
needed for angles. Segment lengths are needed for forward kinematics, and
metric camera coordinates need calibrated depth.

The operator guides the following steps after a three-second countdown. The
subject remains upright, with the other arm beside the body. The camera sees
both shoulders and the selected elbow/wrist. Hips may be behind the table.
Keep all reference movements clear of the tabletop. Do not move the camera.

| Step | Duration | Subject position or action |
| --- | --- | --- |
| Still | 5 s | Upper arm beside body, elbow about 90 degrees, forearm forward, thumb up. |
| Raise | 10 s | Hold elbow angle and thumb-up position. Raise forward first, then lower; repeat slowly. |
| Bend | 10 s | Keep upper arm still; bend the elbow further first, then return. Repeat slowly, thumb up. |
| Turn | 10 s | Keep upper arm/elbow still. Turn palm down first, then up; repeat comfortably. |
| Return | 5 s | Restore the original pose and remain still. |
| Relax | 5 s | Support forearm and relax the open hand. |
| Grip | 5 s | Maintain a firm, comfortable grip on the empty cup, forearm supported. |
| Release | 3 s | Release and relax the open hand. |

There is no time limit for reading or positioning between steps. These pauses
and the step countdowns are saved but excluded from the alignment fit. The
operator positions the cup for the EMG step, then restores it to marker 2.
A still pose alone does
not establish the mounting rotation; all movement steps are needed.

The fit uses isolated gyro axes plus the still gravity direction to construct
proper three-dimensional sensor-to-segment rotations. A stereo reference checks
the starting pose and movement directions. The upper-arm frame assumes the
upper arm hangs beside an upright body during the still reference. The method
is a constrained project implementation, not the three-IMU/magnetometer method
in the literature. Its checks do not establish anatomical accuracy.

Checks cover sample continuity, stillness, movement range, dominant-axis quality,
axis consistency, stereo observations and the return pose. Failed checks name
the movement to repeat. These thresholds are initial collection settings to
evaluate in the pilot; they are not published clinical acceptance thresholds.

References go in `SUBJECT/references/R001_DATE_TIME/`. They contain raw signals,
video, depth, actual cue timestamps and `alignment.json`. This file keeps passed
or failed status, mounting matrices, gyro biases, checks, settings, source hashes
and code hashes. `reference_checks` also keeps IMU zero and EMG results. References which
fail remain available for inspection but do not enable normal trials.

Trials link the subject reference and carry a copy of its alignment profile.
`sensor_to_arm_alignment: fitted` means a numerical fit passed these collection
checks. `accuracy_validated: false` records the remaining need for physical
validation. A new subject, moved sensor or BLE reconnect needs a new calibration.
Moving the camera also requires a new reference. The fit does not independently
track the trunk or resolve six-axis heading drift during later motion.

For diagnostics only, the command-line recorder accepts `--raw-only`, optionally
with `--no-depth`. This explicitly bypasses subject alignment; it is not the
normal participant collection workflow. It still saves all available raw data
and its calibration status. Existing raw-only recordings remain readable.

## Saved files

```text
outputs/recordings/S01/T001_DATE_TIME/
    trial.json
    ble/
        emg.csv
        imu.csv
        imu_aligned.csv         # derived segment-frame copy after passed alignment
        packets.jsonl
        meta.json
    camera/
        video.mp4
        frame_times.jsonl
        depth/                  # stereo capture only, lossless uint16 PNG maps
        depth_times.jsonl        # stereo capture only
        pose.json
        meta.json
```

`trial.json` keeps subject, arm, trial ID, requested pace, timing settings,
reference link, alignment profile, calibration, stream health and completion status. Sources start
sequentially at the software GO transition and keep their actual timestamps;
opening their files does not establish simultaneous acquisition.
If a stream stops, available streams keep recording. Detected interruptions mark
the attempt incomplete. File-write errors stop the attempt. `complete` describes
capture completion, not task success. New trials keep `movement_labels: not_reviewed`,
`task_completion_verified: false` and `quality_review_required: true`.

Guided cup trials use `protocol_version: 3`. Calibrated captures use
`recording_mode: raw_with_alignment`; diagnostic captures use `raw_signals`.
`movement_go_host_monotonic_ns` marks the software GO cue.
`recording_ended_host_monotonic_ns` marks capture end.
`movement_duration_seconds` is time recorded after GO, not measured movement time.
`cue_events` saves recorder cues with `delivery: operator`; countdown cues have
negative `time_s` because they precede trial-file creation. The times do not
measure audio onset or the subject's response. `operator_events` is empty in the
normal flow. Older trials may contain manual markers; those remain readable.
`protocol.repeat_of` links repeated attempts. A cancelled countdown has no files;
started, interrupted or repeated attempts stay on disk. Older recordings are unchanged.

Raw BLE files contain received EMG ADC samples, both IMUs' acceleration and
angular velocity, and the original packet bytes. `imu_aligned.csv` rotates acceleration and
bias-corrected gyro into segment coordinates: X positive flexion, Y towards the
hand, Z = X cross Y. It uses g and degrees/s, preserves packet/timestamp fields,
and does not contain joint angles. The raw file remains unchanged.
Nominal high-rate firmware sends
EMG at 4,000 samples/s and each IMU at about 100 samples/s. Review actual periods,
missing samples, sequence gaps and decode errors in the recording metadata.

`video.mp4` is unmirrored RGB with lossy MP4 compression. The operator preview is
mirrored and annotated. `frame_times.jsonl` maps each received RGB frame to host
receipt time, source/device sequence numbers, OAK device time and encoded video
indices/copies. Fixed-rate video can repeat frames to approximate playback time;
use the timestamp mapping for analysis.

With stereo enabled, `depth/` keeps each received aligned depth map losslessly
in millimetres, including zero/invalid values. `depth_times.jsonl` maps files to
host receipt and OAK device timestamps. Camera metadata stores RGB intrinsics
from the device calibration. RGB and depth share the OAK clock. Optional tracking
uses the closest available depth map within 20 ms and records the time difference;
this is a matching tolerance, not a claim of exact simultaneous exposure.
Unmatched maps are still saved for later processing. OAK device time and ESP32
device time are different clock domains and must not be compared directly.

`pose.json` contains tracking observations, all detected pose landmarks in
MediaPipe index order, available hand landmarks, cup boxes/scores and estimated
angles. Monocular world landmarks are model estimates, not measured depth.
Elbow flexion needs shoulder, elbow and wrist. Full trunk-relative shoulder
angles need visible trunk reference points; hidden hips leave those estimates
unavailable. `data` contains available depth-backed points with RGB x/y pixels
and `depth_m` in metres; `keypoints_camera_m` uses calibrated camera coordinates.
Hand landmark z in observations is relative model output, not stereo depth.

New camera files use `format_version: 3`. `source_frame_count` counts received
RGB frames written to video, `video_frame_count` counts encoded copies,
`observation_frame_count` counts tracking observations, `pose_frame_count` counts
depth-backed pose entries, and `depth_frame_count` counts saved maps. Trial counts
use the `camera_` prefix. Tracking can skip frames and can be rerun from video.
`capture_health` records bounded-queue overruns, camera sequence gaps and errors.
Older camera recordings may contain mirrored, annotated video; inspect
`video_kind`, `video_mirrored` and coordinate notes before processing.

For BLE/camera alignment use BLE `synced_host_monotonic_ns` and camera
`host_monotonic_ns`. Camera host times mark frame receipt before tracking in
all recorders; `timestamp_reference` records that meaning. BLE clock-sync metadata
includes estimated uncertainty. These are host-aligned times, not proof of
simultaneous sensor sampling. After a BLE reconnect, `packets.jsonl` records
`connection_generation` and metadata keeps `time_sync_epochs`. Synced sample
times stay empty until the new clock fit is available. Review epochs separately.

## Inspect before processing

Open **Review & export**, select a trial's `trial.json`, then press **Plot / export image**.
This shows raw EMG, both IMUs and saved camera angle estimates. Use the plot
toolbar to zoom and pan. Selecting `emg.csv`, `imu.csv` or `pose.json` plots
that source alone. Missing samples break the lines. Operator marker times are
shown when the saved clocks can be aligned.

Check the relaxed level, contraction changes, clipping and gaps before applying
filters. Press **Process EMG** to save filtered samples, six-feature windows and
processing settings in a new folder selected in the form. Use **Browse last outputs in Review** and select the generated
CSV to plot it. Raw files stay unchanged. See [signal inspection](signal-inspection.md)
for the spectrum and filter settings.

Use **Review EMG labels** to check phase boundaries against the raw signal and
video, then **Advanced Tools → EMG models → Compare EMG models** for participant evaluation. See
[EMG learning](emg-learning.md) for labels, model settings and selection.

## Advanced recordings

Open **Advanced Tools → Raw capture** for individual sources, camera depth/detail
settings or guided grip trials. USB monitoring is under **Connect & inspect**.
The existing recorders still
save separate source folders with shared subject and session metadata.

For a guided grip trial, select Bluetooth and EMG, enable **Guided grip trial**
and set duration to `0`. Five repeats take about 38 seconds. The operator relays
the relax, close, hold and open cues. Stop a separate camera recorder afterward.

The BLE folder adds `emg_trial.json` cue times and `emg_trial_features.csv`
with six time-domain features. Cue times are labels to review against the
actual movement. Cup transfers have a trial-level task name; grasp and release
phases are not automatically labelled.

Older recordings can contain operator marker observations. These do not establish
exact grasp and release times. Assign movement labels only after checking the
video and signals. Keep participants separate when evaluating a model on new people.

Process a saved EMG file from the project environment:

```powershell
python -m aurora.recording.emg_processing --file outputs/recordings/S01/TRIAL/ble/emg.csv
```

This preserves the raw file and exports six filtered-signal features. Samples
must be regularly spaced; review gaps before processing. To rebuild cue-derived
features, add `--labels PATH/emg_trial.json`. Use **Review EMG labels** for
separate reviewed training labels.

Finish recording before starting live simulation. It uses the same camera and
BLE device. See [troubleshooting](troubleshooting-guide.md) for connection problems
and [sensor setup](sensor-data.md) for wiring.
