# Recording formats and quality

Use the [workflow guide](workflow-guide.md) for the operator sequence, visual
preparation steps, trial timing, processing and export. This page describes the
saved data and numerical reference fit.

## Reference fitting

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

## Raw diagnostics and derived data

**Advanced Tools ? Raw capture** keeps individual BLE, camera and USB entry points
available. Sources save separate folders with shared subject/session metadata.
Guided grip captures add `emg_trial.json` cues and `emg_trial_features.csv`.
Cues and older operator marker observations need review against actual movement.

[Signal inspection](signal-inspection.md) describes plotting, sample-gap rejection
and EMG filtering. [EMG learning](emg-learning.md) describes reviewed labels and
participant evaluation. Processing creates derived files and preserves raw input.
