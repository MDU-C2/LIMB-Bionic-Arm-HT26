# Sensor data and experiment guide

This guide compares the 2025 LIMB system-design report with the later
`Sensor Information.pdf` notes. The notes describe proposed 2026 experiments;
they are not confirmed device specifications or calibrated control settings.

## What the streams can answer

| Stream | Save or inspect | Useful question |
| --- | --- | --- |
| Cuff EMG | Raw ADC values, channel, packet sequence and time | Did muscle activity increase before a grip? What are the rest and grip RMS distributions? |
| Cuff IMU | Acceleration and angular velocity for each available sensor channel | Did the user move the upper or lower arm, and in which direction? |
| Piezo | Raw cuff piezo samples | Did mechanical contact or movement coincide with EMG changes? |
| OAK-D | Original camera video and six arm/trunk keypoints; optional stereo depth | What elbow and shoulder angles occurred during the mug task? |
| Robot pressure | Hand contact/force, if the robot firmware exposes it | Was the object actually contacted or slipping? This is **not** currently in the BLE or OAK-D recorder. |

The 2025 report describes a 4 kHz EMG stream, 100 Hz IMU stream, and 1 kHz
piezo stream packed into 10 ms BLE notifications. The corresponding characteristic
sizes are 174, 38, and 34 bytes: a 14-byte header plus 160, 24, and 20 bytes of
sensor data. The BLE decoder supports these layouts and some older packet
revisions. It saves the original bytes so an uncertain firmware revision can be
decoded again. Check actual packet sizes and rates in a preview before relying
on any displayed value.

The report says the final **human cuff** used one EMG and one IMU, although its
buffers retained space for two of each. The later sensor notes propose an IMU
on each of the upper and lower arm. The preview displays channel 2 when packets
contain it; a second trace does not prove a second sensor is installed.

## Camera points and mug task

Oscar Ågren's 2026 demonstration pipeline used **six** human keypoints: left
shoulder, left elbow, left wrist, right shoulder, and both hips. The shoulder
and hip points define a trunk coordinate frame. From this, his method derives
elbow flexion, shoulder flexion, shoulder abduction, and a forearm-based proxy
for shoulder lateral/medial rotation. The proxy is not a direct measurement of
humeral axial rotation. Wrist and finger motion were outside his study.

The current live view draws only the selected shoulder, elbow, and wrist. It
uses all six points internally and saves them under
`pose.json.observations[*].keypoints_2d`, along with the four estimated angles
when available. The default angle source is MediaPipe's **monocular world-pose
estimate**, labelled as such in the preview and JSON; do not treat these values
as measured stereo coordinates. Keep both shoulders, both hips, the selected
arm, and the mug in the full camera frame for a seated trial. The displayed
and saved video keeps the original camera orientation. Tracking and saved
image coordinates use that same camera frame.

Oscar's experiment used 640 x 480 RGB at 25 fps, a 3 x 3 median depth patch,
and 0.5 pose confidence thresholds. The recorder now uses that frame size,
capture rate, depth patch, and confidence level. His offline experiment used
the heavy Pose Landmarker; this live view defaults to MediaPipe pose complexity
1 for responsiveness, with `--pose-model 2` available for a heavier model.
With `--depth`, the recorder attempts RGB-aligned stereo measurements,
deprojects the six points using camera intrinsics, and computes stereo-based
angles when all six depths are valid. Complete shoulder/elbow/wrist depths also
enter the existing playback `data` format. This stereo mode still needs a
device retest after DepthAI stream errors.

The **2025 LIMB report** describes a separate vision task: YOLOv6 detects the
cup, stereo depth estimates its 3D position, and a TAG16H5 AprilTag plus PnP
estimates the robot pose. It did not use the cup box to calculate the human
elbow or shoulder angles. Those cup-model and robot-tag assets are not in this
repository, so the current camera does not claim to recognize the mug.

## Preview before recording

In **Sensors**, choose **Open live (no recording)** for BLE or serial. Choose
**Open camera** for OAK-D. In **Recording**, selecting OAK-D also gives one
**Open camera** action. The camera opens without creating a session folder; click
**START REC** in the camera window or press R to begin, and click **STOP REC**
or press R again to save while keeping the live view open. Q closes the camera.
Close the BLE window or use **Stop active program** to stop the other sources.

- BLE shows recent raw EMG ADC samples, centered RMS over the latest 400 samples
  per channel, decoded IMU acceleration/gyro values, acceleration magnitude,
  piezo mean, packet rates, sequence-gap counts, and stale-stream status. These are diagnostic
  displays, not calibrated grip decisions. IMU scaling depends on firmware;
  treat the displayed numbers as device-scaled values until verified.
- OAK-D runs color and MediaPipe pose tracking in the live view. It shows only
  the selected arm's shoulder, elbow, and wrist markers (left by default), plus
  useful joint angles. Frame the seated subject, both shoulders, both hips,
  selected arm, and mug. If the camera points at a ceiling, it reports no person.
  The camera picture keeps its original orientation, and labels are drawn on
  that frame so the text remains readable. It saves nothing until REC.
  The `--depth` option attempts RGB-aligned stereo depth and adds depth-backed
  points for simulator playback. The connected OAK-D returned DepthAI stream
  errors during the stereo check, so 3D capture still needs a device retest.
- The 2025 report used a YOLOv6 cup detector and an AprilTag on the robot for
  camera-to-robot pose estimation. Neither the trained cup model nor a robot
  tag/calibration configuration is supplied in this repository. The live view
  therefore marks cup detection unavailable instead of implying that a body
  landmark is a mug detection. Its optional depth-backed arm points are for
  analysis and pose playback, not robot-frame grasp points.
- Serial preview prints incoming lines in Activity without writing a file.

Direct commands from the repository root:

```powershell
micromamba run -n aurora-simulation python src/recording/record_ble_sensors.py --preview --device LIMBServer
micromamba run -n aurora-simulation python src/recording/record_oak_pose.py --preview
micromamba run -n aurora-simulation python src/recording/record_serial_sensors.py --preview --port COM5 --baud 115200
```

## A useful first data collection

Assign a subject ID, trial ID, and test type in **Recording**. The sensor notes
recommend recording all available sensors during each trial and repeating each
test several times:

1. **Grip/EMG:** hold the arm still; 2 s REST, 4 s GRIP, 2 s REST. This isolates
   baseline noise, grip activation, and EMG changes caused only by the hand.
2. **Movement/IMU:** keep the hand relaxed; REST, a defined arm movement, REST.
   Compare IMU motion and any EMG contamination from arm movement.
3. **Combined:** REST, REACH, GRIP, MOVE/HOLD, RELEASE, REST. This checks event
   order and whether vision and EMG agree with movement and contact.

The BLE recorder writes `packets.jsonl`, `emg.csv`, `imu.csv`, `piezo.csv`, and
`meta.json`. CSV rows carry UTC host arrival time, host monotonic nanoseconds,
packet sequence, device timestamp, sensor index, and sample index. The host
timestamp is shared by all samples in a BLE packet; it is **not** a separate
4 kHz timestamp for each EMG sample. The device timestamp unit and clock offset
must be checked against current firmware before precise cross-device alignment.
`meta.json` includes the subject, trial ID, test type, duration, and packet
counts. OAK-D recording writes `video.mp4`, `pose.json`, and `meta.json`. In
the default RGB mode, `pose.json` contains 2D observations but its 3D playback
`data` list is empty.
The current control center manages one child program at a time. One BLE session
does capture EMG, IMU, and piezo together, but concurrent BLE and OAK-D capture
requires starting separate terminal processes. Matching trial IDs alone does
not synchronize their clocks; record a shared visible event and validate the
offset before doing sensor fusion.

## Analysis and control parameters

For offline EMG analysis, the sensor notes suggest removing the mean, applying
a fourth-order 20–1000 Hz Butterworth bandpass to 4 kHz samples, and computing
moving RMS. Compare rest and grip distributions and inspect clipping, packet
gaps, baseline drift, and 50 Hz interference. The report tested four 8 s grip
trials and found a clear increase in EMG activity; those results do not set a
threshold for a new wearer or electrode placement. The notes propose placing
electrodes over the flexor digitorum superficialis and keeping placement
consistent across trials.

Choose `T_grip` from the recorded rest/grip distributions. Use a lower
`T_release` for hysteresis; the notes propose a 200 ms sustained low-RMS period
before release. These are **candidate** settings requiring calibration and
false-trigger testing. They are not hard-coded into the simulator or hardware
control. EMG should indicate intent; the IMU gives short-term movement cues,
while vision gives object/pose context and robot pressure confirms contact.
Integrating acceleration twice as a stand-alone position estimate will drift.

The 2025 report's vision pipeline used cup detection and AprilTags, and its
IMU direction method was experimental. The current repository's OAK-D recorder
captures six selected arm/trunk landmarks and angle estimates even when stereo depth is
missing; only frames with all three arm depths enter the playback `data` list.
It does **not** implement the report's cup detector, AprilTag calibration,
pressure reflex, or closed-loop sensor fusion.
The report explicitly says full closed-loop integration was not validated.
