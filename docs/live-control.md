# Live control and calibration

The camera estimates shoulder, elbow and wrist positions in a trunk frame.
The upper-arm and forearm IMUs provide faster motion updates. EMG supplies
one grip command after rest and firm-grip calibration. It does not measure
finger positions or shoulder angles.

Use the [workflow guide](workflow-guide.md#control) for illustrated preparation,
calibration reuse, pause, mounting changes and replay. This page describes the
estimator and validity rules.

## Fusion

IMU acceleration supplies mounted tilt and relative tilt supplies elbow bending.
Gyroscopes supply short-term lateral and axial motion. These are mounted-sensor
approximations: acceleration during movement and motion outside the assumed
planes can cause cross-talk. Six-axis sensors also lack an absolute heading.
Fresh camera observations therefore correct elevation, elbow, lateral motion
and the observable upper-arm rotation proxy, rather than correcting twist alone.

For a camera correction weight `w`, the update gain is
`1 - (1 - w) ** (dt / (1/30))`. Each new camera observation corrects the IMU
state once. That correction persists as later IMU packets arrive. Sensor reads
and joint updates continue between camera frames; camera inference runs separately.
Forearm axial drift still has no reliable camera correction.

Measured human angles stay intact during alignment and fusion. For example,
starting at 95° elbow flexion and extending to 85° produces a 10° change even
though the robot can only bend through 60°. Robot limits apply after alignment,
when converting the fused angles to the left-arm joint commands. This is pose
retargeting, so the robot's available range can still limit the reproduced motion.
The same rule applies to integrated shoulder and forearm twist: their state
can exceed robot travel, while the mapped commands stay within joint limits.

The anatomical estimate and the robot target are separate values. A measured
90-degree elbow remains 90 degrees; alignment stores a separate human-to-robot
reference offset. Return error compares fresh camera angles with that human
reference. Shoulder estimates without a visible trunk assume that the trunk
stays upright and still. Moving the trunk under that assumption can produce
incorrect shoulder commands; pause and restore the view.

An IMU-only diagnostic without a human pose reference reports signed changes
since zeroing in `relative_angles`. Those channels stay out of the snapshot's
anatomical `angles` until a camera or functional reference anchors them.
Zero relative elbow movement does not mean a straight human elbow. Diagnostic
retargeting can map that zero change to the current robot pose separately.

Subject recording and stereo live setup use the same functional mounting fit.
Passed rotations and gyro biases are consumed by that connection's live
estimator and by saved replay. Profiles bind the subject, arm,
mounting session and connection generation. They cannot be transferred to a new
connection as a current live calibration. The app saves distinct live attempts in
`SUBJECT/references/` under the selected recording root (standalone CLI defaults
to `outputs/calibration`), with normalized input vectors, source timestamps, stereo
points, cue windows and hashes. These inputs do not replace raw trial recordings.
Fitting and saving run outside the control loop. Failed and cancelled captures
are retained when a reference step has started.

Turning stereo depth off selects a configured-axis diagnostic setup. It requires
the mounting axes in advanced settings and a two-second still reference; it does
not fit an arbitrary mounting from a stationary pose. USB control also uses the
configured mounting convention.

IMU roles expire independently after 0.5 seconds. Each acquired gyro measurement
is integrated once, using its sensor time; held values do not become new samples.
RGB and depth must share an OAK device timestamp within 20 ms. After a short wait,
an RGB frame may be used without depth, with metric coordinates unavailable.
Gyro values at the BLE wire limit pause control and invalidate the live reference.
Their actual rotation rate is unknown. Raw recording keeps the received values
and a saturation flag; repeat calibration after checking the movement and stream.

Camera inference runs in a worker. Optional EMG model inference keeps at most
one pending feature window and counts superseded windows; raw samples are still
recorded. A model result older than 0.5 seconds does not command grip.
Camera results older than 0.5 seconds are displayed with their original receipt
time but do not correct current IMU state. Raw recording keeps received frames.
Physics advances in fixed 1/60-second steps. Telemetry distinguishes wall time,
accumulated physics time and pending simulation time when rendering falls behind.

The distance between IMUs is not needed for these joint angles. Their mounting
orientation and alignment to the arm are needed. Camera points can estimate
segment lengths, but accurate centimetres require a scale reference or calibrated
depth. Integrating acceleration twice is not used to recover hand position.

## EMG amplitude and depth

At 4 kHz, live EMG uses a causal 20–450 Hz band-pass, a 50 Hz mains notch and
an 80 ms RMS envelope. Rest sets the noise floor. The strong-grip trial fixes
the normalization range, which does not shrink during relaxation. Before this
trial the UI labels the scale as estimated. Clipped ADC samples and an inadequate
strong-grip trial are reported. Legacy low-rate streams retain envelope processing
because they cannot resolve the raw-EMG frequency band.

Rest calibration excludes the filter's initial settling transient. Activation
starts above 1.5 times the measured rest RMS (at least one ADC count). A valid
strong-grip trial must sustain a further increase of at least half the rest RMS
or one ADC count. The median contraction level fixes the scale, so a brief
movement spike does not calibrate full grip. Failure reports measured rest/grip
levels and the required level. Missing BLE samples interrupt a grip trial and
pause activation while the filter settles again; repeat calibration after the
stream becomes continuous.

Stereo depth aligns OAK monochrome-camera depth with RGB landmarks. The live
pipeline uses FAST_DENSITY without subpixel processing. RGB-only diagnostics
use estimated monocular geometry and cannot provide metric stereo points.

## Check without hardware

The [offline fusion comparison](fusion-comparison.md) checks experimental
baselines with synthetic or independently referenced planar motion. It does
not change live fusion. Optional [EMG models](emg-learning.md) can be selected
in the advanced live settings after reviewing participant results.

Choose **Replay a recorded cup trial (no sensors)**. The current trial is reused
when available; otherwise select its `trial.json`. Replay decodes the raw BLE packets with the saved calibration
and feeds the saved camera observations through the same alignment and fusion.
No camera inference or BLE connection is opened. The video is the saved operator
view; it is not a new tracking result.
Replay initializes the EMG filters from the saved raw samples; their initial
settling can differ from a live session that was already running before the trial.

Replay checks timing, mapping and consistency. It does not establish anatomical
accuracy. The camera estimates are not ground truth, hidden fingers can be missed,
and fast motion is limited by the recorded camera observation rate.
Replay rejects negative or non-finite durations and non-finite seek times before
advancing the recorded state.

## Basis

The earlier [LIMB fusion code](https://github.com/MDU-C2/LIMB-HT25/tree/main/src/data_fusion)
uses IMU prediction with visual correction. The camera geometry also retains
the earlier six-point trunk-frame approach. The present implementation uses
joint-angle correction rather than the earlier AprilTag position pipeline.
Oscar Ågren's *Personalized, Vision-Based Human-Like Robotic Arm Control on
Real-Time Embedded Systems* (2026), sections 7.3.2 and 7.4.1, describes the
camera placement and four-marker cup-transfer procedure.

[Seel, Raisch and Schauer (2014)](https://doi.org/10.3390/s140406891) explain
IMU joint-angle estimation and sensor mounting identification.
[A Practical Sensor-to-Segment Calibration Method for Upper Limb Inertial Motion Capture
in a Clinical Setting (2025)](https://pmc.ncbi.nlm.nih.gov/articles/PMC12250904/) examines
upper-limb sensor-to-segment calibration using functional movements. Their
reported accuracy does not describe this implementation.
