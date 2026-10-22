# Live control and calibration

The camera estimates shoulder, elbow and wrist positions in a trunk frame.
The upper-arm and forearm IMUs provide faster motion updates. EMG supplies
one grip command after rest and firm-grip calibration. It does not measure
finger positions or shoulder angles.

## Start control

Open **Simulation → Connect + open live setup**. Movement starts paused.

1. Sit upright, shoulder relaxed, elbow bent to about 90 degrees. Rest the open hand at
   marker 1 with the thumb up. Show both shoulders, hips, elbow and wrist.
2. Press **Align starting pose**. Keep still for two seconds and wait for the
   IMU baseline. The measured starting pose maps to the simulator's tabletop pose.
3. Press **Calibrate relaxed EMG** while relaxed.
4. Press **Calibrate firm grip**, hold for three seconds, then relax.
5. Return to marker 1 and press **Start interactive control**.

First bend the elbow without lifting the upper arm, then raise the arm, move
it sideways and turn the forearm. Check each direction. If a sensor axis is
wrong, pause and correct its signed-axis setting before calibrating again.
`0x6B` belongs on the upper arm and `0x6A` on the forearm. The default Y axes
run along the arm segments. A resting pose cannot identify every mounting error.

**Pause / start setup again** and `R` pause live movement. A Bluetooth reconnect
requires calibration again. EMG and IMU streams expire independently.

With **Physics mode** off, live preview follows bounded joint targets directly.
Physics mode models gravity, contact and the configured motor speeds, so it can
lag behind a person's movement. Both modes keep robot joint limits. Live grip
secures a cup only when enough fingertips contact it; opening the hand releases it.

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

The distance between IMUs is not needed for these joint angles. Their mounting
orientation and alignment to the arm are needed. Camera points can estimate
segment lengths, but accurate centimetres require a scale reference or calibrated
depth. Integrating acceleration twice is not used to recover hand position.

## Check without hardware

Choose **Replay a recorded cup trial**, select its `trial.json`, then press
**Play recording**. Replay decodes the raw BLE packets with the saved calibration
and feeds the saved camera observations through the same alignment and fusion.
No camera inference or BLE connection is opened. The video is the saved operator
view; it is not a new tracking result.
Replay initializes the EMG filters from the saved raw samples; their initial
settling can differ from a live session that was already running before the trial.

Replay checks timing, mapping and consistency. It does not establish anatomical
accuracy. The camera estimates are not ground truth, hidden fingers can be missed,
and fast motion is limited by the recorded camera observation rate.

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
