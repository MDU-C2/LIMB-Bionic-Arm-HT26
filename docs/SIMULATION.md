# Simulation

Start the GUI and open **Simulation**:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

**Start with keyboard** opens the PyBullet task scene, the compact keyboard
controller, and the separate sensor display.

| Action | Keys |
| --- | --- |
| Shoulder up/down | W / S |
| Shoulder left/right | A / D (A left, D right) |
| Upper-arm rotation | Q / E |
| Elbow bend/extend | Up / Down |
| Wrist rotation | Left / Right |
| Close/open fingers | F / G |
| Reach, grab, or release | Space |
| Move toward the target | H |
| Change camera | C or Tab |
| Show camera panels | P |
| Show sensor display | I |
| Recalibrate live IMUs | K (live mode) |
| Start live feedback after matching reference | L (live mode) |
| Reset and return to neutral tabletop reference | R |
| Quit | Escape |

The GUI can start the simulator in direct mode or physics mode. Physics mode
uses gravity, motor control, collision, and simulated torque/contact values.
The arm is a complete left/right reflection, including the wrist and fingers,
mounted to the left of the centered table. See [Joint limits](JOINT_LIMITS.md)
for the source of every movement range and the camera-to-CAD zero conversion.

**Start live interactive control** opens the same task scene together with an
annotated OAK-D camera monitor and a separate live sensor monitor. A connected
shoulder IMU supplies mounted Y/Z tilt and gyro-Z left/right motion. The wrist
IMU adds relative elbow motion when present; a wrist-only IMU is monitored but
is not mistaken for an upper-arm sensor. Camera tracking supplies absolute pose,
axial-rotation correction, and finger curl. The result passes through the
existing joint and speed limits. Live control connects to the **BLE device**
name (default `LIMBServer`); no COM port is needed. The GUI releases its BLE
monitor automatically; close the standalone camera preview before starting.
The simulator reconnects if Bluetooth drops
and stops using sensor values after half a second without updates. **Grip source**
defaults to **Camera**, so opening and closing the tracked hand works while
electrodes are unattached. Select **EMG** once electrodes are attached and the
rest/contraction signals have been checked. **Auto** retains EMG-first selection,
with camera fallback only when EMG is unavailable; receiving EMG packets does
not establish electrode contact. A missing selected grip source holds the last
finger pose. F/G still provides manual opening and closing.

Upper-arm axial rotation now follows the upper-arm IMU, with camera correction
when a bent elbow is visible. It continues from the calibrated gyro estimate
when camera axial tracking is missing. Forearm/wrist twist uses both IMUs and
removes upper-arm motion, accounting for the measured elbow bend. Wrist twist
has no camera correction in the old mapping and can drift during longer trials.
Q/E and Left/Right provide manual rotation when the respective live target is
unavailable. Keyboard mode retains full manual control.
Shoulder left/right is a separate channel, driven by camera abduction and the
shoulder IMU's gyro-Z estimate. Keep the chosen arm and hips in view.

Before trying this on a person, follow the mounting check below. Loose boards
can verify sensor reception, but cannot establish correct anatomical movement.

### Mounting and twist calibration

1. Fix IMU1 (`0x6B`) to the upper arm and IMU2 (`0x6A`) to the forearm, with
   consistent board orientation. The default assumes each board's **Y axis runs
   along the limb**, with matching positive directions and bend axes.
2. Simulation's **Upper-arm twist axis** and **Forearm twist axis** select the
   longitudinal axes. Use a negative axis when its direction needs reversing.
   These selections remap both acceleration and gyro vectors for live control;
   recordings keep their original sensor axes. Defaults preserve the existing
   Y/Z tilt mapping. An X-length mounting uses sensor Y for the bend axis;
   Y/Z-length mountings use sensor X for the bend axis.
3. Match the neutral tabletop reference, press **L**, and hold both sensors still for
   **0.5 seconds**. Moving restarts the twist baseline. K recalibrates the current
   pose; R returns to alignment. Recalibration/reconnection resets twist references.
4. Test small isolated movements: elbow bend, upper-arm twist, then forearm
   pronation/supination with the upper arm still. Check direction, unwanted movement
   in other joints, and stops before attempting combined movements. If directions
   are wrong, stop control, correct the axes/mounting, restart and recalibrate.
5. Repeat with a bent elbow and during a short camera occlusion. The mapping uses
   HT25's relative orientation approach and needs this physical check; it is not
   a validated full anatomical model. Forearm twist starts at the old firmware's
   zero command and is limited to 0–140 degrees, so only the positive direction
   moves from zero. A middle reference offset has not been invented.

The hand uses a shared grip value, rather than mirroring each finger separately.
Live grip follows the final HT25 finger-servo speed limits; a full close/open
takes about 2.25 seconds. The CAD finger coupling is an approximate visual pose,
not a measured tendon model.

BLE live control also opens a graph window with raw EMG and acceleration/gyro
traces for both IMUs. It shares the control connection and keeps bounded sample
buffers, without saving files. EMG displays the most recent 800 received samples
(about 200 ms at 4 kHz), and each IMU displays up to 300 samples. The plots redraw
at most 20 times per second; acquisition still processes every received sample.
Streams stop displaying after half a second without data. Packet gap counters
remain visible because missing samples are not represented as time gaps on the
plots. Use **Show live BLE graphs** in the sensor monitor to reopen the window.
Enable EMG auto-scale to inspect small signals, checking the ADC axis values
because noise is enlarged too.

The BLE reader processes every EMG sample and IMU packet in the background,
using the existing rest-baseline/envelope and arm-angle filters. Camera rendering
reads the latest result. Keep the muscle relaxed during the first half-second
after calibration. Live control does not create a sensor recording.

For EMG grip, select **EMG** before starting and calibrate in the simulator's
live graph window:

1. Press **1. Calibrate EMG at rest** and relax the monitored muscle for at least
   half a second.
2. Press **2. Calibrate strong grip (3 s)** and hold a strong, comfortable grip
   until the status says **Grip calibrated**.
3. Relax to open the simulated hand and contract to close it. Match the neutral
   tabletop pose and press **L** to enable feedback. L/K align the arm sensors;
   they preserve the EMG calibration.

Repeat both EMG steps after starting a new simulator or moving electrodes. The
standalone monitor and simulator have separate acquisition sessions. For grip,
electrodes need to observe a forearm muscle involved in gripping; upper-arm EMG
can rise with elbow movement without a grip. One EMG channel estimates muscle
activation, not measured grip force or individual finger intent.

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

**Stereo depth** uses the OAK's two monochrome cameras to measure distance and
align it with RGB landmarks. Without it, arm geometry uses MediaPipe's estimated
3D pose. The live pipeline uses the lighter FAST_DENSITY preset without subpixel
processing; the default DepthAI v3 preset caused X_LINK_ERROR on the connected
RVC2 OAK-D. RGB-only remains available by clearing **Use stereo depth**.

```bash
python src/simulation/ai/live_sensor_fusion.py --device LIMBServer
```

`--shoulder-twist-axis y --wrist-twist-axis y` selects the default mounting;
each option accepts `x`, `y`, `z`, `-x`, `-y`, or `-z`.
`--grip-source emg` selects EMG grip; `--grip-source auto` selects EMG-first
fallback. `--address` can select a known BLE address. Older `--port COM5` commands retain
serial control for firmware explicitly built with USB transport. BLE packet
decoding, fusion, and connection cleanup are checked offline; ESP32/OAK-D live
operation passed a short connected check; muscle signals and tracked-arm accuracy
still need validation. See [Hardware validation](HARDWARE_VALIDATION.md).

**Open camera monitor** in the GUI header shows live OAK-D tracking without
recording. Close that standalone monitor before starting live interactive
control because only one process can own the camera.

The **Motion AI** tab can play compatible OAK-D pose recordings and run the
experimental movement-recognition model.

The tabletop defaults to **60 × 60 cm**, with the cup placed above its surface.
Change **Tabletop width × depth (cm)** in Simulation to match another surface;
the keyboard and live simulator use the same dimensions. CLI equivalents are
`--table-width-cm 60 --table-depth-cm 60`.

The table is centered on the torso's world-X centerline. The left shoulder is
mounted outside the table's left edge, angled inward, so a forward lift beside
the table clears it with either a straight or bent elbow. Lift before swinging
over the tabletop. The cup starts on the left side of the table, to the right of
the arm; the green disc marks its destination farther right. Grip, carry and
release the cup using the existing controls. Reach assistance uses the mounted
shoulder's coordinate frame. In physics mode, open the fingers and tilt or
withdraw the hand when placing the cup so it can settle onto the table.

Live control begins paused with the shoulder and elbow at 45 degrees and the
forearm approximately horizontal beside the table's left edge, wrist neutral
and hand open. Match that
pose with the tracked real arm, then press `L` to calibrate and start feedback.
Press `R` to pause and return to the reference pose, or `K` to recalibrate at
the current pose. Press `Q` in the camera window to stop. See
[Sensor data](SENSOR_DATA.md) for wiring and units.

Calibration anchors camera and IMU angles to the displayed joint pose. A matched
bent elbow therefore stays bent instead of being reset to zero; raising/lowering
and bending/extending use signed changes around this reference. Relative wrist
twist uses the calibrated elbow angle when subtracting shared upper-arm rotation.
