# Simulation

Start the GUI and open **Simulation**:

```powershell
micromamba run -n aurora python -m aurora.gui.app
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
| Advance live setup | K or L (live mode) |
| Reset and return to neutral tabletop reference | R |
| Quit | Escape |

The GUI can start the simulator in direct mode or physics mode. Physics mode
uses gravity, motor control, collision, and simulated torque/contact values.
The arm is a complete left/right reflection, including the wrist and fingers,
mounted to the left of the centered table. See [Joint limits](joint-limits.md)
for the source of every movement range and the camera-to-CAD zero conversion.

**Connect + open live setup** opens the scene paused, the camera view and a
step-by-step calibration panel. Follow **Align starting pose → Relaxed EMG →
Firm grip → Start interactive control**. The [live control guide](live-control.md)
explains arm placement, fusion and offline replay.

Live preview follows bounded pose targets directly. **Physics mode** keeps
motor speed and force limits, so it may follow a person more slowly. The shared
finger command follows calibrated EMG by default in the app; Camera and Auto
remain available. Live grip needs fingertip contact to secure the cup. Opening
the hand releases it. F/G and Space remain available in keyboard control.

Camera inference runs separately from the simulation loop. BLE control reads
continue between camera observations. Each stream expires after half a second
without updates, and reconnecting invalidates the guided calibration.

### Mounting and twist calibration

Fix IMU1 (`0x6B`) to the upper arm and IMU2 (`0x6A`) to the forearm. Select
signed longitudinal sensor axes in Simulation; the default is Y for each board.
Keep their bend axes consistent. The camera provides trunk-relative pose and
corrects the mounted IMU estimate. Sensor distance is not needed for angle control.

Calibrate with the hand open at marker 1, thumb up and elbow comfortably bent.
The guide waits for a still pose and then an IMU rest baseline. After calibration,
check isolated elbow bending, arm raising, sideways motion and forearm twist.
Pause and correct mounting/settings if a direction is wrong. The tilt and twist
mapping remains an approximation; forearm axial drift has no camera correction.

**Replay a recorded cup trial** accepts `trial.json` and uses saved BLE packets,
calibration and camera observations without opening hardware. Replay validates
the control path, not anatomical tracking accuracy.

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
python -m aurora.simulation.live_control --device AURORA
```

`--shoulder-twist-axis y --wrist-twist-axis y` selects the default mounting;
each option accepts `x`, `y`, `z`, `-x`, `-y`, or `-z`.
`--grip-source emg` selects EMG grip; `--grip-source auto` selects EMG-first
fallback. `--address` can select a known BLE address. Older `--port COM5` commands retain
legacy serial control for firmware explicitly built with USB transport. That path
uses **Align and start USB control** and does not use the guided BLE calibration.
BLE packet
decoding, fusion, and connection cleanup are checked offline; ESP32/OAK-D live
operation passed a short connected check; muscle signals and tracked-arm accuracy
still need validation. See [Hardware validation](hardware-validation.md).

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

Live control begins paused at the tabletop reference. Follow the calibration
panel with the open hand at marker 1, then press **Start interactive control**.
Press `R` to pause and reset. `L` and `K` advance the current setup step.
Press `Q` in the camera window to stop. See [Sensor data](sensor-data.md) for wiring.

Calibration anchors camera and IMU angles to the displayed joint pose. A matched
bent elbow therefore stays bent instead of being reset to zero; raising/lowering
and bending/extending use signed changes around this reference. Relative wrist
twist uses the calibrated elbow angle when subtracting shared upper-arm rotation.
