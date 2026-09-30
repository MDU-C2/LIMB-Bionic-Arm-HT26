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
| Reset and return to straight-arm reference | R |
| Quit | Escape |

The GUI can start the simulator in direct mode or physics mode. Physics mode
uses gravity, motor control, collision, and simulated torque/contact values.
The arm is a complete left/right reflection, including the wrist and fingers;
the cup and camera are mirrored with it. See [Joint limits](JOINT_LIMITS.md)
for the source of every movement range and the camera-to-CAD zero conversion.

**Start live interactive control** opens the same task scene together with an
annotated OAK-D camera monitor and a separate live sensor monitor. A connected
shoulder IMU supplies mounted Y/Z tilt and gyro-Z left/right motion. The wrist
IMU adds relative elbow motion when present; a wrist-only IMU is monitored but
is not mistaken for an upper-arm sensor. Camera tracking supplies absolute pose,
axial-rotation correction, and finger curl. The result passes through the
existing joint and speed limits. The ESP32 port is released from the standalone
sensor monitor automatically before live control starts. EMG is calibrated from
a brief relaxed-muscle baseline and controls grip; tracked hand curl is the
fallback when no current EMG sample is available.

**Open camera monitor** in the GUI header shows live OAK-D tracking without
recording. Close that standalone monitor before starting live interactive
control because only one process can own the camera.

The **Motion AI** tab can play compatible OAK-D pose recordings and run the
experimental movement-recognition model.

Live control begins paused with the simulated arm fully extended. Match that
pose with the tracked real arm, then press `L` to calibrate and start feedback.
Press `R` to pause and return to the reference pose, or `K` to recalibrate at
the current pose. Press `Q` in the camera window to stop. See
[Sensor data](SENSOR_DATA.md) for wiring and units.
