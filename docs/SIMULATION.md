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
| Reset | R |
| Quit | Escape |

The GUI can start the simulator in direct mode or physics mode. Physics mode
uses gravity, motor control, collision, and simulated torque/contact values.

**Start live interactive control** opens the same task scene together with an
annotated OAK-D camera monitor and a separate live sensor monitor. It uses the
shoulder and wrist IMUs for
responsive orientation, corrects them with camera-derived arm angles, and
passes the result through the existing joint limits and speed limits. The ESP32
port is released from the standalone IMU monitor automatically before live
control starts.

**Open camera monitor** in the GUI header shows live OAK-D tracking without
recording. Close that standalone monitor before starting live interactive
control because only one process can own the camera.

The **Motion AI** tab can play compatible OAK-D pose recordings and run the
experimental movement-recognition model.

Press `C` in the live camera window to recalibrate both IMUs at the current pose
and `Q` to stop. See [Sensor data](SENSOR_DATA.md) for wiring and units.
