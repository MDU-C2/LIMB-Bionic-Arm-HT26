# Simulation

Start the GUI and open **Simulation**:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

**Open full simulator** starts the PyBullet scene, the keyboard controller, and
the simulated sensor display.

| Action | Keys |
| --- | --- |
| Shoulder up/down | W / S |
| Shoulder left/right | A / D |
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

The same tab can play trajectories from `examples/simulation/demo/` or another
folder containing a supported NPZ file. **Manual joint control** opens sliders
for inspecting individual joints.

The **Motion AI** tab can play compatible OAK-D pose recordings and run the
experimental movement-recognition model.

Live cuff values do not control the simulated arm yet. The current sensor
previews and simulation run separately.
