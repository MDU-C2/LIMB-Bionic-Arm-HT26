# Simulation and recording

Start the control center:

```powershell
micromamba create -f src/simulation/environment.yml
micromamba run -n aurora-simulation python src/gui/app.py
```

## Interactive simulator

Open **Simulation** and select **Open full simulator**, or run:

```powershell
micromamba run -n aurora-simulation python src/simulation/interactive/limb_simulator.py
```

Three coordinated windows open: the PyBullet scene, the keyboard controller,
and the sensor dashboard. Click the controller for arm movement keys. Space,
F/G, H, and R also work while the PyBullet scene has focus.

| Action | Keys |
| --- | --- |
| Shoulder up/down | W / S |
| Shoulder left/right | A / D |
| Upper-arm rotation | Q / E |
| Elbow bend/extend | Up / Down |
| Wrist rotation | Left / Right |
| Precise movement | Ctrl |
| Close/open fingers | F / G |
| Reach for and grab / release a target | Space |
| Move toward the target with IK | H |
| Toggle target guide | T |
| Cycle cameras | C or Tab |
| Select camera | 1 / 2 / 3 |
| Toggle camera preview panels | P |
| Show/hide sensor dashboard | I |
| Reset the arm and target | R |
| Quit | Escape |

Space reaches for a reachable target and grasps it within 10 cm; press again to
cancel or release. Direct mode positions joints for a responsive preview.
Select **Physics mode** in the GUI, or pass `--mode dynamic`, for gravity,
motors, and payload effects. Add
`--telemetry-out outputs/simulation/physics.jsonl` to save diagnostics. See
the [robot dynamics guide](SIMULATION_DYNAMICS.md) for the model and limits.

Press P to show the simulated camera buffers.

## Joint model

The URDF has five driven arm motions:

| Actuator | Range | Maximum speed | Acceleration |
| --- | ---: | ---: | ---: |
| Shoulder up/down | 0 to 90 deg | +10 / -20 deg/s | 15 deg/s² |
| Shoulder left/right | 5 to 40 deg | +20 / -10 deg/s | 15 deg/s² |
| Upper-arm rotation | -60 to 60 deg | 40 deg/s | 20 deg/s² |
| Elbow | 0 to 60 deg | 40 deg/s | 20 deg/s² |
| Lower-arm rotation | 0 to 140 deg | 100 deg/s | Not specified |

These limits come from LIMB-HT25 firmware and need checking against the current
arm. Update `src/simulation/sim/joint_limits.py` and both URDFs together after
measurement.

Physics-mode torque values come from PyBullet motors; gravity-hold effort is an
inverse-dynamics estimate. The URDF mass and inertia remain unverified.

## Record sensors and camera pose

Use **Sensors** for BLE/serial live previews or the OAK-D camera. Recordings
start only when requested; the camera has its own REC control. The camera shows
the selected shoulder, elbow, and wrist and saves six arm/trunk points and
angle estimates. Motion AI pose playback requires optional stereo depth, which
still needs a device retest. See the [recording guide](../src/recording/README.md)
and [sensor data guide](SENSOR_DATA.md) for formats and experiment details.

## Trajectory tools

Loop the included example trajectory:

```powershell
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --loop
```

Open the left-arm joint sliders:

```powershell
micromamba run -n aurora-simulation python src/simulation/sim/manual_sim.py
```

The sliders set joints directly, including lower-arm rotation.

Run a trajectory without a window as an installation check:

```powershell
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
```

Playback accepts `q_gen_rad` with shape `(T, 4)` and a positive `dt` in an NPZ
file. `--refit` fits a DMP from recorded angles. Joint values and playback
speed follow the configured limits.

Camera-pose tools can also run directly:

```powershell
micromamba run -n aurora-simulation python src/simulation/ai/pose_recording_sim.py recording.json
micromamba run -n aurora-simulation python src/simulation/ai/movement_recognition.py recording.json --references path/to/references
```
