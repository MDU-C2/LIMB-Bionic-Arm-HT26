# Simulation and recording

The project provides an interactive right-arm task simulator, four-joint
trajectory playback, manual joint sliders, sensor recording, and camera-pose
playback. Start with the control center:

```powershell
micromamba create -f src/simulation/environment.yml
micromamba run -n aurora-simulation python src/gui/app.py
```

The environment uses Python 3.10 and contains PyBullet, Pygame, NumPy, SciPy,
ONNX Runtime, Bleak, pyserial, DepthAI, OpenCV, and MediaPipe.

## Interactive simulator

Open **Simulation** and select **Open full simulator**, or run:

```powershell
micromamba run -n aurora-simulation python src/simulation/interactive/limb_simulator.py
```

Three coordinated windows open: the PyBullet scene, the keyboard controller,
and the sensor dashboard. Click the controller before using the keyboard.

| Action | Keys |
| --- | --- |
| Shoulder up/down | W / S |
| Shoulder left/right | A / D |
| Upper-arm rotation | Q / E |
| Elbow bend/extend | Up / Down |
| Wrist rotation | Left / Right |
| Precise movement | Ctrl |
| Close/open fingers | F / G |
| Grab/release a nearby target | Space |
| Move toward the target with IK | H |
| Toggle target guide | T |
| Cycle cameras | C or Tab |
| Select camera | 1 / 2 / 3 |
| Toggle camera preview panels | P |
| Show/hide sensor dashboard | I |
| Reset the arm and target | R |
| Quit | Escape |

The reset pose keeps the arm clear of the table. The target stays at its start
position until it is grabbed, which prevents collisions from throwing it out
of the workspace. Move the hand within 10 cm and press Space to attach it.

The RGB, depth, and segmentation panels start hidden. Press P to show them.
They are simulated camera buffers and are not saved by the simulator.

## Joint model

The URDF includes the five driven motions visible on the physical arm:
shoulder left/right, shoulder up/down, rotation above the bicep, elbow flexion,
and lower-arm rotation at the wrist.

| Actuator | Range | Maximum speed | Acceleration |
| --- | ---: | ---: | ---: |
| Shoulder up/down | 0 to 90 deg | +10 / -20 deg/s | 15 deg/s² |
| Shoulder left/right | 5 to 40 deg | +20 / -10 deg/s | 15 deg/s² |
| Upper-arm rotation | -60 to 60 deg | 40 deg/s | 20 deg/s² |
| Elbow | 0 to 60 deg | 40 deg/s | 20 deg/s² |
| Lower-arm rotation | 0 to 140 deg | 100 deg/s | Not specified |

These values come from the LIMB-HT25 motor firmware and are applied by manual
control, inverse kinematics, trajectory playback, and the URDF. The old
shoulder firmware notes that the mechanism may only reach about 75 degrees,
so the current arm must be measured before the 90-degree value is trusted.
Update `src/simulation/sim/joint_limits.py` and both URDFs together when the
physical limits are confirmed.

Joint angles and hand orientation on the dashboard come from PyBullet. Torque
and shoulder effort are simulated motor values. The URDF mass, inertia, finger
linkage, and contact behavior are estimates, so they must not be treated as
physical measurements until the arm is calibrated.

## Record sensors and camera pose

Use **Sensors** to select BLE, serial, or OAK-D capture, then configure and
start the session in **Recording**. Every run creates a timestamped directory
below `outputs/recordings` by default. See the
[recording guide](../src/recording/README.md) for file formats and direct
commands.

For a LIMB-style training capture, select BLE and check **Labeled BLE capture**.
The GUI guides 20 rest, 40 movement, and 20 rest windows, then saves the old
wide CSV layout. Raw packets remain available if a capture is interrupted or
needs review.

OAK-D capture shows live video and records arm and hand landmarks. After the
session, select `pose.json` in **Motion AI** to animate the arm or compare the
movement with reference recordings. Live camera-to-simulation control has not
been implemented.

The migrated GRU model creates a movement embedding and compares it with
reference recordings. It identifies similar motion profiles; it does not
produce robot joint commands. Treat its output as experimental until it is
validated with a larger, documented dataset.

## Trajectory tools

Loop the included example trajectory:

```powershell
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --loop
```

Open the left-arm joint sliders:

```powershell
micromamba run -n aurora-simulation python src/simulation/sim/manual_sim.py
```

Run a trajectory without a window as an installation check:

```powershell
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
```

Playback accepts an NPZ file containing `q_gen_rad` with shape `(T, 4)` and a
positive scalar `dt`. With `--refit`, it accepts elbow and shoulder angle
arrays and fits a DMP rollout. Non-finite rows are removed, joint positions are
clipped to the shared limits, and playback slows when needed to respect motor
speed limits.

Camera-pose tools can also run directly:

```powershell
micromamba run -n aurora-simulation python src/simulation/ai/pose_recording_sim.py recording.json
micromamba run -n aurora-simulation python src/simulation/ai/movement_recognition.py recording.json --references path/to/references
```

## Troubleshooting

- If an import is missing, recreate or update `aurora-simulation` from
  `src/simulation/environment.yml`.
- If simulator windows cover the control center, use Alt+Tab or the taskbar.
- If BLE discovery fails, check the device name, power, and Windows Bluetooth
  access.
- If serial capture cannot start, select the current port and matching baud.
- `No available devices` from DepthAI means the OAK-D is not detected; check
  its USB connection and close other camera programs.
- Run the headless trajectory command above after changing the environment or
  simulation model.

## Provenance

The interactive task scene and right-arm model were migrated from
`MDU-C2/LIMB-HT25`. The trajectory player came from Oscar Ågren's DMP-arm work
previously included by that repository. Shared meshes are kept in one location
under `src/simulation/sim/arm`; archived videos, duplicate meshes, fixed-port
experiments, and machine-specific build products were not carried into the
runtime.
