# Simulation

The main simulation is the interactive PyBullet table-and-target scene. It can
be driven by the keyboard or by live OAK-D, one/two-IMU, and EMG measurements.

Start it from the **Simulation** tab in the GUI:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

The current simulation can:

- move the shoulder, upper arm, elbow, wrist, and fingers;
- run with direct joint control or PyBullet physics;
- reach for and hold the simulated cup;
- show simulated contact and motor torque values;
- play compatible camera pose recordings through Motion AI; and
- fuse live OAK-D and available IMU angles to control the left arm;
- select camera hand curl or calibrated EMG activation for grip.

Useful direct commands:

```powershell
micromamba run -n aurora-simulation python src/simulation/interactive/limb_simulator.py
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
```

Live control is started from the Simulation tab after entering the ESP32 BLE
device name (default `LIMBServer`). No COM port is needed. Close standalone BLE
and camera previews first. BLE input reconnects automatically and processes
EMG/IMU samples in the background without saving a recording.
It opens the same interactive task scene plus an annotated OAK-D camera window.
A shoulder IMU controls elevation and left/right movement; adding the wrist IMU
enables relative-IMU elbow control. Camera tracking supplies absolute arm pose
and corrects IMU upper-arm twist when the bent elbow is visible. Both IMUs also
supply relative forearm twist with elbow-bend compensation. Mount matching axes
along the limbs and hold still for 0.5 seconds after L; default axes are Y.
Forearm twist has no camera correction and can drift. Camera hand curl
controls grip by default; select EMG once electrodes and muscle signals are
ready. Auto retains EMG-first selection with camera fallback when EMG is absent.
Q/E and Left/Right provide manual fallback when live twist targets are absent. A wrist-only
sensor is monitored without being misidentified as the upper arm. Press `K` in
the simulator to recalibrate. At startup, copy the simulated straight-arm pose
and press `L` to start feedback. Press `R` to pause and align again.

See [the simulation guide](../../docs/SIMULATION.md) for the controls.

## Interactive code

`interactive/limb_simulator.py` is the launcher. `runtime.py` owns initialization,
the frame order, and cleanup. The existing behavior is divided by responsibility:

| Module | Responsibility |
| --- | --- |
| `settings.py` | Arguments, reference poses, and task settings. |
| `arm_control.py` | Arm commands, limits, and PyBullet joint mapping. |
| `scene.py` | World setup, target collisions, and camera views. |
| `input_control.py` | Keyboard commands, live calibration, and arm movement. |
| `grasp_control.py` | Safe approach, finger movement, grasping, and release. |
| `physics.py` | Physics steps, measured state, and torque output. |
| `dashboard.py` / `hud.py` | Tk sensor monitor and Pygame controls display. |

Each part operates on the simulator instance, so task state is explicit rather
than stored in module globals. `dmp/dmp_signal.py` holds trajectory preprocessing
and optional curvature coupling; `dmp.py` keeps the fitting and rollout API.
