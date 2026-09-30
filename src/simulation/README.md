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
- use calibrated EMG activation for grip, with camera hand curl as fallback.

Useful direct commands:

```powershell
micromamba run -n aurora-simulation python src/simulation/interactive/limb_simulator.py
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
```

Live control is started from the Simulation tab after choosing the ESP32 port.
It opens the same interactive task scene plus an annotated OAK-D camera window.
A shoulder IMU controls elevation and left/right movement; adding the wrist IMU
enables relative-IMU elbow control. Camera tracking supplies absolute arm pose
and the otherwise-unobservable axial-rotation proxy. EMG controls finger curl
after a short relaxed-muscle calibration; camera hand curl is used when EMG is
unavailable. A wrist-only
sensor is monitored without being misidentified as the upper arm. Press `K` in
the simulator to recalibrate. At startup, copy the simulated straight-arm pose
and press `L` to start feedback. Press `R` to pause and align again.

See [the simulation guide](../../docs/SIMULATION.md) for the controls.
