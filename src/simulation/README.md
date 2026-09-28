# Simulation

The main simulation is the interactive PyBullet table-and-target scene. It can
be driven by the keyboard or by live OAK-D and one/two-IMU measurements.

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
- fuse live OAK-D and available IMU angles to control the left arm.

Useful direct commands:

```powershell
micromamba run -n aurora-simulation python src/simulation/interactive/limb_simulator.py
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
```

Live control is started from the Simulation tab after choosing the ESP32 port.
It opens the same interactive task scene plus an annotated OAK-D camera window.
A shoulder IMU controls elevation and left/right movement; adding the wrist IMU
enables relative-IMU elbow control. Camera tracking supplies absolute arm pose,
the otherwise-unobservable axial-rotation proxy, and finger curl. A wrist-only
sensor is monitored without being misidentified as the upper arm. Press `K` in
the simulator to recalibrate while the tracked arm is down and still.

See [the simulation guide](../../docs/SIMULATION.md) for the controls.
