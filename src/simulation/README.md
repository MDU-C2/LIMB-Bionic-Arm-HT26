# Simulation

The simulation contains the PyBullet arm, manual controls, trajectory playback,
and the experimental Motion AI tools.

Start it from the **Simulation** tab in the GUI:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

The current simulation can:

- move the shoulder, upper arm, elbow, wrist, and fingers;
- run with direct joint control or PyBullet physics;
- reach for and hold the simulated cup;
- show simulated contact and motor torque values;
- play the example or another supported trajectory; and
- play compatible camera pose recordings.

Useful direct commands:

```powershell
micromamba run -n aurora-simulation python src/simulation/interactive/limb_simulator.py
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
```

The simulation does not yet mirror live movement from the physical cuff. That
can be connected later so the simulated arm follows the measured movement.

See [the simulation guide](../../docs/SIMULATION.md) for the controls.
