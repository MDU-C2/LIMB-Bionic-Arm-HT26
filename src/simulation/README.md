# Simulation

Open Simulation in the app. Choose keyboard control or live camera and sensor
input. Direct mode shows the requested angles. Physics mode adds gravity,
collisions, motor control and torque estimates.

The table is 60 x 60 cm and centered in front of the torso. The left shoulder
sits outside its left edge. Lift beside the table, turn inward, pick up the cup
and carry it to the green disc on the right.

For live control, use BLE with the device name `LIMBServer` and both IMUs.
Follow the setup panel: align at marker 1, calibrate relaxed EMG, calibrate firm
grip, then start control. Press R to pause and reset. K and L advance the current
step. The app defaults to EMG grip; Camera and Auto are also available.

Run directly from the project environment:

```powershell
python src/simulation/interactive/limb_simulator.py
python src/simulation/sim/limb_sim.py --headless
```

See the [simulation guide](../../docs/SIMULATION.md) for keyboard controls,
sensor mounting and calibration.

## Code

- `interactive/`: task scene, arm controls, grip and displays.
- `sim/`: arm models, joint limits and physics calculations.
- `ai/`: live sensor input, pose playback and movement comparison.
- `dmp/`: trajectory fitting and playback.

The movement model is loaded from `artifacts/models/movement_gru.onnx`.
