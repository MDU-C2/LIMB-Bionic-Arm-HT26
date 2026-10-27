# Simulation

Open Simulation in the app. Choose keyboard control or live camera and sensor
input. Direct mode shows the requested angles. Physics mode adds gravity,
collisions, motor control and torque estimates.

The table is 60 x 60 cm and centered in front of the torso. The left shoulder
sits outside its left edge. Lift beside the table, turn inward, pick up the cup
and carry it to the green disc on the right.

For live control, use BLE with the device name `AURORA` and both IMUs.
Follow the setup panel: fit mounting through the guided movements with stereo
depth, calibrate relaxed EMG, calibrate firm
grip, then start control. Press R to pause and reset. K and L advance the current
step. The app defaults to EMG grip; Camera and Auto are also available.

Run directly from the project environment:

```powershell
python -m aurora.simulation.interactive
python -m aurora.simulation.trajectory_playback --headless
```

See the [simulation guide](../../../docs/simulation-guide.md) for keyboard controls,
sensor mounting and calibration.

Saved DMP rollouts keep their samples and timestep. When only angles are available,
the inherited fallback fits a one-second movement. Endpoint changes below 0.02 radians
keep the demonstrated motion's absolute amplitude, including returns to the start.

## Code

- `interactive/`: task scene, arm controls, grip and displays.
- `model/`: arm models, joint limits and physics calculations.
- `trajectory_playback.py`, `pose_playback.py`: saved trajectory and camera playback.
- `joint_sliders.py`: manual joint sliders.
- `live_control.py`: command-line launcher for live control.
- `dmp/`: trajectory fitting and playback.

Shared sensor input and fusion are in `../sensors/`. Movement comparison is in `../ml/`.
