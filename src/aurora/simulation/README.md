# Simulation

Follow the [workflow guide](../../../docs/workflow-guide.md) for keyboard/live
control, calibration and replay. See [simulation conventions](../../../docs/simulation-guide.md)
for keys, scene geometry and DMP timing, and [dynamics](../../../docs/simulation-dynamics.md)
for physics assumptions.

## Code

- `interactive/`: task scene, arm controls, grip and displays.
- `model/`: arm models, joint limits and physics calculations.
- `trajectory_playback.py`, `pose_playback.py`: saved trajectory and camera playback.
- `joint_sliders.py`: manual joint sliders.
- `live_control.py`: command-line launcher for live control.
- `dmp/`: trajectory fitting and playback.

Shared sensor input and fusion are in `../sensors/`. Movement comparison is in `../ml/`.
