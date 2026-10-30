# Simulation controls and conventions

Use the [workflow guide](workflow-guide.md#control) for setup, calibration,
preview/physics selection and recorded replay.

## Keyboard

| Action | Keys |
| --- | --- |
| Shoulder up/down | W / S |
| Shoulder left/right | A / D (A left, D right) |
| Upper-arm rotation | Q / E |
| Elbow bend/extend | Up / Down |
| Wrist rotation | Left / Right |
| Close/open fingers | F / G |
| Reach, grab, or release | Space |
| Move toward the target | H |
| Change camera | C or Tab |
| Show camera panels | P |
| Show sensor display | I |
| Advance live setup | K or L (live mode) |
| Reset and return to neutral tabletop reference | R |
| Quit | Escape |

R pauses and resets the robot while retaining valid same-connection live
calibration. K/L advance the current live preparation step. **Sensors moved /
repeat preparation** explicitly invalidates calibration. Escape quits.

## Scene and physics

The table is centered on the torso's world-X centerline. The left shoulder is
mounted outside the table's left edge, angled inward, so a forward lift beside
the table clears it with either a straight or bent elbow. Lift before swinging
over the tabletop. The cup starts on the left side of the table, to the right of
the arm; the green disc marks its destination farther right. Grip, carry and
release the cup using the existing controls. Reach assistance uses the mounted
shoulder's coordinate frame. In physics mode, open the fingers and tilt or
withdraw the hand when placing the cup so it can settle onto the table.


The human arm selection does not change the robot model: control targets the
left robot arm. Direct preview applies bounded commands. Motor physics uses
fixed timesteps, gravity, collision and actuator speed/force limits. See
[joint limits](joint-limits.md), [motor settings](motor-settings.md) and
[dynamics](simulation-dynamics.md). Table dimensions default to 60 x 60 cm;
other simulation dimensions do not change the participant collection protocol.

## Saved movement playback

**Advanced Tools / DMP / manual joints** provides fitting/export, saved/refitted
DMP playback, compatible camera pose playback and robot sliders. Saved DMP
rollouts keep their samples and timestep. An angles-only inherited fit assumes
one second. Endpoint changes below 0.02 radians retain the demonstration's
absolute amplitude, including returns to the start. Exports keep unclipped
radians; playback maps them into robot limits.

DMP durations and timesteps are in seconds. Fitting and rollout use the same
time scaling, including the velocity term, so changing duration preserves the
motion at the same phase. This follows the first-order equations in
[Ijspeert et al. (2013), equation 2.1](https://viterbik12.usc.edu/wp-content/uploads/2017/06/Dynamical-Movement-Primitives_-Learning-Attractor-Models-for-Motor-Behaviors.pdf).
Motor speed limits can lengthen actual playback beyond the requested duration.

**Advanced Tools / Movement profiles** retains camera profile playback,
comparison and optional recognition. Training results need independent evaluation.
See [live fusion](live-control.md) for sensor timing and observability limits.
