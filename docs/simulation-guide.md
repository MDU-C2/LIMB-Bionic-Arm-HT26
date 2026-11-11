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

## DMP experiments before participant data

Open **Advanced Tools → DMP / manual joints**. Use this sequence:

1. **Learn and compare DMP** starts with the bundled example folder. Leave start
   and end at zero to use the full example. Choose a new output folder and Run.
2. **Open DMP comparison** proposes the last experiment folder. Blue is the
   original; orange is the fitted DMP. Each joint also shows reconstruction error
   and RMSE in degrees, aligned by movement phase. Select duration 0.5, 1 or 2 to
   inspect faster or slower motion. This changes the view, not the saved files.
3. **Play DMP trajectory** proposes the experiment folder. Its duration multiplier
   applies to the exported trajectory: 0.5 takes half as long, 2 takes twice as long.
   Actual playback can take longer because of motor speed limits.
4. **Create synthetic cup trial for DMP** creates a four-second exercise with
   generated IMU packets and camera angle observations. It contains no video or
   participant measurements and is explicitly marked synthetic. Choose its
   `trial.json` in **Learn and compare DMP** to practise the future import flow.

A new experiment saves `experiment.json` (provenance, timing, settings and metrics),
`comparison.png`, `comparison.csv`, `comparison.npz` (source estimates and DMP),
`model.npz` (weights, basis functions and endpoints), and compatible playback
files. `angles.npz` and `dmp_rollout_clean.npz` contain unclipped actuator angles;
`timing.json` preserves the original duration when refitting. Original source
files remain unchanged. The bundled legacy angles have no measured duration and
use an explicitly reported one-second assumption; invalid legacy rows are reported.

## Import a future cup trial

Finish recording and review the capture first. Choose a completed `trial.json`
through **Learn and compare DMP**, or use **Learn DMP from selected trial** in
Review. Set start/end in seconds from capture start; end=0 means the recording
end. A trial's first camera observation may occur after zero; choose a start at
or after that time. Replay consumes the preceding events before extracting the
selected interval, retaining the integration history and saved calibration.

The import uses the existing offline replay/fusion code without opening hardware.
All four source channels must be available throughout the chosen interval;
missing/unreferenced channels, stale inputs or reconnections stop the import.
Select another continuous interval or repair the recording/calibration rather
than filling missing angles with zero. The source and derived files are kept
separate. Resampling and the selected source-time interval are recorded explicitly.

The fourth cup-trial channel is **shoulder rotation proxy**, not validated
anatomical axial rotation. Comparison plots retain that label. Playback uses the
existing left CAD convention: shoulder flexion = 90° − estimated flexion,
rotation = −proxy, with abduction sign applied by the playback joint mapping.
The preview uses the left CAD model for either recorded arm; it is not a
side-specific anatomical validation. Grip/EMG is not fitted by this four-joint DMP.
Reconstruction errors measure agreement with estimated angles, not independent
physical tracking accuracy or task success. Synthetic exercises cannot establish
performance on participant recordings.
