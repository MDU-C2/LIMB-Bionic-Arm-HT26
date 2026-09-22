# LIMB robot model and dynamics

## Model

The interactive scene loads `sim/arm/right_arm.urdf`. The trajectory and
manual-slider tools load `left_arm.urdf`. Direct preview and playback use
`resetJointState`; Physics mode uses gravity and position motors.

The right URDF has **five driven arm joints**: shoulder Z and Y, upper-arm X
rotation, elbow Y, and forearm/wrist X rotation, plus articulated fingers.
Wrist bend and deviation are fixed. The [older HT25 simulation](https://github.com/MDU-C2/LIMB-HT25/tree/main/LIMB%20Simulation)
described seven arm DOFs. `sim/joint_limits.py` holds the current
firmware-derived limits, which are also encoded in the URDFs.

The old BOM's seven arm motors are not the same as seven verified joints. It
lists three NEMA17, two NEMA23, and two HV2060MG motors, while the final firmware
only maps two HV2060MG shoulder motors and two NEMA17 motors to arm joints. See
the [motor inventory](MOTORS.md) for the evidence and unresolved assignments.

`interactive/kinematics.py` keeps a planar two-link IK approximation. The old
HT25 [control-layer FK and Jacobian](https://github.com/MDU-C2/LIMB-HT25/blob/main/src/layers/control/control_layer.py)
assume a different shoulder rotation order. The loaded URDF supplies FK and
the Jacobian for `sim/dynamics.py`.

The right model uses 0.305 m upper arm, 0.310 m forearm, and a 0.105 m grip
offset: about 0.720 m straight reach. The old 0.120 m hand estimate gives a
different reach. The wrist origin is `(0.310, 0.0085, -0.005)` m. A fixed +90°
base rotation is in the right URDF; the left playback loader uses a separate
base orientation.

Repeated link masses and inertias in the URDF appear to be placeholders.
`sim/controller_params.py` contains simulation gains and torque caps. Neither
set has been verified against the current arm; CAD and bench measurements are
needed before using torque or payload results for hardware decisions.

## Modes and commands

The GUI's **Simulation** tab has a Physics mode checkbox. In Physics mode,
the target is held by a constraint and its mass affects the arm. The dashboard
shows current PyBullet motor effort, a ten-second graph for all five arm
motors, and estimated shoulder gravity torque. The graph is simulation output;
it is not a measurement from the physical actuators. The
estimate adds a 0.1 kg point payload while grasping. Motor requests are capped
by the URDF effort and speed fields.

The grasp closes at a limited speed and requires contact on at least two of the
five fingertip links. It preserves the cup's pose at the moment of contact.
The dashboard's fingertip values are virtual haptic signals: Physics mode uses
normal contact force, while Direct mode estimates a signal from collision
penetration. They are useful for control logic and visualization but are not
calibrated pressure sensor measurements.

Run either mode directly:

```powershell
micromamba run -n aurora-simulation python src/simulation/interactive/limb_simulator.py --mode kinematic
micromamba run -n aurora-simulation python src/simulation/interactive/limb_simulator.py --mode dynamic
```

Add `--torque-out outputs/simulation/torque.csv` for a long-format CSV that
names each movement and its firmware-mapped motor. The GUI can create a
timestamped file automatically. Add
`--telemetry-out outputs/simulation/physics.jsonl` to save time, joint
states and commands, motor torque, grip pose and velocity, Jacobians, mass
matrix, and dynamics terms. Arrays contain five arm joints followed by finger
joints. Position control has no commanded torque, so that field is null.

Inspect the complete loaded model or calculate an optional point payload:

```powershell
micromamba run -n aurora-simulation python src/simulation/sim/dynamics.py --inspect
micromamba run -n aurora-simulation python src/simulation/sim/dynamics.py --payload-kg 0.5
```

`--inspect` prints joint/link names, axes, origins, limits, mass, CoM, inertia,
and damping/friction. `--payload-kg` adds an ideal point mass at the grip
center; it does not model cup shape or grasp contact.

The module uses PyBullet's `calculateMassMatrix` and
`calculateInverseDynamics`. Gravity is inverse dynamics at zero velocity and
acceleration; subtracting it from zero-acceleration inverse dynamics isolates
the velocity term. A point payload adds `Jᵀ m(a − g)` with
`a = J qdd + Jdot qd`. Contact, damping, and controller effects can differ
from this rigid-body estimate.

## Validation and limits

`tests/test_simulation_dynamics.py` checks FK/Jacobian agreement, mass matrix,
inverse dynamics, gravity and payload effects, motor movement, and a grasped
0.1 kg object. Tests run headlessly; GUI windows and the physical arm have not
been validated by them.
