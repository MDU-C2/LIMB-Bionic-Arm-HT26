# LIMB simulation

The project contains two simulation paths:

- The **interactive task simulator** is the main workspace. It recreates the
  final LIMB-HT25 table-and-target scene with the right arm, keyboard control,
  inverse kinematics, grasping, and live sensor values.
- The **trajectory tools** play a four-joint DMP rollout or open the left arm
  with simple joint sliders.

The easiest way to use either path is the project GUI:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

Open the **Simulation** tab and select **Open full simulator**. The launcher
finds the simulation environment without storing a machine-specific path, so
the same button works from every group member's clone.

## Install

Create the tested Python 3.10 environment from the repository root:

```powershell
micromamba create -f src/simulation/environment.yml
```

Conda can also read the same file:

```powershell
conda env create -f src/simulation/environment.yml
conda activate aurora-simulation
```

The environment contains NumPy, SciPy, PyBullet, and Pygame. Pygame is
installed from its Windows wheel through pip because the tested conda-forge
build could not initialize its SDL DLL on Windows.

## Interactive simulator

You can also start the full simulator directly:

```powershell
micromamba run -n aurora-simulation python src/simulation/interactive/limb_simulator.py
```

It opens three coordinated windows:

1. **PyBullet scene** — the right arm, table, target, reach line, camera views,
   collisions, and grasp constraint.
2. **Arm Control HUD** — joint angles, simulated motor torque, link positions,
   reachability, and keyboard controls.
3. **LIMB Sensor Dashboard** — five actuator angle/torque rows, hand orientation,
   simulated shoulder effort, and hand-contact status.

Click the Arm Control HUD before using the keyboard.

| Control | Keys |
| --- | --- |
| Shoulder up/down | Up / Down |
| Shoulder left/right | Left / Right |
| Upper-arm rotation | C / V |
| Elbow bend/extend | Z / S |
| Lower-arm rotation | A / E |
| Full / slow movement speed | Shift / Ctrl |
| Grip / release target | F / G |
| Move toward target with IK | H |
| Toggle target overlay | T |
| Orbit / shoulder camera | Tab |
| Reset arm and target | Space |
| Quit | Escape |

The target is anchored at its start position so an accidental collision cannot
throw it out of the workspace. Pressing **F** within the displayed grasp radius
removes that anchor and attaches the target to the hand. Pressing **G** releases
it to normal gravity, while **Space** restores the complete starting state.

The RGB, depth, and segmentation windows are PyBullet preview panels. The
simulator does not yet publish or record camera frames.

## Physical arm limits

The simulation uses the limits and maximum speeds from the final LIMB-HT25
motor firmware. These values are shared by the interactive controls, IK,
trajectory playback, sliders, and URDF models.

| Actuator | Range | Maximum speed | Acceleration |
| --- | ---: | ---: | ---: |
| Shoulder up/down | 0 to 90 deg | +10 / -20 deg/s | 15 deg/s² |
| Shoulder left/right | 5 to 40 deg | +20 / -10 deg/s | 15 deg/s² |
| Upper-arm rotation | -60 to 60 deg | 40 deg/s | 20 deg/s² |
| Elbow up/down | 0 to 60 deg | 40 deg/s | 20 deg/s² |
| Lower-arm rotation | 0 to 140 deg | 100 deg/s | Not set in firmware |

The source values are in the old `LIMB-HT25/src/esp32` shoulder, elbow, and
hand motor modules. The acceleration values are kept in the shared profile,
but the current PyBullet controller only enforces position and speed. Matching
the firmware acceleration ramps needs measured response data from the rebuilt
arm.

The two wrist bend/deviation joints in the mesh are fixed because the physical
arm has five driven arm motions. Finger commands use their firmware ranges:
thumb 0–30, index 0–85, middle 0–90, ring 0–50, and pinky 0–90 degrees.
The finger linkage in the URDF is still only a visual approximation.

The shoulder firmware calibrates up/down as 0–90 degrees, but its own note says
the measured mechanism reached closer to 75 degrees. Keep 90 degrees as the
software limit until the rebuilt arm is measured, then update
`src/simulation/sim/joint_limits.py` and the URDFs together.

## What the sensor values mean

The dashboard is useful for software integration, but it is not a validated
hardware measurement system:

- Joint angles come from the simulated URDF joints.
- Torque is PyBullet's applied motor torque, not measured physical torque.
- Hand roll, pitch, and yaw come from the simulated hand-link orientation.
- `EMG Shoulder` is the average absolute simulated shoulder torque.
- Hand pressure reports no measurement because the model has no pressure
  sensor. Grasping uses a fixed constraint when the hand is close enough.

The URDF uses estimated mass and inertia values. Replace them with measured
values before treating simulated dynamics or torque as representative of the
physical arm.

## Trajectory tools

Loop the included example trajectory:

```powershell
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --loop
```

Open the left-arm model with four joint sliders:

```powershell
micromamba run -n aurora-simulation python src/simulation/sim/manual_sim.py
```

Run one trajectory pass without a window as an installation check:

```powershell
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
```

The playback accepts a saved rollout NPZ containing `q_gen_rad` with shape
`(T, 4)` and a positive scalar `dt`. With `--refit`, it accepts elbow and
shoulder angle arrays and fits a new DMP rollout. `--source clean` and
`--source raw` select the preferred input variant and fall back to compatible
available data.

The included LIMB25 angle example contains 13 non-finite measurement rows.
Refitting reports and removes those rows before training. Playback reports any
values clipped to the robot limits and slows the sample interval when needed to
stay inside the firmware speed limits.

## Source and migration scope

The interactive simulator comes from `LIMB-HT25/LIMB Simulation`, where the
main implementation was introduced in commit `8f4fea9`. The migrated runtime
is deliberately small:

- `src/simulation/interactive/limb_simulator.py`
- `src/simulation/interactive/kinematics.py`
- `src/simulation/sim/arm/right_arm.urdf`
- Two right-hand meshes absent from the existing shared arm assets

The remaining 25 meshes were byte-identical to assets already used by the
trajectory model, so both simulators share one copy. The old six MKV screen
recordings and three cable-management photos are reference material rather
than runtime dependencies and were not copied.

The LIMB-HT25 URDF placed the forearm mesh before the lower-arm rotation joint
and the hand shell before the wrist-deviation joint. Those parts could therefore
look detached even though the kinematic links remained connected. The migrated
URDF assigns each mesh, collision shape, mass, and inertia to the downstream
link that actually rotates it.

The old `simulation_1_IMU.py`, `simulation_2_imus.py`, serial readers, and
Arduino test are separate COM5/COM6 hardware experiments. They do not produce
the final scene shown in the screenshots and contain machine-specific serial
settings, so they were not folded into the portable simulator. Real IMU input
should return later as a selectable sensor backend with ports chosen in the
Robot tab.

A repository-wide search also found `dmp/sim/limb_sim_table.py`, the slider
sandbox, older playback scripts, collision-analysis scripts, and archived
Panda/InMoov experiments. The table variant is the four-joint trajectory player
with a configurable box and disabled arm/table collisions; it is not the final
right-arm task scene. The maintained playback and manual slider paths already
cover the reusable parts of those scripts.

The four-joint trajectory player came from Oscar Ågren's `DMP-arm` commit
`88b0d2ea5df63764ebb56aad8926f92084926fc7`, previously included as a LIMB-HT25
submodule.
