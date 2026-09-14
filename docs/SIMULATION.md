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
3. **LIMB Sensor Dashboard** — seven joint angle/torque rows, hand orientation,
   simulated shoulder effort, and hand-contact status.

Click the Arm Control HUD before using the keyboard.

| Control | Keys |
| --- | --- |
| Shoulder elevation | Up / Down |
| Shoulder lateral movement | Left / Right |
| Shoulder rotation | C / V |
| Elbow flexion | Z / S |
| Elbow pronation | A / E |
| Wrist flexion | Q / D |
| Wrist deviation | W / X |
| Faster / slower movement | Shift / Ctrl |
| Grip / release target | F / G |
| Move toward target with IK | H |
| Toggle target overlay | T |
| Orbit / shoulder camera | Tab |
| Reset arm and target | Space |
| Quit | Escape |

The RGB, depth, and segmentation panels match the final LIMB25 screenshots.
They are PyBullet preview panels; the inherited simulator does not yet publish
or record camera frames.

## What the sensor values mean

The dashboard is useful for software integration, but it is not a validated
hardware measurement system:

- Joint angles come from the simulated URDF joints.
- Torque is PyBullet's applied motor torque, not measured physical torque.
- Hand roll, pitch, and yaw come from the simulated hand-link orientation.
- `EMG Shoulder` is the average absolute simulated shoulder torque.
- Hand pressure reports no measurement because the model has no pressure
  sensor. Grasping uses a fixed constraint when the hand is close enough.

PyBullet warns that `base_link`, `arm_base_rotated`, and `hand_cover_link` have
no inertial data. It supplies default inertia so the simulator runs, but those
links need measured mass and inertia before dynamics or torque results can be
treated as representative of the physical arm.

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
Refitting reports and removes those rows before training. Both the saved and
refitted examples also report 20 samples clipped to the selected joint limits;
this is expected for the inherited data and remains visible in the console so
it is not mistaken for unrestricted robot motion.

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
