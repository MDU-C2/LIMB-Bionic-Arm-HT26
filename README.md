# AURORA - Adaptive User Robotic Rehabilitation Arm

AURORA is a student project at Mälardalen University. This repository currently
contains the computer-side software for sensor previews, recording, camera
tracking, movement data, and robot simulation.

The physical hardware files and finished robot firmware are not included yet.

## Run the program

Open PowerShell in the repository root:

```powershell
micromamba create -f src/simulation/environment.yml
micromamba run -n aurora-simulation python src/gui/app.py
```

The GUI can:

- preview EMG, IMU, piezo, OAK-D camera, and serial data;
- keep several preview windows open at the same time;
- record BLE sensors, camera data, and serial data as one session;
- run the interactive PyBullet arm simulation;
- play saved trajectories and camera poses; and
- run the experimental movement-recognition tools.

## Main folders

| Folder | Contents |
| --- | --- |
| `src/gui/` | The desktop application. |
| `src/recording/` | BLE, camera, and serial preview and recording tools. |
| `src/simulation/` | PyBullet simulation and Motion AI tools. |
| `src/ml/` | Movement-data capture and model training. |
| `data/` | The movement training data currently included in the project. |
| `docs/` | Short usage guides. |
| `tests/` | Automated software tests. |

Generated recordings are written below `outputs/recordings/` and should not be
committed to Git.

More details are in the [GUI guide](src/gui/README.md),
[recording guide](src/recording/README.md), and
[simulation guide](src/simulation/README.md).
