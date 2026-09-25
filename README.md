# AURORA - Adaptive User Robotic Rehabilitation Arm

AURORA is a student project at Mälardalen University. This repository currently
contains the computer-side software for sensor previews, recording, camera
tracking, movement data, robot simulation, and ESP32-C3 dual-IMU firmware.

## Run the program

Open PowerShell in the repository root:

```powershell
micromamba create -f src/simulation/environment.yml
micromamba run -n aurora-simulation python src/gui/app.py
```

The GUI can:

- monitor shoulder and wrist IMUs in a dedicated window;
- monitor the OAK-D camera and pose tracking without recording;
- record dual-IMU serial data and OAK-D camera data as one session;
- fuse live camera and IMU angles to control the PyBullet arm;
- run the interactive PyBullet arm simulation;
- play saved camera poses; and
- run the experimental movement-recognition tools.

## Main folders

| Folder | Contents |
| --- | --- |
| `src/gui/` | The desktop application. |
| `src/recording/` | BLE, camera, and serial preview and recording tools. |
| `src/simulation/` | PyBullet simulation and Motion AI tools. |
| `firmware/` | ESP-IDF firmware for the dual-IMU ESP32-C3. |
| `src/ml/` | Movement-data capture and model training. |
| `data/` | The movement training data currently included in the project. |
| `docs/` | Short usage guides. |
| `tests/` | Automated software tests. |

Generated recordings are written below `outputs/recordings/` and should not be
committed to Git.

More details are in the [GUI guide](src/gui/README.md),
[recording guide](src/recording/README.md), and
[simulation guide](src/simulation/README.md).
