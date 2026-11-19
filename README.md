# AURORA

AURORA is a student project at [Mälardalen University](https://www.mdu.se/)
with the aim of developing a bionic arm for the rehabilitation of stroke
survivors. The project continues the work started in LIMB. This repository
contains the sensor firmware, recording tools, arm simulation and movement
training code.

## Platforms used

The current setup uses an ESP32-C3-Zero to read EMG and two LSM6DSO32 IMUs,
an OAK-D camera to track arm movement, and a PC to run the app and PyBullet
simulation. The ESP32 sends sensor data over Bluetooth Low Energy. The camera
connects to the PC by USB.

## Running the program

On Windows, run `Setup.bat` for the first setup, then use `Start.bat` to open
the app offline. Setup installs all app packages, checks the pose models and builds
firmware; it needs internet. See the
[setup notes](scripts/README.md) for direct launching and the macOS/Linux scripts.

Follow the [workflow guide](docs/workflow-guide.md) to connect and inspect,
prepare/calibrate, record or control, then review, process and export.
Advanced Tools contains optional training, DMP/manual joints, diagnostics and firmware.
See [architecture](docs/architecture.md) for code ownership and development conventions.

## Before recording

For the Grove EMG Detector cable with white wires and red, black and white snap heads:

| Lead | Electrode placement for grip |
| --- | --- |
| Red | Reference on a bony area near the elbow, away from the measured muscle. |
| Black | Palm-facing forearm, over the muscle that firms when making a fist. |
| White | Same muscle, a few centimetres towards the wrist from the black electrode. |

Keep black and white in line along the muscle without overlapping the pads.
Use clean, dry skin. These colours apply to this cable; see
[Seeed's placement and cable diagrams](https://wiki.seeedstudio.com/Grove-EMG_Detector/#hardware-installation).
Use battery power and BLE with USB and chargers disconnected while wearing electrodes.

In **Record**, connect camera + BLE and run **Calibrate subject** before the first
trial. This includes IMU zeroing, mounting movements and EMG rest/grip calibration.
See the [recording guide](docs/recording-guide.md#before-the-first-trial) for the
still pose and when to repeat calibration.

## Structure of repository

The main folders are:

- [`scripts/`](scripts/README.md): setup and sensor checks.
- [`src/aurora/gui/`](src/aurora/gui/README.md): the desktop app.
- [`src/aurora/sensors/`](src/aurora/sensors/README.md): sensor input, camera tracking and fusion.
- [`src/aurora/recording/`](src/aurora/recording/README.md): sensor and camera recording, grip trials and EMG processing.
- [`src/aurora/simulation/`](src/aurora/simulation/README.md): the arm simulation and playback.
- [`src/aurora/ml/`](src/aurora/ml/README.md): movement profiles and EMG classifiers.
- [`firmware/`](firmware/README.md): programs running on the ESP32-C3.
- [`data/`](data/README.md): inherited movement recordings, kept separately.
- [`docs/`](docs/README.md): setup, recording and simulation guides.
- [`tests/`](tests/README.md): software checks.

Bundled models are in `artifacts/models/`. New recordings, training runs and
generated files go in `outputs/`, which is ignored by Git.
