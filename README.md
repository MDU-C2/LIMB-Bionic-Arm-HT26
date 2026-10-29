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
the app offline. Setup needs internet once to install Python and the required packages. See the
[setup notes](scripts/README.md) for direct launching and the macOS/Linux scripts.

Follow the [workflow guide](docs/workflow-guide.md) to connect and inspect,
prepare/calibrate, record or control, then review, process and export.
Advanced Tools contains optional training, DMP/manual joints, diagnostics and firmware.
See [architecture](docs/architecture.md) for code ownership and development conventions.

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
