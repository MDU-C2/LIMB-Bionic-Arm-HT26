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
the app. Setup installs Python and the required packages. See the
[setup notes](scripts/README.md) for direct launching and the macOS/Linux scripts.

Power the ESP32 and connect to `AURORA` through the app. The ESP32 can use
a separate USB power source while sending data over BLE. Earlier LIMB firmware
advertising `LIMBServer` is also supported.

The Recording tab saves EMG, IMU and camera data and includes guided grip
trials. The Simulation tab runs the arm with keyboard or live sensor control.
The Firmware tab builds and flashes the ESP32. See the
[recording guide](docs/recording-guide.md) and
[simulation guide](docs/simulation-guide.md) for the controls and calibration steps.

Live EMG grip control is still being developed and tested.

## Structure of repository

The main folders are:

- [`scripts/`](scripts/README.md): setup and sensor checks.
- [`src/aurora/gui/`](src/aurora/gui/README.md): the desktop app.
- [`src/aurora/sensors/`](src/aurora/sensors/README.md): sensor input, camera tracking and fusion.
- [`src/aurora/recording/`](src/aurora/recording/README.md): sensor and camera recording, grip trials and EMG processing.
- [`src/aurora/simulation/`](src/aurora/simulation/README.md): the arm simulation and playback.
- [`src/aurora/ml/`](src/aurora/ml/README.md): movement training and recognition.
- [`firmware/`](firmware/README.md): programs running on the ESP32-C3.
- [`data/`](data/README.md): inherited movement recordings, kept separately.
- [`docs/`](docs/README.md): setup, recording and simulation guides.
- [`tests/`](tests/README.md): software checks.

Models are stored in `artifacts/models/`. New recordings and generated files
go in `outputs/`, which is ignored by Git.
