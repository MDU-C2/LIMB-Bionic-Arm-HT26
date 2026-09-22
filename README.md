# AURORA – Adaptive User Robotic Rehabilitation Arm

AURORA continues the LIMB robotic-arm work at Mälardalen University. It
provides a local control center for simulation, motion playback, and sensor
recording.

## What works

- PyBullet arm simulation with direct preview and physics modes, joint sliders,
  and DMP trajectory playback.
- BLE, serial, and OAK-D live previews and recording.
- Camera-pose playback and experimental movement recognition.

The current camera, cuff, and firmware still need device validation. The
as-built robot model and firmware are not yet in this repository.

## Quick start

Install [micromamba](https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html),
then run these commands from the repository root:

```powershell
micromamba create -f src/simulation/environment.yml
micromamba run -n aurora-simulation python src/gui/app.py
```

The GUI opens PyBullet, camera, and sensor previews in separate windows.

See the [simulation guide](docs/SIMULATION.md),
[robot dynamics](docs/SIMULATION_DYNAMICS.md),
[motor inventory](docs/MOTORS.md),
[sensor data guide](docs/SENSOR_DATA.md), and
[GUI development guide](src/gui/README.md).

The model and parts of the simulator derive from
[MDU-C2/LIMB-HT25](https://github.com/MDU-C2/LIMB-HT25).
