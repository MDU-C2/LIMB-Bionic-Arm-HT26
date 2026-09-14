# AURORA – Adaptive User Robotic Rehabilitation Arm

Project repository for the **DVA490 / DVA474 – Projektkurs i Robotik** course at Mälardalen University (MDU), autumn semester 2026 (HT2026).

Course center: [Projects @ Collaborative Center (C2) MDU](https://github.com/MDU-C2)

AURORA is a student project focused on the continued development of a robotic arm. This repository will collect the project's documentation, software, firmware, hardware files, and research as the work develops.

The project builds on earlier LIMB work

## Project launcher

The shared Tkinter control center has Simulation, Recording, Recordings, Robot,
and Info tabs, with a live activity panel that stays visible. It resolves files
relative to the repository, so each group member can clone the project wherever
they want without changing paths.

The full LIMB task simulator, trajectory playback, and manual PyBullet joint
sliders are ready now. Recording scripts and ESP-IDF or PlatformIO firmware
projects appear automatically when their entry points are added to `src` and
`firmware`.

On a fresh clone, create the environment once and then open the launcher:

```powershell
micromamba create -f src/simulation/environment.yml
micromamba run -n aurora-simulation python src/gui/app.py
```

See the [GUI extension guide](src/gui/README.md) for adding tabs and program
buttons. The [simulation setup and usage guide](docs/SIMULATION.md) covers the
environment, controls, data formats, and migrated sources.
