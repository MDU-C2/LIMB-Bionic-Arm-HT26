# LIMB – Bionic Arm

Project repository for the **DVA490 / DVA474 – Projektkurs i Robotik** course at Mälardalen University (MDU), autumn semester 2026 (HT2026).

<<<<<<< Updated upstream
Course center: [Projects @ Collaborative Center (C2) MDU](https://github.com/MDU-C2)
=======
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

## Train the movement model

The repository includes the minimal LIMB1 GRU training pipeline and its 100
landmark recordings. Install the ML dependencies and train from the repository
root:

```powershell
python -m pip install -r src/ml/requirements.txt
python src/ml/train_gru.py
```

See [the training guide](src/ml/README.md) for outputs and a quick smoke test.

## OAK-D Lite camera

Open the connected Luxonis RGB camera with:

```powershell
python src/camera/oak_preview.py
```

See [the camera guide](src/camera/README.md) for setup details.
>>>>>>> Stashed changes
