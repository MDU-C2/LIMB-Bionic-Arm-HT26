# AURORA – Adaptive User Robotic Rehabilitation Arm

AURORA continues the LIMB robotic-arm work in the **DVA490 / DVA474 –
Projektkurs i Robotik** course at Mälardalen University (MDU), autumn 2026.
The repository is maintained by the project team for future student groups.

Course organization: [MDU Collaborative Center](https://github.com/MDU-C2)

## Current scope

The current repository provides:

- a Tkinter control center for simulation, recording, saved data, sensors,
  Motion AI, and future robot firmware;
- an interactive PyBullet model of the five driven arm motions and fingers;
- DMP trajectory playback and manual joint controls;
- BLE (continuous or labeled), serial, and OAK-D recording programs; and
- camera-pose playback and an experimental movement-recognition model.

The simulation and desktop software run locally. Hardware recording still
needs validation with the current camera, cuff electronics, and firmware.
Robot firmware and the as-built hardware description have not been migrated.

## Quick start

Install [micromamba](https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html),
then run these commands from the repository root:

```powershell
micromamba create -f src/simulation/environment.yml
micromamba run -n aurora-simulation python src/gui/app.py
```

The environment uses Python 3.10. The GUI resolves repository paths at runtime,
so no group member needs to edit paths for their clone.

## Control center

| Tab | Purpose |
| --- | --- |
| Simulation | Run the interactive scene, trajectory playback, or joint sliders. |
| Recording | Configure and start a sensor or camera recording. |
| Recordings | Browse files below the selected recording folder. |
| Sensors | Select the BLE, serial, or OAK-D recording source. |
| Motion AI | Play camera poses on the arm or compare movement recordings. |
| Robot | Discover, build, flash, and monitor future firmware projects. |
| Info | Check local dependencies and open project documentation. |

The simulator opens PyBullet, controller, and sensor windows separately. The
Control Center remains open and can be reached through Alt+Tab or the taskbar.

## Repository layout

| Path | Contents |
| --- | --- |
| [`src/`](src/) | Maintained computer-side software. |
| [`docs/`](docs/) | Setup, operation, technical notes, and references. |
| [`examples/`](examples/) | Small data and commands for confirmed features. |
| [`firmware/`](firmware/) | Reserved for reviewed embedded firmware. |
| [`hardware/`](hardware/) | Reserved for the verified hardware source of truth. |
| [`research/`](research/) | Reproducible experiments outside the runtime. |
| [`scripts/`](scripts/) | Maintenance and conversion utilities. |
| [`tests/`](tests/) | Automated and hardware-in-the-loop validation. |

## Handover priorities

1. Measure the physical joint zero positions, travel, speed, mass, and inertia,
   then update the shared limits and URDF.
2. Validate BLE packet decoding, serial capture, and OAK-D depth alignment with
   the devices used by the 2026 team.
3. Add live camera-to-simulation control after recording and playback are
   stable.
4. Inventory the installed controller boards and migrate only their active
   firmware, pin maps, CAN identifiers, and recovery procedures.
5. Expand automated tests for packet formats, pose mapping, joint limits, and
   GUI discovery before hardware control is introduced.

See the [simulation and recording guide](docs/SIMULATION.md),
[GUI extension guide](src/gui/README.md), and
[development references](docs/resources.md) for details.

## Provenance

The project builds on [MDU-C2/LIMB-HT25](https://github.com/MDU-C2/LIMB-HT25).
Migrated code is kept only when its role, dependencies, and limitations can be
explained in this repository.
