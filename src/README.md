# Computer-side software

`src` contains the maintained programs that run on a project computer.

| Directory | Responsibility |
| --- | --- |
| [`gui/`](gui/) | Tkinter launcher, process supervision, and tool discovery. |
| [`recording/`](recording/) | BLE, serial, and OAK-D data capture. |
| [`simulation/`](simulation/) | Interactive PyBullet scene, trajectories, pose mapping, and Motion AI. |

Start the software through the project GUI:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

Microcontroller applications belong in `firmware`, experiments that are not
part of the runtime belong in `research`, and generated recordings belong
under `outputs/recordings` or external project storage.
