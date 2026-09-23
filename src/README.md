# Source code

Start the current software through the GUI:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

| Folder | Contents |
| --- | --- |
| `gui/` | Desktop interface and program control. |
| `recording/` | Sensor and camera previews and recordings. |
| `simulation/` | Arm simulation, trajectories, and Motion AI. |
| `ml/` | Movement-data capture and model training. |

Generated recordings and simulation output belong in `outputs/`, not `src/`.
