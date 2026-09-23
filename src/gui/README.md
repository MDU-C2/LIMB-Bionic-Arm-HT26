# GUI

The GUI is the main way to use the current project:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

## Tabs

| Tab | Use |
| --- | --- |
| Simulation | Open the arm simulator or play a trajectory. |
| Recording | Record several selected sources as one session. |
| Recordings | Browse files created by the recorders. |
| Sensors | Open separate EMG, IMU, piezo, camera, and serial previews. |
| Motion AI | Play camera poses or run movement recognition. |
| Robot | Detect firmware projects when they are added later. |
| Info | Show the files and dependencies found by the program. |

EMG, IMU, and piezo each have their own window. They share one BLE connection
to the cuff, so all three can be open at the same time. Camera and serial
previews can also stay open alongside them.

## GUI files

- `app.py` builds the main window and shared controls.
- `recording_tab.py` contains sensor, recording, and recording-browser actions.
- `simulation_tabs.py` contains Simulation and Motion AI.
- `project_tabs.py` contains Robot and Info.
- `process_manager.py` starts and stops the programs opened by the GUI.
- `project_support.py` contains paths and project discovery helpers.

Output from started programs appears in the Activity panel.
