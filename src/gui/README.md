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
| Sensors | See the USB ESP32/IMU stream and open BLE or camera previews. |
| Motion AI | Play camera poses or run movement recognition. |
| Robot | Detect firmware projects when they are added later. |
| Info | Show the files and dependencies found by the program. |

BLE EMG, BLE IMU, and piezo each have their own window and share one connection
to the optional LIMBServer cuff. The USB-connected ESP32 IMU is shown directly
in the embedded sensor panel instead. The camera window can stay open alongside
these previews.

When an ESP32 serial port is present, the Sensors tab connects at startup using
the port and baud rate shown under Recording. The live panel shows ESP32 uptime
and free heap together with LSM6DSO32 temperature, acceleration, and angular
velocity. Disconnect it before opening another serial monitor; the GUI does
this automatically before serial recording or firmware flashing.

## GUI files

- `app.py` builds the main window and shared controls.
- `recording_tab.py` contains sensor, recording, and recording-browser actions.
- `simulation_tabs.py` contains Simulation and Motion AI.
- `project_tabs.py` contains Robot and Info.
- `process_manager.py` starts and stops the programs opened by the GUI.
- `project_support.py` contains paths and project discovery helpers.

Output from started programs appears in the Activity panel.
