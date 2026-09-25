# GUI

Run the control center with:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

## Main workflow

| Tab | Use |
| --- | --- |
| Simulation | Run the interactive task arm by keyboard or live camera/IMU fusion. |
| Recording | Capture OAK-D and the dual-IMU serial stream as one session. |
| Recordings | Browse files created by the recorders. |
| Motion AI | Play camera poses or run movement recognition. |
| Firmware | Build, flash, and monitor the ESP-IDF project. |
| Info | Check local files, tools, and dependencies. |

IMU values are intentionally not embedded in the main window. **Open IMU
monitor** opens a dedicated window with separate shoulder and wrist panels. The
GUI connects to the selected serial port at startup when a port is available.
It releases the port before recording, flashing, or starting live sensor fusion.

**Open camera monitor** starts the existing OAK-D pose preview without creating
a recording. Live interactive control opens its own annotated camera monitor,
so close the standalone preview before starting that mode.

The Recording page defaults to the two current sources: OAK-D and the ESP32
dual-IMU serial stream. The legacy LIMB25 BLE recorder is still discoverable for
compatibility, but its extra cuff channels do not clutter the main workflow.

## GUI files

- `app.py` composes the window, tabs, shared controls, and activity panel.
- `sensor_window.py` renders the separate dual-IMU monitor.
- `recording_tab.py` coordinates preview, capture, and recording browsing.
- `simulation_tabs.py` launches simulation, fusion, and Motion AI tools.
- `project_tabs.py` handles ESP-IDF and project status.
- `process_manager.py` supervises programs opened by the GUI.
- `project_support.py` contains paths and discovery helpers.
