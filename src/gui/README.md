# GUI

For the easiest setup, double-click `Setup.bat` at the repository root, then
use `Start.bat` for later launches. On macOS/Linux, use `Setup.command` and
`Start.command`. See the [root README](../../README.md) for platform details.

If you use an existing manually installed environment, run:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

## Main workflow

| Tab | Use |
| --- | --- |
| Simulation | Run the interactive task arm by keyboard or live camera/IMU/EMG control. |
| Recording | Capture OAK-D, dual-IMU, and EMG data as one synchronized session. |
| Recordings | Browse files created by the recorders. |
| Motion AI | Play camera poses or run movement recognition. |
| Firmware | Build, flash, and monitor the ESP-IDF project through the project environment. |
| Info | Check local files, tools, and dependencies. |

Live values are intentionally not embedded in the main window. **Open sensor
monitor** opens a dedicated window with shoulder, wrist, and EMG panels. The
GUI connects to the selected serial port at startup when a port is available.
It releases the port before recording, flashing, or starting live sensor fusion.

**Open camera monitor** starts the existing OAK-D pose preview without creating
a recording. Live interactive control opens its own annotated camera monitor,
so close the standalone preview before starting that mode.

The Recording page defaults to OAK-D plus both IMU and EMG Bluetooth streams
published by the current firmware. A selected unavailable characteristic does
not stop the other connected stream. Device
timestamps are mapped to the same host monotonic clock saved with camera frames.
The serial sensor stream remains available for diagnostics and the legacy LIMB25
BLE characteristics remain readable.

## GUI files

- `app.py` composes the window, tabs, shared controls, and activity panel.
- `sensor_window.py` renders the separate IMU and EMG monitor.
- `recording_tab.py` coordinates preview, capture, and recording browsing.
- `simulation_tabs.py` launches simulation, fusion, and Motion AI tools.
- `project_tabs.py` handles native ESP-IDF commands and project status.
- `process_manager.py` supervises programs opened by the GUI.
- `project_support.py` contains paths and discovery helpers.
