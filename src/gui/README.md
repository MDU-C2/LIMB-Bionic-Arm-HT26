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

The GUI starts a BLE sensor reader and opens live graphs at startup. The header
shows the BLE connection state, connected IMU count and EMG reception.
**Connect BLE / show graphs** reopens the graph window. The USB monitor is
available explicitly under Recording for diagnostic firmware; startup does not
open a serial port. Recording, flashing and live control release the BLE monitor
before starting. It reconnects when the managed programs finish.

**Open camera monitor** starts the existing OAK-D pose preview without creating
a recording. Live interactive control opens its own annotated camera monitor,
so close the standalone preview before starting that mode.

Live interactive control uses the **BLE device** field in Simulation (default
`LIMBServer`) for IMU motion and EMG reception; it does not require a serial port.
**Grip source** defaults to **Camera** so hand tracking can control grip with
unattached electrodes. Select **EMG** for muscle control after checking electrode
contact and rest/contraction signals. **Auto** prefers any current EMG activation,
including a resting stream, and uses camera curl only when EMG is unavailable.
Upper-arm twist uses the IMU and camera correction; relative forearm twist uses
both IMUs. Select the signed axes along each limb; the default is Y for both.
After matching the reference, press L and hold still for 0.5 seconds. Loose
boards do not establish correct anatomical mapping. Camera correction needs a
bent elbow; wrist twist has no camera correction and may drift. Q/E and
Left/Right provide manual rotation when live targets are unavailable. Follow
[the mounting checks](../../docs/SIMULATION.md#mounting-and-twist-calibration).
The table defaults to 60 × 60 cm. Its centerline represents the torso center,
with the shoulder mounted on the left. The cup starts on the left of the table,
to the right of the arm; the green disc is the landing area farther right.
The width/depth fields in Simulation apply to
keyboard and live control. Reset returns to a neutral tabletop pose with a bent
elbow; calibration keeps that displayed pose as the camera/IMU reference.
Its **Open live BLE graphs** button opens the same standalone BLE monitor.
Starting live control releases that monitor and opens its own graph window
using the same BLE connection: raw EMG and both IMUs' acceleration and gyro
traces. Use **Show live BLE graphs** in the simulator's sensor monitor to reopen
it. EMG starts at the full 0–4095 ADC scale; enable auto-scale to inspect small
signals. Auto-scale also magnifies noise, so check the axis values. These views
do not save a recording.

After attaching electrodes, use **1. Calibrate EMG at rest** and relax for half
a second, then **2. Calibrate strong grip (3 s)** while gripping comfortably.
The second step fixes the grip scale so it does not change after resting. Repeat
in the simulator's live graphs after starting live control; the standalone
monitor's calibration belongs to its own session. L/K align the arm without
resetting EMG. See the [EMG instructions](../../docs/SIMULATION.md).

The BLE device name is read-only during normal use. **Change** selects another
advertised device name; it does not rename the board. The **Recording guide**
button on Recording opens the [step-by-step guide](../../docs/RECORDING_GUIDE.md).

The Recording page defaults to OAK-D plus the ESP32 Bluetooth stream with both
IMUs and EMG. The firmware defaults to BLE sensor streaming. Its time-sync
characteristic aligns device sample times with camera frames. A selected unavailable BLE
characteristic does not stop the other connected stream. Legacy LIMB25 BLE
characteristics remain readable.

Before collecting the dataset, flash the current firmware and run a 30-second
trial with both BLE streams selected. Check the saved samples, both IMU roles
and time-sync metadata. Bluetooth delivery and synchronization still need
validation for each setup. See the [connected hardware checks](../../docs/HARDWARE_VALIDATION.md).

## GUI files

- `app.py` composes the window, tabs, shared controls, and activity panel.
- `app_layout.py` builds the shared layout and theme.
- `sensor_window.py` renders the separate IMU and EMG monitor.
- `recording_tab.py` coordinates preview, capture, and recording browsing.
- `recording_views.py` builds recording settings; `recording_settings.py` defines sources.
- `recording_browser.py` handles saved files and filtering.
- `serial_dashboard.py` owns the optional USB diagnostic monitor.
- `ble_dashboard.py` shares the live reader and graphs with the simulator.
- `simulation_tabs.py` launches simulation, fusion, and Motion AI tools.
- `project_tabs.py` handles native ESP-IDF commands and project status.
- `process_manager.py` supervises programs opened by the GUI.
- `project_support.py` contains paths and discovery helpers.
