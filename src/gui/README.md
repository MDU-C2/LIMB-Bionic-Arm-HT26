# App

Run `Start.bat` from the repo root. For a new setup, use `Setup.bat` first.
To launch directly, select the project Python and run `python src/gui/app.py`.

| Tab | Use |
| --- | --- |
| Simulation | Keyboard or live sensor control of the arm |
| Recording | Calibrate once and record repeated cup transfers per subject |
| Recordings | Browse and plot saved files |
| Motion AI | Play saved poses or compare movements |
| Firmware | Build, flash and monitor the ESP32 |
| Info | Check installed tools and project files |

The BLE graphs open at startup. The header shows whether both IMUs and EMG
are receiving. Recording and live simulation take over that connection when
started. The monitor reconnects after they finish.

Under Recording, connect camera and BLE, calibrate, then use **Record** and
**Stop and save** for each cup transfer. Trial numbers advance automatically.
Use **Finish subject** before changing participant.

Use **Guided grip trial** under Advanced recording options to collect repeated rest, grip,
hold and release phases. Follow the cues on that page. Saved cue times can
be corrected before generating labelled features.

Live simulation uses the Bluetooth name `LIMBServer`. Match the displayed
reference pose and press L. Press R to reset and K to recalibrate at the current
pose. EMG needs a rest calibration and a strong-grip calibration in that session.

The table is 60 x 60 cm. The arm is mounted on the left, with the cup to its
right. The green disc marks the placement area farther right.

See the [recording guide](../../docs/RECORDING_GUIDE.md) and
[simulation guide](../../docs/SIMULATION.md) for the full controls.

## Files

- `app.py`, `app_layout.py`: window and shared controls.
- `recording_tab.py`, `recording_views.py`, `recording_settings.py`: recording setup.
- `cup_recording.py`: subject calibration and repeated cup trials.
- `recording_browser.py`: saved files and plots.
- `ble_dashboard.py`, `serial_dashboard.py`, `sensor_window.py`: sensor monitors.
- `simulation_tabs.py`: simulation and movement tools.
- `project_tabs.py`, `firmware_workspace.py`: firmware actions and project status.
- `process_manager.py`, `project_support.py`: child programs, paths and tool checks.

Program output is kept under `outputs/logs/`. A failed recording or simulation
shows its error and log path in the app.
