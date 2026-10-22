# App

Run `Start.bat` from the repo root. For a new setup, use `Setup.bat` first.
To launch directly, select the project Python and run `python -m aurora.gui.app`.

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

Live simulation uses the Bluetooth name `AURORA`. Follow the setup panel:
align the starting pose, calibrate relaxed EMG, calibrate firm grip, then start
control. Keep the open hand at marker 1 with the thumb up and elbow about 90°.
Press R to pause and reset. K and L advance the current setup step.

The table is 60 x 60 cm. The arm is mounted on the left, with the cup to its
right. The green disc marks the placement area farther right.

See the [recording guide](../../../docs/recording-guide.md) and
[simulation guide](../../../docs/simulation-guide.md) for the full controls.

## Files

- `app.py`, `app_layout.py`: window and shared controls.
- `recording_tab.py`, `recording_views.py`, `recording_settings.py`: recording setup.
- `cup_recording.py`: subject calibration and repeated cup trials.
- `recording_browser.py`: saved files and plots.
- `ble_dashboard.py`, `serial_dashboard.py`, `sensor_window.py`, `sensor_graphs.py`: sensor monitors.
- `sensor_plot.py`: drawing shared by the live graphs and standalone preview.
- `simulation_tabs.py`: keyboard and live simulation setup.
- `motion_tools.py`: movement comparison and pose playback.
- `project_tabs.py`, `firmware_workspace.py`: firmware actions and project status.
- `process_manager.py`, `project_support.py`: child programs, paths and tool checks.

Program output is kept under `outputs/logs/`. A failed recording or simulation
shows its error and log path in the app.

In Recordings, select a trial or CSV and press **Plot signals** to inspect it.
Use **Process EMG** afterward; the raw files stay unchanged.
