# AURORA control center

`app.py` is the shared Tkinter launcher for software in this repository. It
uses paths relative to the repository, so each group member can keep the clone
in a different location.

Start it from the repository root:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

The control center has tabs for simulation, recording, saved recordings,
sensors, Motion AI, robot tools, and project information. Programs started by
the GUI write to the Activity panel and can be stopped from the same window.
PyBullet and camera previews open as separate windows.

## Architecture

- `TAB_DEFINITIONS` sets the tab keys, labels, order, and builder methods.
- `_build_*_tab()` methods create the contents of one tab.
- `_start_program()` manages one child program and captures its output.
- `_refresh_*()` methods discover recorders, recordings, serial ports, and
  firmware projects.
- `_update_controls()` disables actions whose requirements are unavailable.

Pages are scrollable so all controls remain available on smaller or scaled
displays. Keep repository paths relative and let users select device names,
ports, and output folders in the GUI.

## Add a tab

1. Add a `TabDefinition` entry in `app.py`.
2. Add the matching `_build_*_tab()` method to `ProjectGui`.
3. Store shared state in `ProjectGui.__init__`.
4. Call `_update_controls()` whenever the new feature changes readiness.
5. Update the Info tab and this guide if the feature adds a new entry point.

Use `_heading()`, `_card()`, and `_start_program()` so new pages behave like
the existing ones.

## Add a recorder

The GUI discovers Python files named `record*.py` or `capture*.py` below `src`
when they contain a `__main__` entry point. Recorders receive these settings:

| Environment variable | Value |
| --- | --- |
| `AURORA_OUTPUT_DIR` | Parent directory for session folders. |
| `AURORA_SUBJECT` | Subject or session label. |
| `AURORA_DURATION_SECONDS` | Duration; `0` means record until stopped. |
| `AURORA_BLE_DEVICE` | BLE device name selected in the GUI. |
| `AURORA_SERIAL_PORT` | Selected serial port. |
| `AURORA_SERIAL_BAUD` | Selected serial baud rate. |

A recorder should create a timestamped session directory, write `meta.json`,
flush data while running, and close devices and files on Ctrl+C. Add new output
extensions to `RECORDING_EXTENSIONS` if the Recordings tab must display them.
The BLE recorder also accepts `--dataset-label` when its labeled-capture
checkbox is selected.

## Firmware integration

The Robot tab discovers PlatformIO projects by `platformio.ini` and ESP-IDF
projects by a top-level `CMakeLists.txt` below `firmware`. Flashing requires a
selected serial port and confirmation in the GUI.

## Maintenance

- Keep obvious widget code clear through names instead of comments.
- Document public behavior and hardware constraints where they matter.
- Start long-running tools through `_start_program()`.
- Never store a personal path, serial port, or device address in source code.
- Keep hardware actions user initiated and report failures in Activity.
