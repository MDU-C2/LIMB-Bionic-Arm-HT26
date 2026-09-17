# AURORA control center

`app.py` launches the project's desktop tools from repository-relative paths.

Start it from the repository root:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

Child-program output appears in Activity. PyBullet and camera previews open in
separate windows.

## Architecture

- `TAB_DEFINITIONS` sets the tab keys, labels, order, and builder methods.
- `_build_*_tab()` methods create the contents of one tab.
- `_start_program()` manages one child program and captures its output.
- `_refresh_*()` methods discover recorders, recordings, serial ports, and
  firmware projects.
- `_update_controls()` disables actions whose requirements are unavailable.

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
| `AURORA_CAMERA_SIDE` | Subject arm selected for OAK-D tracking; left by default. |
| `AURORA_BLE_DEVICE` | BLE device name selected in the GUI. |
| `AURORA_SERIAL_PORT` | Selected serial port. |
| `AURORA_SERIAL_BAUD` | Selected serial baud rate. |
| `AURORA_TRIAL_ID` | Trial identifier saved in session metadata. |
| `AURORA_TEST_TYPE` | Selected grip, movement, or combined test type. |

A recorder should create a timestamped session directory, write `meta.json`,
flush data while running, and close devices and files on Ctrl+C. Add new output
extensions to `RECORDING_EXTENSIONS` if the Recordings tab must display them.
The BLE recorder also accepts `--dataset-label` when its labeled-capture
checkbox is selected.

The three maintained sources accept `--preview`. BLE and serial use it for a
read-only monitor. OAK-D always opens a live camera view first; it creates a
session only when START REC is clicked inside that view. The GUI gives OAK-D
one Open camera action and passes the selected subject arm and output settings.

## Firmware integration

The Robot tab discovers PlatformIO projects by `platformio.ini` and ESP-IDF
projects by a top-level `CMakeLists.txt` below `firmware`. Flashing requires a
selected serial port and confirmation in the GUI.
