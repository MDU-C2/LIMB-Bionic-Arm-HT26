# AURORA project GUI

`app.py` is the shared Tkinter control center for software that runs from this
repository. It gives every group member the same entry point even when their
clone is stored in a different directory.

Start it from the repository root:

```powershell
micromamba run -n aurora-simulation python src/gui/app.py
```

## Structure

The launcher separates common services from tab-specific controls:

- `TAB_DEFINITIONS` declares the tab keys, labels, order, and builder methods.
- `_build_*_tab()` methods create controls for one feature area.
- `_start_program()` runs one child program at a time and sends its output to
  the Activity panel.
- `_refresh_*()` methods discover scripts, recordings, firmware, and local
  tools without fixed machine paths.
- `_update_controls()` keeps actions disabled when their requirements are not
  available or another program is running.

All repository paths are derived from the location of `app.py`. Paths shown in
the GUI stay relative to the repository when possible.

## Add a tab

Add one entry to `TAB_DEFINITIONS`:

```python
TabDefinition("sensors", "Sensors", "_build_sensors_tab"),
```

Then add the matching builder to `ProjectGui`:

```python
def _build_sensors_tab(self, tab: ttk.Frame) -> None:
    """Populate controls and status for live sensors."""
    tab.columnconfigure(0, weight=1)
    self._heading(tab, "Sensors", "Inspect live robot sensor data.")
    card = self._card(tab, 2, "Sensor source", "Choose and start a source.")
    ttk.Button(card, text="Start", command=self.start_sensors).grid(
        row=2,
        column=0,
        sticky="w",
    )
```

The notebook creates the page and calls the registered builder. Put shared
state in `ProjectGui.__init__`, and call `_update_controls()` after readiness
changes.

## Add a program button

Start repository programs through `_start_program()` so their output, status,
and stop behavior remain consistent:

```python
def start_sensors(self) -> None:
    """Start the maintained sensor reader."""
    script = REPOSITORY_ROOT / "src" / "sensors" / "read_sensors.py"
    command = [str(self.simulation_python or sys.executable), "-u", str(script)]
    self._start_program("Sensor reader", "sensors", command)
```

Pass `working_directory=` when a tool expects a particular directory and
`environment=` when it needs explicit environment variables. Do not put a
group member's absolute path or serial port in the source.

## Recording integration

The Recording tab and **Start recording** button are already connected. The
button becomes available when the launcher finds a Python file below `src`
whose name starts with `record` or `capture` and which has a `__main__` entry
point. For example:

```text
src/recording/record_session.py
```

The launcher passes the selected output directory in `AURORA_OUTPUT_DIR` and
appends the optional arguments entered in the tab. A recorder should read that
variable, create its files below that directory, handle `KeyboardInterrupt`,
and close files and devices before exiting. The Recordings tab recognizes the
file extensions listed in `RECORDING_EXTENSIONS`.

The same tab has a separate **Open training camera** button for OAK-D Lite.
Choose a user ID and arm, open the camera, then press Space to start and Space
again to save a landmark sequence. It uses `src/camera/record_movement.py` and
is supervised by the same Activity-panel stop control.

## Firmware integration

The Robot tab discovers PlatformIO projects by `platformio.ini` and ESP-IDF
projects by their top-level `CMakeLists.txt` below `firmware`. Build, flash, and
monitor output uses the shared Activity panel. Flashing requires a selected
serial port and an explicit confirmation in the GUI.

## Maintenance rules

- Give every module, class, and method a short docstring that states its role.
- Comment the reason for a decision or constraint; keep obvious widget code
  self-explanatory through clear names.
- Keep tab code in its own builder and reuse `_heading()`, `_card()`, and
  `_start_program()`.
- Add a dependency only when the standard library or current simulation
  environment cannot provide the required behavior.
- Update this guide and the Info tab when an entry-point convention changes.
