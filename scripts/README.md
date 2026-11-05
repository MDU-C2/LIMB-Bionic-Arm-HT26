# Setup and checks

Run `Setup.bat` once on Windows, or `Setup.command` on macOS/Linux. Setup needs
internet to install Python and all app packages, including training and firmware tools.

Use `Start.bat` or `Start.command` afterwards. Start only launches the installed
app. It does not install packages, build firmware or require internet. Run Setup
again explicitly when dependencies change or the environment needs repairing.

On Windows, the managed Python is under `%LOCALAPPDATA%/aurora/`.
`.aurora/runtime.json` records its path. On macOS/Linux it is in `.aurora/env/`.
Select that Python in the editor to run directly:

```powershell
python -m aurora.gui.app
```

Running `src/aurora/gui/app.py` directly also selects the saved environment.
It does not run installation. The managed Windows launcher sets `Library/bin`
on PATH for native libraries.

Setup downloads and checks all three MediaPipe pose models: Fast (lite), Balanced
(full) and Detailed (heavy). Camera tools can use them offline after Setup.
It also downloads the pinned ESP32 toolchain and checks a firmware build.
It does not flash the board or require connected hardware. A failed dependency,
pose-model or firmware check stops Setup before it reports success. Run Setup again to
repair an earlier installation that is missing tools.

## Sensor checks

Close AURORA before a separate BLE/camera check:

```powershell
python scripts/check_ble.py --name AURORA
python scripts/check_oak_camera.py
python scripts/check_emg_calibration.py --phase grip --interactive --seconds 6
```

USB diagnostics, when needed:

```powershell
python scripts/check_dual_imu.py --port COM5 --require-emg
```

Run `python scripts/check_project.py` for offline checks. Training tests need the
training packages; style checks are listed in [tests](../tests/README.md).
`python scripts/check_package.py` checks a built package and its resources.

To use the saved Windows environment from PowerShell:

```powershell
$runtimePrefix = (Get-Content -Raw .aurora/runtime.json | ConvertFrom-Json).prefix
$env:PATH = "$runtimePrefix/Library/bin;$env:PATH"
& "$runtimePrefix/python.exe" -m aurora.gui.app
& "$runtimePrefix/python.exe" scripts/check_project.py
& "$runtimePrefix/python.exe" scripts/check_package.py
```

The default check suite covers runtime software without connected devices.
`--suite training` checks the training packages; `--suite all` includes
both suites. A missing training prerequisite is reported separately.

`benchmark_runtime.py --output outputs/diagnostics/runtime.json --ui --physics-gui`
compares synthetic sensor processing, queued recording, DIRECT/GUI physics and
Tk responsiveness. Use `--inference-only` in a separate process to time camera
models on blank images. These runs report host costs, CPU time, memory and latency
percentiles. They do not measure BLE transport, camera capture or tracking accuracy.
Add `--keyboard-gui` to compare the complete keyboard frontend with display
updates disabled/enabled, or `--ui --browser-files 2000` to stress the browser.

The package base needs NumPy and SciPy. Optional groups are `simulation`, `sensors`,
`camera`, `inference`, `plots`, `training`, `firmware` and `dev`. Setup installs the
app feature groups, including training and firmware. The `dev` group is separate.
Ordinary Start never installs missing groups.
