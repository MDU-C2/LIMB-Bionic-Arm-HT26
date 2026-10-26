# Setup and checks

Run `Setup.bat` once on Windows, or `Setup.command` on macOS/Linux. Setup needs
internet to install Python and the recording/simulation packages.

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

Training and firmware tools are optional. From the project Python:

```powershell
python -m pip install -r requirements-training.txt
python -m pip install -r requirements-firmware.txt
```

The first firmware build downloads its toolchain. Training and firmware downloads
are not needed to open the app or record signals. On Windows/Linux, a CPU-only
PyTorch install can be selected using the index at https://download.pytorch.org/whl/cpu.

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
