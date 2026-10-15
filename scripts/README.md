# Setup and checks

Run `Setup.bat` once on Windows, then use `Start.bat`.
Setup installs the project environment, checks the simulation and builds the
firmware. Flashing is a separate action in the app.

On macOS and Linux, use `bash Setup.command` and `bash Start.command`.
Windows has been tested. The other launchers still need testing.

The first setup needs internet access and several GB of disk space. Run Setup
again if installation was interrupted. To install without opening the app:

```powershell
.\Setup.bat -NoLaunch
```

Windows environments are stored under `%LOCALAPPDATA%\aurora`. The local
`.aurora/runtime.json` file records the Python path. If that Windows path has
spaces or non-ASCII characters, set `AURORA_RUNTIME_ROOT` to a writable ASCII
path before running Setup. On macOS and Linux, the environment is in `.aurora/`.

To run directly from the repo, select that project Python and run:

```powershell
python src/gui/app.py
```

The manual environment setup is also available:

```powershell
micromamba create -f src/simulation/environment.yml
micromamba run -n aurora-simulation python -m pip install -r requirements-setup.txt
micromamba run -n aurora-simulation python src/gui/app.py
```

Run these checks from the project environment:

```powershell
python scripts/check_ble.py --name LIMBServer
python scripts/check_dual_imu.py --port COM5 --require-emg
python scripts/check_oak_camera.py
python scripts/check_emg_calibration.py --phase grip --interactive --seconds 6
```

The COM-port check is for USB diagnostics. Normal recording uses BLE.
Close the app before running a separate BLE check so it can connect to the ESP32.
