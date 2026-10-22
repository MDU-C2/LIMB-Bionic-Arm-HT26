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

To run directly from the repo after setup, select that project Python and run:

```powershell
python -m aurora.gui.app
```

Running `src/aurora/gui/app.py` directly, including the editor's Run button,
uses the checkout's launcher to select or set up the project environment.

The manual environment setup is also available:

```powershell
micromamba create -f environment.yml
micromamba run -n aurora python -m pip install -r requirements-setup.txt
micromamba run -n aurora python -m pip install --no-deps -e .
micromamba run -n aurora python -m aurora.gui.app
```

Runtime dependencies are declared in `pyproject.toml`. Training, firmware and
development tools use its optional dependency groups. The requirements files
provide those groups to Setup and standalone checks. `environment.yml` creates
the base Python/native environment; Setup then installs the app and its dependencies
from the checkout.

An installed wheel includes models, guides, demo trajectories and firmware
sources. In a checkout, outputs go under that checkout. With a wheel installed,
outputs go under the directory where the app is launched. Python needs Tk
support; the managed environment supplies it.

Run these checks from the project environment:

```powershell
python scripts/check_ble.py --name AURORA
python scripts/check_dual_imu.py --port COM5 --require-emg
python scripts/check_oak_camera.py
python scripts/check_emg_calibration.py --phase grip --interactive --seconds 6
```

The COM-port check is for USB diagnostics. Normal recording uses BLE.
Close the app before running a separate BLE check so it can connect to the ESP32.

Run `python scripts/check_project.py` for the software tests. See
[`tests/`](../tests/README.md) for code style checks.
Run `python scripts/check_package.py` to build and check an installed copy.
