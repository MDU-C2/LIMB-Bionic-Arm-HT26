# AURORA - Adaptive User Robotic Rehabilitation Arm

AURORA is a student project at Mälardalen University. This repository currently
contains the computer-side software for sensor previews, synchronized recording,
camera tracking, movement data, robot simulation, and ESP32-C3 dual-IMU/EMG
firmware.

## Run the program

After cloning, **double-click `Setup.bat` on Windows**. It downloads Python and
installs the GUI, simulation, sensor/camera packages, ML training dependencies,
and PlatformIO. It also builds the ESP32-C3 firmware to download and check the
complete toolchain, then opens the GUI. No Python, Conda, or VS Code installation
is needed beforehand.

For later launches, **double-click `Start.bat`**. Start also runs setup if this is
a fresh clone or the dependency definitions have changed.

| Computer | First run | Later runs |
| --- | --- | --- |
| Windows 10/11, Intel/AMD 64-bit | Double-click `Setup.bat` | Double-click `Start.bat` |
| macOS 13+, Intel or Apple Silicon | Open `Setup.command` | Open `Start.command` |
| Linux, Intel/AMD 64-bit desktop | `bash Setup.command` | `bash Start.command` |

The first run needs internet access, several GB of free disk space, and several
minutes. Later launches reuse the installed environment. Training uses CPU
PyTorch so a graphics card or CUDA installation is unnecessary. Setup does not
flash a connected board; use the GUI's **Firmware** tab when ready.

The installer has been tested on Windows. The macOS/Linux launchers still need
validation on those operating systems.

On Windows, each clone gets its own environment under `%LOCALAPPDATA%\aurora`.
This avoids native-tool failures on spaces and non-ASCII characters in OneDrive
paths. The clone's ignored `.aurora/runtime.json` records its environment path.
On macOS/Linux, the Python environment and its package cache live in the ignored
`.aurora/` folder inside the clone. PlatformIO caches its SDKs in `~/.platformio`
on all platforms. Existing Python installations and shell profiles are
left alone. Run Setup again to repair an interrupted installation.

Hardware still needs the actual camera/sensors and working USB/Bluetooth access.
Linux needs a graphical desktop, `curl`, `tar`, `bzip2`, OpenGL libraries, and may
need [USB/serial permissions](https://docs.platformio.org/en/stable/core/installation/udev-rules.html)
and [OAK USB rules](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide/).
Native Windows ARM, 32-bit systems, and Alpine Linux are not supported by this
setup. If a Windows account's local-data path contains spaces or non-ASCII
characters, set `AURORA_RUNTIME_ROOT` to a writable absolute ASCII path without
spaces before running Setup. macOS may require **Open** from the context menu
for downloaded scripts; if needed, run `bash Setup.command` in Terminal.

To install without opening the GUI:

```powershell
.\Setup.bat -NoLaunch
```

On macOS/Linux, use `bash Setup.command --no-launch`.

The original manual environment workflow remains available:

```powershell
micromamba create -f src/simulation/environment.yml
micromamba run -n aurora-simulation python -m pip install -r requirements-setup.txt
micromamba run -n aurora-simulation python src/gui/app.py
```

The GUI can:

- monitor shoulder/wrist IMUs and EMG in a dedicated window;
- monitor the OAK-D camera and pose tracking without recording;
- record clock-synchronized dual-IMU, EMG, and OAK-D camera data together;
- fuse live camera and IMU angles while EMG controls the simulated grip;
- run the interactive PyBullet arm simulation;
- play saved camera poses; and
- run the experimental movement-recognition tools.

## Main folders

| Folder | Contents |
| --- | --- |
| `src/gui/` | The desktop application. |
| `src/recording/` | BLE, camera, and serial preview and recording tools. |
| `src/simulation/` | PyBullet simulation and Motion AI tools. |
| `firmware/` | ESP-IDF firmware for the dual-IMU/EMG ESP32-C3. |
| `src/ml/` | Movement-data capture and model training. |
| `data/` | The movement training data currently included in the project. |
| `docs/` | Short usage guides. |
| `tests/` | Automated software tests. |

Generated recordings are written below `outputs/recordings/` and should not be
committed to Git.

More details are in the [GUI guide](src/gui/README.md),
[recording guide](src/recording/README.md), and
[simulation guide](src/simulation/README.md).
