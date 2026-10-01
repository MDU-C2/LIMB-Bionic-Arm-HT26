# Scripts

This folder contains small standalone checks.

`setup.ps1` and `setup_project.py` power the root-level `Setup.bat`/`Start.bat`
and `Setup.command`/`Start.command` launchers. Setup downloads its own Python,
installs all desktop and ML dependencies, checks native imports and a headless
simulation, and builds the firmware through PlatformIO without flashing it.
Start reuses a completed setup and detects changes to dependency definitions.

To require complete live acceleration/gyroscope values from both IMUs plus EMG
(the port is auto-detected when possible):

```powershell
micromamba run -n aurora-simulation python scripts/check_dual_imu.py --port COM5 --require-emg
```

Omit `--require-emg` for an IMU-only harness. The check exits with a failure
code when required data is absent and translates the firmware's pull-up fields
into a wiring-specific diagnostic.

To test the OAK-D RGB camera without recording:

```powershell
micromamba run -n aurora-simulation python scripts/check_oak_camera.py
```

Press `Q` or `Esc` to close the camera window. Use the GUI when you want pose
tracking or saved data.

For a non-interactive connection check:

```powershell
micromamba run -n aurora-simulation python scripts/check_oak_camera.py --headless --frames 20
```
