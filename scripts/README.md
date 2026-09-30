# Scripts

This folder contains small standalone checks.

To require complete live acceleration and gyroscope values from both ESP32
IMUs (the port is auto-detected when possible):

```powershell
micromamba run -n aurora-simulation python scripts/check_dual_imu.py --port COM5
```

This exits with a failure code when either address is absent and translates the
firmware's pull-up continuity fields into a wiring-specific diagnostic.

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
