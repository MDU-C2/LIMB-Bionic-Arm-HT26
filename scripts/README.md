# Scripts

This folder contains small standalone checks.

To test the OAK-D RGB camera without recording:

```powershell
micromamba run -n aurora-simulation python scripts/check_oak_camera.py
```

Press `Q` or `Esc` to close the camera window. Use the GUI when you want pose
tracking or saved data.
