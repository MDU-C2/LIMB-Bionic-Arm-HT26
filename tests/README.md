# Tests

Run software checks from the repository root:

```powershell
micromamba run -n aurora-simulation python -m compileall -q src
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
micromamba run -n aurora-simulation python -m unittest discover -s tests -v
```

The tests cover BLE capture, sensor previews, camera tracking, and headless
PyBullet dynamics. They do not validate connected hardware or the GUI windows.
