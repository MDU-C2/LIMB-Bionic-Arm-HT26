# Tests

Run the software checks from the repository root:

```powershell
micromamba run -n aurora-simulation python -m compileall -q src
micromamba run -n aurora-simulation python -m unittest discover -s tests -v
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
```

The tests cover sensor decoding, previews, recording setup, camera processing,
and simulation behavior. They do not test a real ESP32, IMU, OAK-D, or robot.
