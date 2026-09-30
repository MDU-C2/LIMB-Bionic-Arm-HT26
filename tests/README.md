# Tests

Run the software checks from the repository root:

```powershell
micromamba run -n aurora-simulation python -m compileall -q src
micromamba run -n aurora-simulation python -m unittest discover -s tests -v
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
```

The automated tests cover sensor decoding, previews, recording setup, camera
processing, and simulation behavior. Hardware is checked separately with
`scripts/check_dual_imu.py` and `scripts/check_oak_camera.py`; no automated test
can validate a disconnected IMU or physical robot.
