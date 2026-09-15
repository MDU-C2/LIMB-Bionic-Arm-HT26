# Tests

The repository has a small automated BLE capture test. Broader automated
coverage is a handover priority before hardware control is added.

Current software checks from the repository root are:

```powershell
micromamba run -n aurora-simulation python -m compileall -q src
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
micromamba run -n aurora-simulation python -m unittest discover -s tests -v
```

Future tests should cover additional firmware packet formats, pose mapping,
joint limits, trajectory loading, recorder cleanup, and GUI discovery.
Hardware-in-the-loop tests must identify the connected equipment and must not
actuate the arm by default.
