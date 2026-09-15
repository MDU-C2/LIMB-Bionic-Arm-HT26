# Examples

`simulation/demo` contains the small four-joint dataset used to verify DMP
trajectory playback:

- `angles.npz` contains inherited measured angles;
- `dmp_rollout_clean.npz` contains the prepared playback trajectory.

Run the example from the **Simulation** tab or from the repository root:

```powershell
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
```

Add examples only for maintained interfaces. Keep them small, state their
origin, and avoid data that identifies a participant.
