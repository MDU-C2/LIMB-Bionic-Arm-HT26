# Tests

Run the software checks from the repository root:

```powershell
micromamba run -n aurora-simulation python -m compileall -q src
Get-ChildItem tests/test_*.py | ForEach-Object {
    micromamba run -n aurora-simulation python -m unittest discover -s tests -p $_.Name -v
    if ($LASTEXITCODE -ne 0) { throw "Failed: $($_.Name)" }
}
micromamba run -n aurora-simulation python src/simulation/sim/limb_sim.py --headless
```

The automated tests cover sensor decoding, previews, recording setup, camera
processing, simulation behavior, and installer recovery/launch behavior.
Installer tests run without downloading packages or flashing hardware.
Run test modules in separate Python processes: the installed training and camera
packages load different OpenMP runtimes, which abort when combined in one process.
Hardware is checked separately with
`scripts/check_dual_imu.py` and `scripts/check_oak_camera.py`; no automated test
can validate a disconnected IMU or physical robot.

EMG checks use synthetic signals to verify noise rejection, Nyquist limits,
timestamp gaps and RMS/MAV windows. GRU checks verify preprocessing and cached
validation embeddings; they are skipped if PyTorch is not installed.
Live EMG calibration checks cover proportional response, repeatability after
30 seconds of rest and stronger effort, mains/drift rejection, clipping and
weak calibration trials.

BLE live-control checks feed real firmware packet layouts through decoding,
arm fusion, and EMG grip processing. They also cover independent stream expiry,
recalibration, reconnecting, cancellation during scanning, and the GUI command
without serial settings. Bluetooth and camera operation require connected hardware.

Interactive runtime checks use real PyBullet DIRECT physics and a headless
Pygame display to exercise the split task modules, including manual movement,
grasp/release/reset, live calibration, and dynamic torque CSV output. Live grip
checks reproduce resting EMG with camera opening/closing, verify finger-servo
speeds and loaded URDF bounds, and exercise manual rotation limits plus camera
axial priority.

IMU twist tests cover signed mounting axes, resting gyro bias, isolated elbow and
forearm movement, common upper-arm movement with a bent elbow, packet-rate
independence, camera correction, sensor loss and immediate reversal at stops.
BLE-to-PyBullet checks verify both rotation joints using the firmware's actual
packet layout and units. Human movement still needs the mounting checks in the
simulation guide.
