# Live control diagnosis, 12 October 2026

Changes were made in the current HT26 working tree. The related HT25 sensor
logic and offline EMG filters, Jonas AURORA ADC/BLE source, and DMP-arm camera
preview/3D capture source were inspected for comparison.

## Reproduced failures

1. Running the managed interpreter without its `Library/bin` on Windows PATH
   caused a native MKL/BLAS exception, including `0xc06d007f` in a numerical
   solve. Supplying the directory made NumPy and SciPy solves pass. The GUI
   now supplies the selected interpreter's DLL paths to children; the simulator
   entry point also configures them for direct launches. Native fault logging is
   enabled for GUI children.
2. The connected OAK-D with the default DepthAI 3.10 stereo preset lost its RGB
   and depth streams with `X_LINK_ERROR`. FAST_DENSITY, subpixel disabled, and
   left/right checking enabled delivered stereo arm angles. A subsequent
   20-second dynamic simulator run completed with no error. RGB-only control
   was also checked. Initial camera-window visibility is checked only after a
   frame arrives, avoiding a premature exit while the camera boots.
3. Live-start cleanup was duplicated. Cleanup now belongs to the simulator
   lifecycle so partially initialized windows and hardware are released once.

## Grip behavior

The previous normalization peak decayed over eight seconds. That amplified the
same contraction after resting. It also processed raw EMG without the legacy
project's frequency filtering. High-rate live processing now has a causal
20–450 Hz band-pass, 50 Hz notch, RMS envelope, resting noise floor, and a
three-second strong-grip reference. The calibrated reference remains fixed.
Rest and strong-grip buttons are available in both standalone and simulator
graphs. IMU alignment preserves EMG calibration. Clipping and inadequate
calibration are reported. The [simulation guide](SIMULATION.md) describes use.

Synthetic checks verified approximately 48% output for half of the calibrated
signal amplitude, repeatability after 30 seconds of rest and stronger effort,
mains/drift rejection, clipping, and rejection of a weak calibration trial.
All 15 test modules passed when run in separate processes (154 tests); this
avoids the installed camera/training packages' conflicting OpenMP runtimes.

## Physical validation remaining

The diagnostic could open the camera and produce depth-backed arm angles, but
could not discover a fresh LIMB BLE stream. The existing AURORA application
remained running; its connection may prevent advertising. No fresh on-arm EMG
or IMU measurements were obtained, and physical grip accuracy is unverified.
Electrode placement and a relaxed/strong-grip trial must be checked in the
updated live simulator. A single EMG channel measures activation, not grip force.

### BLE-only follow-up on the same day

With the ESP32 powered from a separate USB source, inspecting the open GUI
confirmed **2/2 IMUs | EMG receiving**. Its red failure banner concerned the
simulator/camera, not the BLE connection. After closing the old GUI normally,
`LIMBServer` advertised with the expected service UUID. A fresh GUI session
again showed both IMUs and EMG, and its live graphs reported **BLE Connected**
and **Grip calibrated**. The inspected EMG trace ranged from about 1968 to 2182
ADC counts; activation at that snapshot was 1.3%. This verifies live BLE
reception with external power, but does not establish physical grip accuracy.

The firmware stops advertising while its central is connected. A separate
name-based scan can therefore miss the board even while the GUI is receiving
data. Use one AURORA session; its live simulator handles releasing the preview
connection before connecting. Neither mode needs a PC USB/COM sensor connection.

Bounded reproduction:

```powershell
python scripts/diagnose_live_control.py --depth --mode dynamic --seconds 20
```

Use the managed interpreter from `.aurora/runtime.json`. Reports go into the
ignored `outputs/` directory. The diagnostic opens simulator/camera windows and
does not command physical actuators or flash firmware.
