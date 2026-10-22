# Troubleshooting

## Live simulation closes

Check the last error under Activity. Run the app through `Start.bat`, or select
the project Python before launching it directly. On Windows, numerical libraries
need the environment's `Library/bin` directory. The app sets this for its child
programs.

`OAK-D camera unavailable` means the camera could not be opened. Check its USB
data connection, close other camera previews and retry after a few seconds.
Failed programs show an error dialog and keep their output in `outputs/logs/`.

If the camera reports `X_LINK_ERROR`, try live control with stereo depth off.
Close other camera programs before retrying. The stereo pipeline uses the
FAST_DENSITY preset with subpixel processing disabled to reduce its load.

## LIMBServer is missing from a scan

The ESP32 stops advertising while connected. Close other AURORA sessions or
sensor programs that may own the connection. The app releases its BLE preview
before starting recording or live simulation.

The ESP32 can use a separate USB power source. A PC serial connection is not
needed for BLE. If it still does not appear, restart its power and scan again.

## EMG calibration is too weak

Compare the filtered rest and contraction levels. Large raw ADC swings can
come from drift. Check electrode contact, the reference connection and amplifier
gain, then repeat rest and strong-grip calibration with the electrodes attached.
See the [signal check](EMG_SIGNAL_CHECK.md) for the measured limitations.

## Arm movement does not match

Check which IMU is mounted on the upper arm and which is on the forearm. Match
the displayed reference pose, press L and hold still for half a second. Follow
the mounting and axis checks in the [simulation guide](SIMULATION.md).

## Diagnostic commands

Run from the project environment after closing the app:

```powershell
python scripts/check_ble.py --name LIMBServer
python scripts/check_oak_camera.py
python scripts/check_emg_calibration.py --phase grip --interactive --seconds 6
python scripts/diagnose_live_control.py --depth --mode dynamic --seconds 20
```

The live-control diagnostic opens camera and simulator windows and saves its
results under `outputs/`.
