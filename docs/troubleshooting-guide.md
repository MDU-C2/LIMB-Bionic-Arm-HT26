# Troubleshooting

## Start asks for internet

Use **Start** after the first **Setup**. Start only launches the saved environment;
it does not run installation. Setup and the first firmware build need internet.
If the environment is missing, run Setup explicitly.

## Record raw signals

Open **Recording**, enter the subject ID and connect camera + BLE. Wait for
valid stereo arm points, then press **Calibrate subject**. The operator guides
the timed reference movements and EMG steps. Normal trial start requires all
calibration checks to pass; the status identifies the failed step.

The table can hide the hips. Calibration needs both shoulders, elbow and wrist
in stereo, but does not need finger or cup detection. Keep reference movements
clear of the table. For RAISE, hold the elbow angle. For BEND and TURN, keep the
upper arm still. Start by raising/bending first, as instructed. If a sensor moves
or BLE reconnects, calibrate again. **Signals** shows the received raw samples.

Trials default to a three-second countdown and eight seconds of capture with
automatic saving. Increase the window before the next trial if needed. Cancel
before GO to avoid creating files. Early stops keep incomplete attempts.

Missing depth maps after a trial starts flag an incomplete capture; raw RGB and
BLE remain saved. If stereo cannot run, fix the camera connection before normal
collection. `--raw-only --no-depth` is available for command-line diagnostics.
See the [recording guide](recording-guide.md) for the files and calibration checks.

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

## AURORA is missing from a scan

Earlier LIMB firmware advertises `LIMBServer`; the default AURORA scan accepts
either name. Choosing another name or an explicit address selects that device.

The ESP32 stops advertising while connected. Close other AURORA sessions or
sensor programs that may own the connection. The app releases its BLE preview
before starting recording or live simulation.

The ESP32 can use a separate USB power source. A PC serial connection is not
needed for BLE. If it still does not appear, restart its power and scan again.

The live graphs show missed packets per stream as running totals. These are
earlier packet gaps, not a count of disconnected sensors. Check `IMUs 2/2` and
`EMG receiving` for the current streams. Reconnecting starts a new sequence check
without clearing the totals.

## EMG calibration is too weak

Compare the filtered rest and contraction levels. Large raw ADC swings can
come from drift. Check electrode contact, the reference connection and amplifier
gain, then repeat rest and strong-grip calibration with the electrodes attached.
Starting a new grip calibration clears the old reference. A failed attempt
leaves activation unavailable until calibration succeeds.

## Arm movement does not match

Check which IMU is mounted on the upper arm and which is on the forearm. Pause
and follow the live setup panel again: hold still at marker 1 for alignment,
calibrate relaxed EMG, then firm grip, and return to marker 1 before starting.
Follow the mounting and axis checks in the [simulation guide](simulation-guide.md).

## Diagnostic commands

Run from the project environment after closing the app:

```powershell
python scripts/check_ble.py --name AURORA
python scripts/check_oak_camera.py
python scripts/check_emg_calibration.py --phase grip --interactive --seconds 6
python scripts/diagnose_live_control.py --depth --mode dynamic --seconds 20
```

The live-control diagnostic opens camera and simulator windows and saves its
results under `outputs/`.
