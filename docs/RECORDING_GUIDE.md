# Recording guide

Use BLE for EMG and both IMUs. Connect the OAK-D camera to the PC over USB.
The ESP32 can use a separate USB power source.

## Cup transfers

1. Mount the camera on the wall in front of the subject. Frame both shoulders,
   hips, the selected arm, hand and the cup's path across the table.
2. Open **Recording**. Enter a subject ID such as `S01` and choose the arm.
   Leave trial duration at `0` to stop each trial manually, or enter seconds.
3. Press **Connect camera + BLE**. The app releases its standalone previews.
   Wait for both IMUs and EMG. Use **Signals** to inspect the graphs.
4. Press **Hold still at 1: align IMUs** and keep the arm still in its starting pose for half
   a second. Press **Relaxed EMG** while relaxed, then **Firm grip (3 s)**
   and follow the countdown. Wait for `Grip calibrated`.
5. Press **Record T001**, move the coffee cup, then press **Stop and save**.
   Repeat with **Record T002** and the following trials. Camera and BLE stay
   connected and calibration carries over between trials.
6. Press **Finish subject**, enter the next subject ID and repeat calibration.
   Recalibrate if the electrodes or IMUs move. A Bluetooth reconnect also
   requires calibration again.

Normal recording uses the RGB camera with the full pose model. Stereo depth
is off. Cup detection runs locally and shows a box with its score. Hand tracking
uses a closer wrist crop. Fingers hidden behind the cup can still be missed;
missing finger landmarks are left unavailable. Keep the raw files even when a
trial needs repeating.

Choose **natural**, **slow** or **fast** before each trial. The movement is:
hand at 1 → grasp the cup at 2 → place and release at 3 → empty hand at 4.
Use the [subject protocol](CUP_TRANSFER_PROTOCOL.md) for the spoken instructions,
practice, trial order and rest periods. Optional marker buttons save operator
timestamps; **Mark 4 + save** ends the trial.

## Saved files

Trials are grouped automatically:

```text
outputs/recordings/S01/T001_DATE_TIME/
    trial.json
    ble/
        emg.csv
        imu.csv
        packets.jsonl
        meta.json
    camera/
        video.mp4
        pose.json
        meta.json
```

`trial.json` keeps subject, arm, trial ID, calibration values, packet health,
clock-sync details, requested pace, marker definitions, operator events and the
completion status. A disconnected or interrupted
trial is marked incomplete. A completed trial still needs a signal-quality check.
The output folder can be changed under **Advanced recording options**.

`pose.json` keeps timestamps and observations, including visible arm/hand
landmarks, cup boxes and scores, and available joint estimates. Cup boxes use
raw camera pixels; the preview and saved video are mirrored. With stereo off, `observations`
contains the camera tracking and `data` has no depth-backed pose frames.
Monocular 3D angles are estimates. Repeated video frames preserve playback
duration; the observation count shows the actual tracking rate.

Use `synced_host_monotonic_ns` in the BLE files and `host_monotonic_ns` in the
camera observations for alignment. Time-sync metadata records the estimated
uncertainty. These times do not establish exact simultaneous acquisition.
Full-rate EMG is about 4,000 samples/s; each connected IMU is about 100 samples/s.
Keep participants separate when evaluating a model on new people.

## Advanced recordings

Expand **Advanced recording options** for individual sources, USB diagnostics,
custom camera arguments or guided grip trials. The existing recorders still
save separate source folders with shared subject and session metadata.

For a guided grip trial, select Bluetooth and EMG, enable **Guided grip trial**
and set duration to `0`. Five repeats take about 38 seconds. Follow the relax,
close, hold and open cues. Stop a separate camera recorder afterward.

The BLE folder adds `emg_trial.json` cue times and `emg_trial_features.csv`
with RMS and mean absolute value. Cue times are labels to review against the
actual movement. Cup transfers have a trial-level task name; grasp and release
phases are not automatically labelled.

The marker buttons add operator observations. They do not automatically establish
exact grasp and release times. Existing recordings without pace metadata remain
unspecified; assign labels only after checking what was actually recorded.

Process a saved EMG file from the project environment:

```powershell
python src/recording/emg_processing.py --file outputs/recordings/S01/TRIAL/ble/emg.csv
```

This preserves the raw file and exports filtered RMS/MAV features. Samples
must be regularly spaced; review gaps before processing. For corrected grip
labels, add `--labels PATH/emg_trial.json`.

Finish recording before starting live simulation. It uses the same camera and
BLE device. See [troubleshooting](TROUBLESHOOTING.md) for connection problems
and [sensor setup](SENSOR_DATA.md) for wiring.
