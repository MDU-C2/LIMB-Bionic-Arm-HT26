# Sensor recording

The control center discovers all three recorders and starts them from the
**Recording** tab. Choose a source in **Sensors**, then set the subject,
duration, device, and output folder in **Recording**.

| Program | Input | Session files |
| --- | --- | --- |
| `record_ble_sensors.py` | LIMB BLE notifications | `packets.jsonl`, `emg.csv`, `imu.csv`, `piezo.csv`, `meta.json` |
| `record_serial_sensors.py` | Newline-based serial messages | `serial.jsonl`, `meta.json` |
| `record_oak_pose.py` | OAK-D video and six arm/trunk landmarks | `video.mp4`, `pose.json`, `meta.json` |

BLE and serial create a session when recording starts. Duration `0` records
until stopped; labeled BLE capture ends after 80 windows. OAK-D opens a live
camera first, showing only the selected shoulder, elbow, and wrist over the
unflipped image. Press R or click **START REC** to record, repeat to stop, and
press Q to close. Its `pose.json` saves six arm/trunk points and angle estimates.
Motion AI playback requires the optional `--depth` mode for 3D points.

## Labeled BLE capture

Select the BLE recorder, check **Labeled BLE capture**, and enter a movement
label. The old dataset uses `1` for holding and `2` for resting. After a
three-second countdown, Activity and the status banner guide 20 rest windows,
40 movement windows, and 20 rest windows. Each window contains ten consecutive
BLE packets (about 100 ms); a complete capture stops automatically.

The session also contains `raw_data/<subject>/EMG` and `IMU` CSVs in the old
wide format. Complete captures add three files under
`labeled_data/<subject>/segmented_emg`: initial rest, movement, and final rest.
The first sensor channel uses the old filename layout; a second channel, when
present, ends in `_ch2`. Original packets and decoded stream CSVs are saved in
the same session. If packets or IMU windows are missing, the capture is marked
incomplete and no labeled segments are generated. Review captures before
using them for training.

Use **Open live (no recording)** for BLE/serial and **Open camera** for OAK-D.
The camera saves nothing until REC. Enter `--depth` in the GUI's optional
arguments to try stereo playback points; this mode needs a device retest after
DepthAI stream errors. `--pose-model 2` selects a heavier MediaPipe model.
See the [sensor data guide](../../docs/SENSOR_DATA.md) for interpretation.

The programs can also be started from the repository root:

```powershell
micromamba run -n aurora-simulation python src/recording/record_ble_sensors.py --device LIMBServer --subject S01 --duration 30
micromamba run -n aurora-simulation python src/recording/record_ble_sensors.py --device LIMBServer --subject S01 --dataset-label 1
micromamba run -n aurora-simulation python src/recording/record_serial_sensors.py --port COM5 --baud 115200 --subject S01 --duration 30
micromamba run -n aurora-simulation python src/recording/record_oak_pose.py --side left --subject S01 --duration 30
```

Use `--help` for other options. GUI environment variables are in the
[GUI guide](../gui/README.md). The BLE decoder preserves original packets;
serial recording preserves raw lines. Validate both against current firmware.
Cup detection is unavailable because the older project's model is not in this
repository; frame the full seated subject and mug when recording.

Participant recordings may contain identifiable or health-related data. Store
them according to the project's approved data plan and do not commit them to
Git; `outputs/` is ignored by default.
