# Sensor recording

The control center discovers all three recorders and starts them from the
**Recording** tab. Choose a source in **Sensors**, then set the subject,
duration, device, and output folder in **Recording**.

| Program | Input | Session files |
| --- | --- | --- |
| `record_ble_sensors.py` | LIMB BLE notifications | `packets.jsonl`, `emg.csv`, `imu.csv`, `piezo.csv`, `meta.json` |
| `record_serial_sensors.py` | Newline-based serial messages | `serial.jsonl`, `meta.json` |
| `record_oak_pose.py` | OAK-D video, arm pose, and hand landmarks | `video.mp4`, `pose.json`, `meta.json` |

Each run creates a timestamped session folder. A duration of `0` records until
**Stop active program** is pressed, except labeled BLE captures stop after 80
windows. OAK-D preview appears in a separate window. Its `pose.json` can be
opened in the Motion AI tab after recording.

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

The programs can also be started from the repository root:

```powershell
micromamba run -n aurora-simulation python src/recording/record_ble_sensors.py --device LIMBServer --subject S01 --duration 30
micromamba run -n aurora-simulation python src/recording/record_ble_sensors.py --device LIMBServer --subject S01 --dataset-label 1
micromamba run -n aurora-simulation python src/recording/record_serial_sensors.py --port COM5 --baud 115200 --subject S01 --duration 30
micromamba run -n aurora-simulation python src/recording/record_oak_pose.py --side right --subject S01 --duration 30
```

Run a program with `--help` for all options. GUI settings are passed through
the `AURORA_*` environment variables described in the
[GUI guide](../gui/README.md).

The BLE decoder supports the packet layouts found in the LIMB-HT25 firmware
and preserves every original packet as base64. Serial recording keeps the raw
line and adds parsed JSON when valid. These paths still need testing against
the exact 2026 sensor firmware and devices. OAK-D capture records pose for
later playback; it does not currently control the simulator live.

Participant recordings may contain identifiable or health-related data. Store
them according to the project's approved data plan and do not commit them to
Git; `outputs/` is ignored by default.
