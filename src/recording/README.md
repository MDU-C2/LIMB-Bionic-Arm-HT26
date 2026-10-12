# Sensor preview and recording

The default recording workflow combines the OAK-D camera with BLE EMG and
dual-IMU data from one ESP32. EMG acquisition is 4,000 Hz and IMUs are read at
about 100 Hz. The current firmware streams sensor samples over BLE by default.

| Source | Preview | Recorded files |
| --- | --- | --- |
| Bluetooth dual IMU + EMG | Separate live previews | `imu.csv`, `emg.csv`, `packets.jsonl` |
| OAK-D | RGB video, body pose, and hand landmarks | `video.mp4`, `pose.json` |
| USB serial sensors (diagnostic) | Dedicated IMU/EMG window | `serial.jsonl` |
| ESP32 high-rate USB stream | Dedicated IMU/EMG window | `recording.csv`, `raw_serial.txt` |

Every recorder writes `meta.json`. In a batch, sources receive the same session
ID and write separate folders below `outputs/recordings/`.

## Preview

The GUI automatically opens BLE graphs for raw EMG and both IMUs. Use
**Connect BLE / show graphs** to reopen them. The header shows connection and
reception status; previews do not save files. See the
[recording guide](../../docs/RECORDING_GUIDE.md) for the full workflow. The camera
monitor also stays preview-only until `R` or **START REC** is pressed.

## Record a session

1. Open **Recording**.
2. Keep OAK-D camera and ESP32 sensors (Bluetooth) selected.
3. Enter subject/trial metadata and a duration, or use 0 to stop manually.
4. Keep both IMU and EMG selected and set the Bluetooth device name to `LIMBServer`.
5. Press **Start selected sources**.
6. Press **Stop recording sources** when finished.

Use `synced_host_monotonic_ns` in the BLE CSV files to align sensor samples
with camera frames, rather than the BLE notification arrival timestamp.
The firmware's time-sync characteristic maps ESP32 sample times to the same
host monotonic clock used by the camera.
Legacy LIMB25 `LIMBServer` devices still
record, but their synchronized field remains empty because they do not expose
the new time-sync characteristic. See
[Bluetooth synchronization](../../docs/BLUETOOTH_SYNC.md).

See [the sensor guide](../../docs/SENSOR_DATA.md) for wiring, flashing, and live
fusion.

The USB recorder is available for diagnostics when USB streaming is explicitly
enabled in the firmware. It is also available under **Advanced single recorder**.
It parses `EMG,timestamp,raw`,
`IMU1/IMU2,timestamp,ax,ay,az,gx,gy,gz`, and the older two-column EMG format.
The aligned semicolon CSV keeps unmatched samples and each IMU's own timestamp.
Use **Plot selected CSV** in the Recordings tab, or run
`python src/recording/plot_sensor_recording.py --file PATH --sensor all`.
The [live plot](../../docs/assets/signal_preview.png) shows EMG and all six axes
from each IMU on the connected board.

## Process recorded EMG

The offline tool adapts the filters and features in HT25's
[`emg_utils.py`](https://github.com/MDU-C2/LIMB-HT25/blob/cbbe903c66e6b0bf9944b1183f3d3aa2cb8270bc/src/layers/processing/emg_utils.py):
DC removal, a fourth-order 20–450 Hz Butterworth band-pass, a 50 Hz notch
with Q=30, and RMS/MAV over 200 ms windows with 50 ms overlap.
It uses complete windows so the last partial window does not lower the features
through zero padding. Values remain in ADC counts.

For a full-rate timestamped BLE recording:

```powershell
python src/recording/emg_processing.py --file outputs/recordings/SESSION/emg.csv
```

The source CSV stays unchanged. The tool writes `.filtered.csv`, `.features.csv`,
and `.processing.json` beside it. The JSON records the measured sample rate and
processing settings. Sample times must be increasing and regularly spaced;
recordings with missing packets need a continuous segment before filtering.
Legacy BLE packets without individual sample timestamps are not supported.

The BLE recording contains full-rate 4,000 Hz EMG, with a 2,000 Hz Nyquist limit,
so the original 20–450 Hz HT25 band works without changing the cutoff.
USB diagnostic output, when enabled, keeps every tenth sample (400 Hz);
that optional stream needs an explicitly selected `--highcut` below 200 Hz.
Filtering cannot recover frequencies lost during decimation.
Use `--lowcut`, `--notch`, `--window-ms`, and `--overlap-ms` to override the other
settings. This tool processes saved recordings; live previews keep their current
EMG activation estimator.

Select the processed CSV in the GUI and use **Plot selected CSV**, or run:

```powershell
python src/recording/plot_sensor_recording.py --file outputs/recordings/SESSION/emg.filtered.csv
python src/recording/plot_sensor_recording.py --file outputs/recordings/SESSION/emg.features.csv
```

Before the first dataset session, record a 30-second trial with the connected
ESP32 and camera. Check `meta.json` for both subscribed streams and a valid
time-sync result. Check `emg.csv` for 250 µs sample spacing and approximately
120,000 readings, and `imu.csv` for both sensor roles at roughly 100 Hz each.
Check `packet_health` for missing notifications and decode errors, and review
sequence and timestamp gaps before using a recording for training.
Keep labeled capture off for this trial so the requested duration is used.
A connected 30-second BLE trial passed with both IMUs, full-rate EMG and zero
detected packet gaps. Camera recording and live BLE simulation input also worked.
See [hardware validation](../../docs/HARDWARE_VALIDATION.md) for the saved
evidence and limits: muscle signal quality, person tracking and precise physical
camera/sensor alignment remain unverified.

## Recording code

The `record_*.py` files remain the command-line entry points. Shared work is
kept in modules that can also be used by previews and live simulation:

- `ble_packets.py`: packet formats, decoding, and optional CSV output.
- `oak_camera.py`: pipeline setup and camera defaults.
- `oak_geometry.py`: existing arm and finger angle calculations.
- `oak_tracking.py`: frame analysis and tracking overlays.
- `oak_session.py`: video, pose, and metadata files.
- `ble_preview_draw.py`: preview traces and orientation drawing.

The entry points still expose their earlier helper names for existing callers.
