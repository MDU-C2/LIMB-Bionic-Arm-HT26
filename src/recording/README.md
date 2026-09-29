# Sensor preview and recording

The current recording workflow combines the OAK-D camera with selected
acquisition-timestamped IMU, EMG, and piezo notifications from an ESP32.

| Source | Preview | Recorded files |
| --- | --- | --- |
| Bluetooth IMU / EMG / piezo | Separate BLE previews | Selected sensor CSVs, `packets.jsonl` |
| OAK-D | RGB video, body pose, and hand landmarks | `video.mp4`, `pose.json` |
| Dual IMU serial (diagnostic) | Dedicated shoulder/wrist window | `serial.jsonl` |

Every recorder writes `meta.json`. In a batch, sources receive the same session
ID and write separate folders below `outputs/recordings/`.

## Preview

Use **Open IMU monitor** anywhere in the GUI for live ESP32 data. The monitor
does not save files. Open the camera through the advanced recorder preview; it
does not save until `R` or **START REC** is pressed.

## Record a session

1. Open **Recording**.
2. Keep OAK-D camera and ESP32 sensors (Bluetooth) selected.
3. Choose IMU, EMG, and/or piezo. Missing characteristics do not stop available streams.
4. Keep `LIMBServer` (or enter the configured BLE name) and enter subject/trial metadata.
5. Press **Start selected sources**.
6. Press **Stop recording sources** when finished.

The ESP32 acquisition clock is continuously mapped to the same host monotonic
clock saved with every camera frame. Legacy LIMB25 `LIMBServer` devices still
record, but their synchronized field remains empty because they do not expose
the new time-sync characteristic. See
[Bluetooth synchronization](../../docs/BLUETOOTH_SYNC.md).

See [the sensor guide](../../docs/SENSOR_DATA.md) for wiring, flashing, and live
fusion.
