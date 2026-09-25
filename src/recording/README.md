# Sensor preview and recording

The current recording pair is the OAK-D camera plus the shoulder/wrist IMU JSON
stream from one ESP32-C3.

| Source | Preview | Recorded files |
| --- | --- | --- |
| Dual IMU serial | Dedicated shoulder/wrist window | `serial.jsonl` |
| OAK-D | RGB video, body pose, and hand landmarks | `video.mp4`, `pose.json` |

Every recorder writes `meta.json`. In a batch, sources receive the same session
ID and write separate folders below `outputs/recordings/`.

## Preview

Use **Open IMU monitor** anywhere in the GUI for live ESP32 data. The monitor
does not save files. Open the camera through the advanced recorder preview; it
does not save until `R` or **START REC** is pressed.

## Record a session

1. Open **Recording**.
2. Keep OAK-D camera and Serial sensor stream selected.
3. Choose the ESP32 port and enter subject/trial metadata.
4. Press **Start selected sources**.
5. Press **Stop recording sources** when finished.

The sources share host-side session metadata but are not hardware-clock
synchronized.

`record_ble_sensors.py` remains as a compatibility entry point for the older
LIMB25 `LIMBServer` cuff. That old firmware carried EMG, IMU, and piezo streams;
those channels are not part of the current dual-IMU setup.

See [the sensor guide](../../docs/SENSOR_DATA.md) for wiring, flashing, and live
fusion.
