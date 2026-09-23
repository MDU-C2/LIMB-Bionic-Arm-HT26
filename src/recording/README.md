# Sensor previews and recording

The GUI can preview and record these sources:

| Source | Preview | Recorded files |
| --- | --- | --- |
| EMG | Raw signal and recent activity | `emg.csv` |
| IMU | Acceleration, gyroscope, and arm tilt | `imu.csv` |
| Piezo | Contact and vibration signal | `piezo.csv` |
| OAK-D | RGB video, body pose, and hand landmarks | `video.mp4`, `pose.json` |
| Serial | Lines received from a selected port | `serial.jsonl` |

BLE recordings also keep `packets.jsonl`. Every recorder writes `meta.json`.

## Preview the sensors

Open the **Sensors** tab. EMG, IMU, piezo, camera, and serial each have their own
button and window. Several windows can be open at the same time.

The three cuff windows share one connection to `LIMBServer`. Closing one BLE
window does not close the other two. Closing the last BLE window ends the BLE
connection.

The IMU window shows pitch and roll from gravity. Use **Zero tilt** while the
cuff is still. Twisting the arm appears in the gyroscope graph, but it cannot
give a stable absolute yaw angle without a magnetometer.

A piezoelectric sensor creates an electrical signal when it is bent, tapped, or
vibrated. The piezo preview is useful for seeing contact and changes. It is not
a calibrated force measurement.

## Record a session

1. Close the preview windows.
2. Open **Recording**.
3. Select BLE, OAK-D, serial, or a combination.
4. Enter the subject, trial, test type, and duration.
5. Press **Start selected sources**.
6. Press **Stop recording sources** when the trial is finished.

BLE records EMG, IMU, and piezo together. The selected sources receive the same
session ID and write separate folders below `outputs/recordings/`. Their clocks
are not hardware-synchronized.

The OAK-D records automatically when it is part of a batch. In a camera-only
preview, press `R` or **START REC** to begin recording and `Q` to close it.

See [the sensor guide](../../docs/SENSOR_DATA.md) for the current ESP32 and IMU
test setup.
