# Dual IMUs, camera, and recording

The current hardware path is two LSM6DSO32 IMUs on one ESP32-C3. The shoulder
and wrist readings leave the ESP32 as one newline-delimited JSON serial stream.
The OAK-D camera is a separate computer-side source.

## Wire the two IMUs

Both sensors share power, ground, SDA, and SCL. Their SA0/SDO pins select unique
I2C addresses:

| Role | Address | SA0/SDO | ESP32-C3-Zero |
| --- | --- | --- | --- |
| Shoulder | `0x6A` | GND / low | SDA GPIO 8, SCL GPIO 5 |
| Wrist | `0x6B` | 3.3 V / high | SDA GPIO 8, SCL GPIO 5 |

Do not put two sensors with the same address on the shared bus. The firmware
uses the GPIO 8/5 harness verified in the latest project change. If the physical
harness changes, update `I2C_SDA_PIN` and `I2C_SCL_PIN` in
`firmware/dual_imu_serial/main/main.c` deliberately rather than scanning random
pin pairs at runtime.

## Build and flash with ESP-IDF

LIMB-HT25 used ESP-IDF and `idf.py`; this repository follows the same layout.
From an ESP-IDF terminal:

```powershell
cd firmware/dual_imu_serial
idf.py set-target esp32c3
idf.py build
idf.py -p COM5 flash monitor
```

The same Build, Flash, and Serial monitor actions are available under
**Firmware** in the GUI. The serial output uses schema
`aurora.dual_imu.v1`, 115200 baud, and explicit `shoulder`/`wrist` names.

## View readings

Start the GUI and choose **Open IMU monitor** in the header, Recording page, or
Simulation page. The separate window shows each IMU's connection state,
address, temperature, acceleration, angular velocity, and relative arm-angle
estimate. **Calibrate current pose** makes the next complete two-sensor sample
the neutral pose.

Choose **Open camera monitor** in the header, Recording page, or Simulation
page to see the OAK-D image, pose landmarks, arm angles, and hand tracking
without starting a recording.

The old single-IMU JSON packet remains readable during hardware transition,
but shoulder-relative elbow control requires both IMUs.

## Live camera/IMU simulation control

Open **Simulation**, select the ESP32 port, then choose **Start live interactive
control**. It opens the full table-and-target simulator, annotated camera view,
and a separate live sensor monitor. The live controller:

1. estimates shoulder and wrist orientation with the LIMB-HT25 complementary
   accel/gyro filter;
2. computes elbow flexion from the wrist angle relative to the shoulder;
3. corrects drift with the current OAK-D/MediaPipe arm angles; and
4. applies the existing LIMB joint and speed limits to PyBullet.

Camera correction `0` means IMU-only control and `1` means camera-only control;
`0.25` keeps the IMUs responsive while the camera supplies low-frequency
correction. Press `C` in the camera window to recalibrate the IMUs and `Q` to
stop.

## Record synchronized sources

The Recording page selects **OAK-D camera** and **Serial sensor stream** by
default. Both processes share a session ID and write separate folders below
`outputs/recordings/`. Their host timestamps are recorded, but they are not
hardware-clock synchronized.

The older LIMB25 `LIMBServer` BLE cuff recorder remains under
`src/recording/record_ble_sensors.py` for compatibility. LIMB25 did include a
piezo channel in that cuff; it is not part of the current two-IMU ESP32 workflow
and is therefore not shown in the primary GUI.
