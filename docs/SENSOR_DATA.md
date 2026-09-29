# Dual IMUs, camera, and recording

The current hardware path is two LSM6DSO32 IMUs on one ESP32-C3. The shoulder
and wrist readings leave the ESP32 as acquisition-timestamped ESP-IDF NimBLE
notifications. A newline-delimited USB JSON stream remains available for live
control and diagnostics. The OAK-D camera is a separate computer-side source.

## Wire the two IMUs

Both sensors share power, ground, SDA, and SCL. Their SA0/SDO pins select unique
I2C addresses:

| Role | Address | SA0/SDO | ESP32-C3-Zero |
| --- | --- | --- | --- |
| Shoulder | `0x6B` | 3.3 V / high | SDA GPIO 2, SCL GPIO 3 |
| Wrist | `0x6A` | GND / low | SDA GPIO 2, SCL GPIO 3 |

Do not put two sensors with the same address on the shared bus. The firmware
prefers the GPIO 2/3 harness on the current arm. For hardware diagnosis it can
also recognize the finite set of pin pairs found in earlier AURORA/LIMB
firmware, reports the active profile in serial JSON, and returns to 2/3 whenever
that requested pair responds.

## Build and flash

The source uses ESP-IDF drivers and NimBLE directly, with no Arduino framework.
Build and flash using a native ESP-IDF installation:

```powershell
cd firmware/dual_imu_serial
idf.py set-target esp32c3
idf.py build
idf.py -p COM4 flash monitor
```

The checked-in PlatformIO environment is also supported when `idf.py` is not
installed. It is pinned to `framework = espidf`, never Arduino:

```powershell
cd firmware/dual_imu_serial
platformio run
platformio run -t upload --upload-port COM4
```

The same Build, Flash, and Serial monitor actions are available under
**Firmware** in the GUI through native IDF or that ESP-IDF PlatformIO fallback.
The serial output uses schema
`aurora.dual_imu.v1`, 115200 baud, and explicit `shoulder`/`wrist` names.
The live monitor also recognizes the older line-oriented `IMU1 ACC` / `IMU1
GYRO` diagnostic stream still flashed on some lab boards, including the
`WHO_AM_I` result used to distinguish one connected IMU from two.

## View readings

Start the GUI and choose **Open IMU monitor** in the header, Recording page, or
Simulation page. The separate window shows each IMU's connection state,
address, temperature, acceleration, angular velocity, and relative arm-angle
estimate. **Calibrate current pose** makes the next valid sample the neutral
pose. The header explicitly reports `0/2`, `1/2`, or `2/2` connected sensors.

Choose **Open camera monitor** in the header, Recording page, or Simulation
page to see the OAK-D image, pose landmarks, arm angles, and hand tracking
without starting a recording.

The program accepts either one or two connected IMUs. The IMU strapped over
the brachialis/upper arm is the **shoulder** sensor (`0x6B`); it drives shoulder
elevation and left/right motion while the camera supplies the
elbow and axial-rotation channels. With both sensors, their relative mounted
tilt also drives the elbow. A wrist-only sensor remains visible in the monitor,
but does not impersonate an upper-arm sensor. The legacy single-IMU JSON packet
is treated as the upper-arm sensor, matching HT25's original placement.

## Live camera/IMU simulation control

Open **Simulation**, select the ESP32 port, then choose **Start live interactive
control**. It opens the full table-and-target simulator, annotated camera view,
and a separate live sensor monitor. The live controller:

1. detects whether zero, one, or two IMUs are reporting valid samples;
2. reproduces HT25's installed-sensor mapping: sample-rate-independent smoothed
   `atan2(Y, Z)` tilt and scaled, dead-zoned, signed gyro Z for shoulder
   left/right;
3. computes elbow flexion from wrist tilt relative to shoulder tilt when both
   sensors are present;
4. corrects the bounded targets with trunk-relative OAK-D/MediaPipe angles and
   maps tracked finger flexion to the simulated grip; and
5. applies the existing LIMB joint and speed limits to PyBullet.

Camera correction `0` means IMU-only control and `1` means camera-only control;
`0.25` keeps the IMUs responsive while the camera supplies absolute-pose
correction. Live feedback initially stays paused with the simulated arm fully
extended. Copy that pose with the real arm, then press `L` to calibrate and
start feedback. Press `R` to pause, reset the scene, and show the reference pose
again. `K` recalibrates at the current pose. Press `Q` in the camera window to
stop.

## Record synchronized sources

The Recording page selects **OAK-D camera** and **Dual-IMU ESP32 (Bluetooth)**
by default. Both processes share a session ID and write separate folders below
`outputs/recordings/`. The ESP32 acquisition clock is mapped continuously onto
the host monotonic clock also saved with every camera frame, so the time series
can be aligned without treating BLE notification arrival as sample time.

The recorder remains compatible with the older LIMB25 `LIMBServer` EMG, IMU,
and piezo characteristics. Old firmware has no time-sync characteristic, so its
rows retain arrival timestamps and leave the synchronized timestamp empty. See
[Bluetooth synchronization](BLUETOOTH_SYNC.md) for the protocol and fields.
