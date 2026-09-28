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

## Build and flash

The source keeps the ESP-IDF layout and register configuration used by
LIMB-HT25. The GUI invokes PlatformIO from `aurora-simulation`, so a separate
global `idf.py` installation is not required. Select the detected COM port in
the Firmware tab, then use Build and Flash. The equivalent commands are:

```powershell
cd firmware/dual_imu_serial
python -m platformio run
python -m platformio run --target upload --upload-port COM4
python -m platformio device monitor --port COM4 --baud 115200
```

The same Build, Flash, and Serial monitor actions are available under
**Firmware** in the GUI. The serial output uses schema
`aurora.dual_imu.v1`, 115200 baud, and explicit `shoulder`/`wrist` names.

## View readings

Start the GUI and choose **Open IMU monitor** in the header, Recording page, or
Simulation page. The separate window shows each IMU's connection state,
address, temperature, acceleration, angular velocity, and relative arm-angle
estimate. **Calibrate current pose** makes the next valid sample the neutral
pose. The header explicitly reports `0/2`, `1/2`, or `2/2` connected sensors.

Choose **Open camera monitor** in the header, Recording page, or Simulation
page to see the OAK-D image, pose landmarks, arm angles, and hand tracking
without starting a recording.

The program accepts either one or two connected IMUs. A lone shoulder sensor
drives shoulder elevation and left/right motion while the camera supplies the
elbow and axial-rotation channels. With both sensors, their relative mounted
tilt also drives the elbow. A wrist-only sensor remains visible in the monitor,
but does not impersonate an upper-arm sensor. The legacy single-IMU JSON packet
is treated as the upper-arm sensor, matching HT25's original placement.

## Live camera/IMU simulation control

Open **Simulation**, select the ESP32 port, then choose **Start live interactive
control**. It opens the full table-and-target simulator, annotated camera view,
and a separate live sensor monitor. The live controller:

1. detects whether zero, one, or two IMUs are reporting valid samples;
2. reproduces HT25's installed-sensor mapping: smoothed `atan2(Y, Z)` tilt and
   dead-zoned gyro Z for shoulder left/right;
3. computes elbow flexion from wrist tilt relative to shoulder tilt when both
   sensors are present;
4. corrects the bounded targets with trunk-relative OAK-D/MediaPipe angles and
   maps tracked finger flexion to the simulated grip; and
5. applies the existing LIMB joint and speed limits to PyBullet.

Camera correction `0` means IMU-only control and `1` means camera-only control;
`0.25` keeps the IMUs responsive while the camera supplies absolute-pose
correction. Hold the arm down and still, then press `K` in the simulator or `C`
in the camera window to recalibrate at anatomical zero. Press `Q` in the camera
window to stop.

## Record synchronized sources

The Recording page selects **OAK-D camera** and **Serial sensor stream** by
default. Both processes share a session ID and write separate folders below
`outputs/recordings/`. Their host timestamps are recorded, but they are not
hardware-clock synchronized.

The older LIMB25 `LIMBServer` BLE cuff recorder remains under
`src/recording/record_ble_sensors.py` for compatibility. LIMB25 did include a
piezo channel in that cuff; it is not part of the current two-IMU ESP32 workflow
and is therefore not shown in the primary GUI.
