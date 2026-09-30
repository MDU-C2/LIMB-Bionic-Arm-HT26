# Dual IMUs, EMG, camera, and recording

The current hardware path is two LSM6DSO32 IMUs plus one EMG analogue channel
on one ESP32-C3. All three readings leave the ESP32 with the same acquisition
sequence and timestamp over ESP-IDF NimBLE. Newline-delimited USB JSON remains
available for live control and diagnostics. The OAK-D is a computer-side source.

## Wire the two IMUs

Both sensors share power, ground, SDA, and SCL. Their SA0/SDO pins select unique
I2C addresses:

| Role | Address | SA0/SDO | ESP32-C3-Zero |
| --- | --- | --- | --- |
| Shoulder | `0x6B` | 3.3 V / high | SDA GPIO 2, SCL GPIO 1 |
| Wrist | `0x6A` | GND / low | SDA GPIO 2, SCL GPIO 1 |

Connect the EMG module analogue output to **GPIO0 / ADC1 channel 0** and share
3.3 V and ground. This is the channel selected by Jonas's tested prototype;
its old GPIO4 comment did not match the ESP32-C3 ADC pin map.

Do not put two sensors with the same address on the shared bus. The firmware
prefers the GPIO 2/1 harness on the current arm. Its only fallback pairs are the
source-backed LIMB-HT25 bus (SDA 4/SCL 5) and earlier AURORA prototype bus
(SDA 8/SCL 5). It reports the active profile in serial JSON and returns to 2/1
whenever that requested pair responds.

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
The serial output uses schema `aurora.sensors.v1`, 115200 baud, an `emg` object,
and explicit `shoulder`/`wrist` names.
The live monitor also recognizes the older line-oriented `IMU1 ACC` / `IMU1
GYRO` diagnostic stream still flashed on some lab boards, including the
`WHO_AM_I` result used to distinguish one connected IMU from two.

For a terminal-level hardware check that requires valid vectors from both
sensors, run:

```powershell
micromamba run -n aurora-simulation python scripts/check_dual_imu.py --port COM5 --require-emg
```

## View readings

Start the GUI and choose **Open sensor monitor** in the header, Recording page,
or Simulation page. The separate window shows both IMUs, raw EMG, calibrated
EMG activation, and the relative arm estimate. **Calibrate current pose** resets
both the IMU neutral pose and relaxed-muscle baseline. The header explicitly
reports `0/2`, `1/2`, or `2/2` connected IMUs.

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

## Live camera/IMU/EMG simulation control

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
   uses tracked hand curl as a grip fallback when EMG is unavailable;
5. rectifies and smooths EMG against a measured relaxed-muscle baseline and
   maps activation to the simulated grip; and
6. applies the existing LIMB joint and speed limits to PyBullet.

Camera correction `0` means IMU-only control and `1` means camera-only control;
`0.25` keeps the IMUs responsive while the camera supplies absolute-pose
correction. Live feedback initially stays paused with the simulated arm fully
extended. Copy that pose with the real arm, then press `L` to calibrate and
start feedback. Press `R` to pause, reset the scene, and show the reference pose
again. `K` recalibrates at the current pose. Press `Q` in the camera window to
stop.

## Record synchronized sources

The Recording page selects **OAK-D camera**, **IMU**, and **EMG** by default.
Both processes share a session ID and write separate folders below
`outputs/recordings/`. The ESP32 acquisition clock is mapped continuously onto
the host monotonic clock also saved with every camera frame, so the time series
can be aligned without treating BLE notification arrival as sample time.

The recorder remains compatible with older LIMB25 `LIMBServer` packet layouts,
including its optional piezo characteristic. Piezo is hidden from the current
GUI because it is not part of this hardware. Old firmware has no time sync, so
its rows retain arrival timestamps and leave the synchronized timestamp empty. See
[Bluetooth synchronization](BLUETOOTH_SYNC.md) for the protocol and fields.
