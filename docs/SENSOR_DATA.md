# Dual IMUs, EMG, camera, and recording

The current hardware path is two LSM6DSO32 IMUs plus one EMG analogue channel
on one ESP32-C3. The firmware timestamps each EMG and IMU reading on the same
ESP32 clock and sends them over ESP-IDF NimBLE. Tagged USB lines remain
available for diagnostics when USB transport is enabled. The OAK-D is a
computer-side source.

## Wire the two IMUs

Both sensors share power, ground, SDA, and SCL. Their SA0/SDO pins select unique
I2C addresses:

| Role | Address | SA0/SDO | ESP32-C3-Zero |
| --- | --- | --- | --- |
| Shoulder | `0x6B` | 3.3 V / high | SDA GPIO 2, SCL GPIO 1 |
| Wrist | `0x6A` | GND / low | SDA GPIO 2, SCL GPIO 1 |

Connect the EMG module analogue output to **GPIO0 / ADC1 channel 0** and share
3.3 V and ground. GPIO0 is ADC1 channel 0 on the ESP32-C3.

Do not put two sensors with the same address on the shared bus. The firmware
prefers the GPIO 2/1 harness on the current arm. Its only fallback pairs are the
source-backed LIMB-HT25 bus (SDA 4/SCL 5) and earlier AURORA prototype bus
(SDA 8/SCL 5). It reports the selected profile in the serial log and retries
disconnected IMUs while acquisition continues.

## Build and flash

The source uses ESP-IDF drivers and NimBLE directly, with no Arduino framework.
Build and flash using a native ESP-IDF installation:

```powershell
cd firmware/high_rate_emg_imu
idf.py set-target esp32c3
idf.py build
idf.py -p COM4 flash monitor
```

The PlatformIO environment also builds with ESP-IDF:

```powershell
cd firmware/high_rate_emg_imu
platformio run
platformio run -t upload --upload-port COM4
```

The same Build, Flash, and Serial monitor actions are available under
**Firmware** in the GUI through native IDF or that ESP-IDF PlatformIO fallback.
The serial output uses 115200 baud with `EMG`, `IMU1` (shoulder), and `IMU2`
(wrist) rows. When USB transport is enabled, EMG prints every tenth 4 kHz
ADC sample. The default BLE transport sends every acquired EMG sample.
The live monitor also recognizes the older line-oriented `IMU1 ACC` / `IMU1
GYRO` diagnostic stream still flashed on some lab boards, including the
`WHO_AM_I` result used to distinguish one connected IMU from two.

For a terminal-level hardware check that requires valid vectors from both
sensors, run:

```powershell
micromamba run -n aurora-simulation python scripts/check_dual_imu.py --port COM5 --require-emg
```

## View readings

The GUI automatically connects over BLE and opens raw EMG and both IMUs'
acceleration/gyro graphs. **Connect BLE / show graphs** in the header reopens
them. The header reports connected IMUs and EMG reception. Live activation uses
a relaxed-muscle baseline; live simulation also has IMU neutral-pose calibration.
The **USB monitor** on Recording is available for USB diagnostic firmware.
See the [recording guide](RECORDING_GUIDE.md) for the complete BLE workflow.

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

Open **Simulation**, check the ESP32 **BLE device name** (`LIMBServer` by default),
close the standalone camera preview, then choose **Start live interactive
control**. It opens the full table-and-target simulator, annotated camera view,
and a separate live sensor monitor. The GUI releases its BLE preview before
control connects. The live controller:

1. detects whether zero, one, or two IMUs are reporting valid samples;
2. reproduces HT25's installed-sensor mapping: sample-rate-independent smoothed
   `atan2(Y, Z)` tilt and scaled, dead-zoned, signed gyro Z for shoulder
   left/right;
3. computes elbow flexion from wrist tilt relative to shoulder tilt when both
   sensors are present, and adds calibrated upper-arm/relative forearm gyro twist;
4. corrects the bounded targets with trunk-relative OAK-D/MediaPipe angles and
   uses tracked hand curl for the default Camera grip source;
5. filters high-rate raw EMG in the 20–450 Hz band with a 50 Hz notch, computes
   an RMS envelope, and maps rest/strong-grip calibrated activation to grip when
   EMG is selected (low-rate legacy packets use envelope processing); and
6. applies the existing LIMB joint and speed limits to PyBullet.

Grip source can be Camera, EMG, or Auto. Auto prefers a current EMG activation
even with unattached electrodes; it falls back to camera curl only when EMG is
unavailable. This setting does not affect which streams are recorded.

Camera correction weights channels available from both sources. `0` prefers
IMUs and `1` prefers camera; single-source channels remain available. At `0.25`,
IMUs contribute to elevation and elbow movement while camera supplies absolute
abduction and corrects the upper-arm gyro twist state. Live feedback initially stays paused with the simulated arm in its neutral
tabletop pose (shoulder and elbow at 45 degrees, wrist neutral and hand open).
Copy that pose with the real arm, then press `L` to calibrate and
start feedback. Press `R` to pause, reset the scene, and show the reference pose
again. `K` recalibrates at the current pose. Press `Q` in the camera window to
stop.

Camera axial correction requires a visible bent elbow; calibrated IMU twist
continues when that observation is unavailable. Forearm twist uses both sensors
with common-motion compensation; its drift has no camera correction in the old
mapping. Q/E and Left/Right provide fallback when live twist targets are absent.
Mount the default sensor Y axes along the limbs and hold still for 0.5 seconds
after L; see [mounting and isolated-motion checks](SIMULATION.md#mounting-and-twist-calibration)
before trusting anatomical movement. Axis selections affect live control only.
Fingers share a grip
command, with live movement limited by the HT25 finger-servo speeds. See
[Joint limits](JOINT_LIMITS.md) for the source and model limitations.

Live simulation uses the same BLE decoder as recording without creating files.
Every EMG sample contributes to the grip envelope, and IMU updates use device
sample timing when available. Bluetooth reconnects in the background; each
stream expires independently after 0.5 seconds without fresh data. Hardware
operation still needs checking with the ESP32 and camera connected.

## Record synchronized sources

The Recording page selects **OAK-D camera** and **ESP32 sensors (Bluetooth)** by default.
Both processes share a session ID and write separate folders below
`outputs/recordings/`. BLE recording uses an active time-sync exchange to map
ESP32 sample times onto the camera's host monotonic clock. Use the BLE CSV's
`synced_host_monotonic_ns` field for alignment. The firmware defaults to BLE
sensor transport, with 4 kHz EMG and approximately 100 Hz IMU readings.

The recorder remains compatible with older LIMB25 `LIMBServer` packet layouts,
including its optional piezo characteristic. Piezo is hidden from the current
GUI because it is not part of this hardware. Old firmware has no time sync, so
its rows retain arrival timestamps and leave the synchronized timestamp empty. See
[Bluetooth synchronization](BLUETOOTH_SYNC.md) for the protocol and fields.
