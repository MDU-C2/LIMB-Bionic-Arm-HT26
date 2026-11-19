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
| Forearm (`wrist` packet role) | `0x6A` | GND / low | SDA GPIO 2, SCL GPIO 1 |

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
**Advanced Tools → Firmware** in the GUI through native IDF or that ESP-IDF PlatformIO fallback.
The serial output uses 115200 baud with `EMG`, `IMU1` (shoulder), and `IMU2`
(forearm) rows. When USB transport is enabled, EMG prints every tenth 4 kHz
ADC sample. The default BLE transport sends every acquired EMG sample.
The live monitor also recognizes the older line-oriented `IMU1 ACC` / `IMU1
GYRO` diagnostic stream still flashed on some lab boards, including the
`WHO_AM_I` result used to distinguish one connected IMU from two.
That text adapter retains historical scale assumptions of 1/4096 g and 1/65.5
degrees/s per count. The text does not identify full-scale register settings;
absolute units cannot be verified without the flashed source/configuration.
Use the current tagged USB or BLE format for measurements with explicit units.

For a terminal-level hardware check that requires valid vectors from both
sensors, run:

```powershell
python scripts/check_dual_imu.py --port COM5 --require-emg
```

USB diagnostic recording saves received bytes and line receipt times in
`serial.jsonl`. Its derived `recording.csv` has one row per measurement, with
separate device sample times and actual host receipt times. It retains duplicates
and clock resets and reports them in `meta.json`. Unsequenced USB lines cannot
establish a transport-loss count. A disconnect keeps the raw file and marks the
capture incomplete. CSV export runs after the connection closes and streams the
file without holding the capture in memory.

## View readings

**Connect BLE / show graphs** in the header opens raw EMG and both IMUs'
acceleration/gyro graphs. The header reports connected IMUs and EMG reception. Live activation uses
a relaxed-muscle baseline; live simulation also has IMU neutral-pose calibration.
The **Connect / inspect USB** action under **Connect & inspect** is available for USB diagnostic firmware.
See the [recording guide](recording-guide.md) for the complete BLE workflow.

Choose **Open camera monitor** in the header, Connect & inspect page, or Control
page to see the OAK-D image, pose landmarks, arm angles, and hand tracking
without starting a recording.

The decoder and monitors accept either one or two connected IMUs. Guided live
setup and cup recording require both. The IMU strapped over
the brachialis/upper arm is the **shoulder** sensor (`0x6B`); it drives shoulder
elevation and left/right motion while the camera supplies the
elbow and axial-rotation channels. With both sensors, their relative mounted
tilt also drives the elbow. A wrist-only sensor remains visible in the monitor,
but does not impersonate an upper-arm sensor. The legacy single-IMU JSON packet
is treated as the upper-arm sensor, matching HT25's original placement.

## Live camera/IMU/EMG simulation control

Open **Control → Connect + open live setup**. Follow **Mounting movements →
Relaxed EMG → Firm grip → Start interactive control**. Stereo depth is enabled
by default. Return the open hand to marker 1, thumb up and elbow about 90 degrees
after the isolated movements. Without depth, setup uses configured-axis diagnostics.
See [Live control](live-control.md) for mounting, theory and offline replay.

The camera corrects elevation, elbow and lateral motion as well as the observable
upper-arm rotation proxy. IMU predictions continue between camera frames.
Measured angles remain intact through calibration and fusion; robot limits apply
when mapping to joint commands. The mounted tilt/twist model remains approximate.
The forearm's axial drift has no camera correction.

The app defaults to EMG grip. Camera and Auto are also available. EMG supplies a
shared grip command, not individual finger angles or measured grip force. Live
preview follows targets within joint limits; physics mode also models motor speed,
gravity and contact. Reconnecting Bluetooth requires guided calibration again.

## Record synchronized sources

Cup transfers keep BLE and camera files together under each subject and trial.
The advanced recorder selects **OAK-D camera** and **ESP32 sensors (Bluetooth)**
by default; those processes share a session ID and write separate source folders
below `outputs/recordings/`. BLE recording uses an active time-sync exchange to map
ESP32 sample times onto the camera's host monotonic clock. Use the BLE CSV's
`synced_host_monotonic_ns` field for alignment. The firmware defaults to BLE
sensor transport, with 4 kHz EMG and approximately 100 Hz IMU readings.

The recorder remains compatible with older LIMB-HT25 `LIMBServer` packet layouts,
including its optional piezo characteristic. Piezo is hidden from the current
GUI because it is not part of this hardware. Old firmware has no time sync, so
its rows retain arrival timestamps and leave the synchronized timestamp empty. See
[Bluetooth synchronization](bluetooth-sync.md) for the protocol and fields.

Normal cup capture also saves unmirrored RGB video and lossless RGB-aligned
stereo depth maps. Subject calibration fits mounting rotations and saves a
separate aligned IMU copy with each trial. Camera intrinsics and OAK device timestamps are
saved with them. The OAK and ESP32 device clocks differ; use host-aligned BLE
times and camera receipt times to compare sources. The
[recording guide](recording-guide.md) describes the files and limitations.
