# Dual-IMU and EMG ESP32-C3 firmware

This is the maintained ESP-IDF-only target. It combines the working dual-IMU
firmware with Jonas's verified EMG ADC acquisition. Bluetooth uses ESP-IDF
NimBLE directly; there is no Arduino framework or third-party BLE library.

Both IMUs share one I2C bus and must use different addresses:

| Role | LSM6DSO32 address | SA0/SDO |
| --- | --- | --- |
| Shoulder | `0x6B` | High / 3.3 V |
| Wrist | `0x6A` | Low / GND |

The sensor currently strapped over the brachialis is the shoulder-role sensor,
so it must use address `0x6B`. The wrist sensor may be absent; firmware and GUI
continue in single-IMU mode.

Connect the EMG module's analogue output to **GPIO0**, which ESP-IDF identifies
as ADC1 channel 0 on ESP32-C3. Jonas's prototype selected `ADC_CHANNEL_0`; its
old `// GPIO4` comment was inconsistent with the chip pin map. The combined
firmware preserves the proven channel selection and reports both channel and
GPIO in serial JSON.

Connect both SDA pins to ESP32-C3 GPIO 2, both SCL pins to GPIO 1, and share
3.3 V and ground. Do not connect two sensors with the same address to this bus.
At startup the firmware checks those requested pins first. If neither address
answers, it also checks the verified LIMB-HT25 wiring on SDA GPIO 4/SCL GPIO 5,
followed by the older AURORA prototype wiring on SDA GPIO 8/SCL GPIO 5. It
continues on whichever pair responds. The bus runs at the previous repo's
reliable 100 kHz.
The JSON `i2c` object reports the active pins/profile and sets `fallback` to
`true` when an old pair is in use. With no sensor connected it cycles these
three source-backed pairs once per second, so hot-plugging is detected without
a reboot.

The `external_sda_pullup` and `external_scl_pullup` fields test whether the
powered breakout's pull-ups physically reach each ESP32 pin. Both should be
`true` on the correct pair. One `false` value indicates an open or miswired
signal conductor even though the ESP32's internal pull-up can still make the
reported idle line level appear high. Sensor errors distinguish a missing I2C
acknowledgement from an unexpected `WHO_AM_I` value.

Use the native ESP-IDF toolchain to build, flash, and monitor the target:

```powershell
cd firmware/dual_imu_serial
idf.py set-target esp32c3
idf.py build
idf.py -p COM4 flash monitor
```

If `idf.py` is not installed, the checked-in PlatformIO environment is an
equivalent fallback and is pinned to `framework = espidf` (ESP-IDF 4.4.7):

```powershell
platformio run
platformio run -t upload --upload-port COM4
platformio device monitor --port COM4
```

PlatformIO is only the build runner here; Arduino is not installed or linked.

The GUI's **Firmware** page prefers `idf.py` and automatically uses this
ESP-IDF-only PlatformIO environment when the native command is unavailable.
Because ESP-IDF 4.4 rejects project paths containing spaces, the GUI builds an
exact content-addressed mirror under the system temporary directory when the
repository is inside a spaced OneDrive path. The source repository is not moved
or modified by that workaround.

The device emits one `aurora.sensors.v1` JSON line every 20 ms at 115200 baud.
Each line contains the GPIO0 EMG ADC value plus role-named shoulder and wrist
IMUs. A missing I2C address is emitted as disconnected, so the desktop program
automatically changes between `1/2` and `2/2` IMU mode while EMG continues.

To verify both sensors without opening the GUI, run this from the repository
root (replace `COM5` when necessary):

```powershell
micromamba run -n aurora-simulation python scripts/check_dual_imu.py --port COM5 --require-emg
```

The command succeeds only after it receives complete vectors from both physical
IMU addresses and valid EMG ADC samples.

At the sensor rate (100 Hz), the device also advertises as `LIMBServer` and
notifies the existing LIMB EMG and IMU characteristics. The 38-byte IMU packet
on `25011525-1212-efde-1523-785feabcd122` is:

| Field | Type | Meaning |
| --- | --- | --- |
| format/flags | `uint16` | high byte `1`; bits 0/1 mean shoulder/wrist connected |
| sequence | `uint32` | increments once per acquisition |
| device time | `uint64` | ESP monotonic microseconds at acquisition |
| samples | `12 × int16` | shoulder then wrist; accel g and gyro dps, each ×1000 |

The 16-byte EMG packet on `24011525-1212-efde-1523-785feabcd122` uses the same
sequence and device timestamp:

| Field | Type | Meaning |
| --- | --- | --- |
| format/flags | `uint16` | high byte `1`; bit 0 means the ADC sample is valid |
| sequence | `uint32` | same acquisition sequence as the IMU packet |
| device time | `uint64` | same ESP monotonic acquisition time |
| sample | `uint16` | raw 12-bit EMG ADC value |

The time-sync characteristic `27011525-1212-efde-1523-785feabcd122` accepts a
host transmit timestamp and returns device receive/transmit timestamps. The
recorder repeats this four-timestamp exchange to estimate offset and drift.
See [Bluetooth synchronization](../../docs/BLUETOOTH_SYNC.md).
