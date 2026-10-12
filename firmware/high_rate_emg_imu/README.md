# ESP32-C3 EMG and IMU firmware

This is the single maintained firmware target. It combines the 4 kHz
continuous ADC and LSM6DSO32 implementation from the Jonas-AURORA snapshot
with the existing AURORA single-IMU fallback and BLE time-sync service.
The IMU driver is credited to Cornelia, with Jonas's adaptation, in its source
headers.

The default configuration in [`include/config.h`](include/config.h) acquires
EMG, IMU1 (`0x6B`) and IMU2 (`0x6A`). It sends all acquired EMG samples over
BLE in blocks and reads available IMUs at about 100 Hz. Sensor output defaults
to BLE only. If USB output is enabled for diagnostics, it prints every tenth
EMG sample (400 Hz). All samples use the ESP32 monotonic clock;
the BLE time-sync characteristic maps that clock onto the host clock used by
the camera recorder. USB IMU values are mg and mdps. The BLE recorder converts
them to g and dps. The device advertises as `LIMBServer`.

Select `SENSOR_MODE_EMG`, `SENSOR_MODE_IMU`, or `SENSOR_MODE_EMG_IMU`, and
`TRANSPORT_USB`, `TRANSPORT_BLE`, or `TRANSPORT_BOTH` in `config.h`. The default
is both sensor types and BLE transport. BLE packets contain a mask for whichever
IMUs are connected, so one IMU can be used alone. An absent IMU is probed
again during acquisition. The firmware tries SDA/SCL `2/1` first, then `4/5`
and `8/5`. EMG uses GPIO0 / ADC1 channel 0. Both IMUs share the bus and must
have distinct SA0/SDO addresses.

## Build and flash

Use the GUI **Firmware** tab or run from this directory:

```powershell
platformio run
platformio run -t upload --upload-port COM5
platformio device monitor --port COM5
```

The checked-in PlatformIO environment uses ESP-IDF 5.4 through
`espressif32@6.10.0`. An activated native ESP-IDF 5.x installation also works:

```powershell
idf.py set-target esp32c3
idf.py build
idf.py -p COM5 flash monitor
```

The GUI stages this project in a path without spaces before running either
toolchain. `Setup` builds this target and prepares the PlatformIO toolchain.

## Record and plot

For simultaneous camera and BLE sensors, use the GUI **Recording** tab's default
OAK-D and Bluetooth sources, with both IMU and EMG selected. The BLE recorder
writes `emg.csv`, `imu.csv`, `packets.jsonl`, and `meta.json`. Process the saved
EMG and plot the result with:

```powershell
python src/recording/emg_processing.py --file outputs/recordings/SESSION/emg.csv
python src/recording/plot_sensor_recording.py --file outputs/recordings/SESSION/emg.filtered.csv
```

USB is still used for flashing and console diagnostics.
The BLE recorder preserves every received packet and writes individual sample
times. A connected 30-second trial delivered 120,000 EMG samples and both IMUs
at about 100 Hz without detected notification gaps. The link configuration in
`main/ble_connection.c` restores HT25's data-length, PHY and interval requests;
NimBLE notification logs are suppressed to avoid unnecessary USB output.
See [hardware validation](../../docs/HARDWARE_VALIDATION.md) for evidence,
rollback files and the remaining signal-quality and camera-alignment checks.
