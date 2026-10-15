# ESP32-C3 sensors

The firmware records EMG at 4 kHz and reads up to two LSM6DSO32 IMUs at about
100 Hz. Data uses BLE by default. The device name is `LIMBServer`.
USB is used for power, flashing and diagnostics.

## Wiring

- EMG output: GPIO0 / ADC1 channel 0.
- IMU bus: SDA GPIO2 and SCL GPIO1.
- Shoulder IMU: address `0x6B`; wrist IMU: `0x6A`.
- Both IMUs share the bus and need different SA0/SDO settings.

The bus also tries SDA/SCL 4/5 and 8/5. Missing IMUs are checked again while
running. Sensor and transport options are in `include/config.h`.

BLE sends all acquired EMG samples and uses the ESP32 clock for sample times.
Clock synchronization puts those times on the same clock as the camera.
USB streaming, when enabled, sends every tenth EMG sample at 400 Hz.

## Build and flash

Use the app's Firmware tab, or run these commands from this folder:

```powershell
platformio run
platformio run -t upload --upload-port COM5
platformio device monitor --port COM5
```

For a native ESP-IDF environment:

```powershell
idf.py set-target esp32c3
idf.py build
idf.py -p COM5 flash monitor
```

PlatformIO uses `espressif32@6.10.0` with ESP-IDF 5.4. The app builds from a
working folder without spaces because the toolchain needs it.

## Recording

Select Bluetooth under Recording in the app. Use the guided grip option to
collect labelled rest, grip, hold and release phases. Raw samples and clock
information are saved with every session.

See the [recording guide](../../docs/RECORDING_GUIDE.md) for the saved files
and signal checks.
