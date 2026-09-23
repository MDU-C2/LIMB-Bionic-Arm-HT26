# ESP32-C3-Zero + LSM6DSO32 live stream

## Wiring

| IMU | ESP32-C3-Zero |
| --- | --- |
| VIN | 3V3 |
| GND | GND |
| SDA | GPIO8 |
| SCL | GPIO5 |

GPIO8/GPIO5 is tried first. The firmware also tries the older documented pin
pairs GPIO6/GPIO7, GPIO4/GPIO5, and GPIO8/GPIO9. Each pair is tested in both
SDA/SCL orders, and the detected pins are reported in every successful packet.

Do not connect the IMU's VIN to both 3V3 and 5V at the same time.

## Run in VS Code

1. Install the **PlatformIO IDE** extension.
2. In VS Code, choose **File > Open Folder** and open this
   `firmware/imu_i2c_scanner` directory.
3. Connect the ESP32-C3-Zero with a data-capable USB cable.
4. Click **PlatformIO: Upload** (the right-arrow icon in the status bar).
5. Close PlatformIO's serial monitor before starting the AURORA GUI; only one
   program can own the COM port at a time.

The firmware checks both normal IMU addresses (`0x6A` and `0x6B`), configures
the accelerometer and gyroscope at 104 Hz, and sends a JSON line at 20 Hz over
USB serial. The monitor baud rate is 115200. A normal line looks like:

```text
{"device":"ESP32-C3","uptime_ms":1234,"free_heap_bytes":321000,"imu":{"connected":true,"model":"LSM6DSO32","address":"0x6A","temperature_c":25.4,"accel_g":{"x":0.01,"y":-0.02,"z":1.00},"gyro_dps":{"x":0.1,"y":0.2,"z":-0.1}}}
```

Start the desktop app with `python src/gui/app.py`. The **Sensors** tab connects
to the first detected serial port automatically and shows these fields in its
**Connected ESP32 + LSM6DSO32** panel. It keeps retrying if another serial
monitor currently owns the port.

If the IMU cannot be read, the stream contains `"connected":false` and a clear
error. Check power and ground first, then check that SDA and SCL have not been
swapped.
