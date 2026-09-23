# Sensors and recording

The current software supports the BLE cuff, OAK-D camera, and a generic serial
stream.

## BLE cuff

The cuff is expected to advertise as `LIMBServer` and provide EMG, IMU, and
piezo data. The matching firmware currently remains in the older LIMB25 project:

```text
LIMB-HT25/src/esp32/human_lower_arm_module
```

That firmware targets an ESP32-C3-Zero. Its LSM6DSO32 IMU connection is:

| IMU | ESP32-C3-Zero |
| --- | --- |
| 3.3 V | 3.3 V |
| GND | GND |
| SDA | GPIO 4 |
| SCL | GPIO 5 |

The firmware expects I2C address `0x6A`.

To use it:

1. Flash the legacy cuff firmware from an ESP-IDF terminal.
2. Reset the ESP32 and check that the IMU is detected in its serial output.
3. Start the AURORA GUI and keep the BLE device name as `LIMBServer`.
4. Open **Sensors**.
5. Open the EMG, IMU, and piezo previews you want to inspect.

The three windows use one BLE connection and may stay open together. Packet
counts should increase. Move the cuff slowly to check acceleration, gyroscope,
pitch, and roll. Use **Zero tilt** when the cuff is still.

EMG is the electrical muscle signal. The IMU measures acceleration and angular
velocity. The piezoelectric sensor reacts to bending, tapping, and vibration;
its value is not a calibrated force measurement.

## OAK-D camera

Connect the OAK-D and choose **Open camera** in **Sensors**. The window shows the
selected arm, body points, hand landmarks, and angle estimates. It does not save
until you press `R` or **START REC**.

Depth mode can be enabled with the OAK-D arguments in **Recording**, but it still
needs to be retested on the project camera.

## Serial data

The USB-connected ESP32/LSM6DSO32 firmware under
`firmware/imu_i2c_scanner` sends newline-separated JSON at 115200 baud. On GUI
startup, the **Connected ESP32 + LSM6DSO32** card in **Sensors** automatically
opens the first detected port and displays ESP32 uptime/free heap, IMU
temperature, acceleration in g, and angular velocity in degrees per second.

Use **Connect / reconnect** after selecting a different port under Recording.
If the card says that access is denied, close PlatformIO, Arduino, or another
serial monitor that has the COM port open; the GUI retries every two seconds.

## Record camera and sensors together

Close the previews, open **Recording**, select the sources, and press
**Start selected sources**. BLE saves all available cuff streams. The camera
starts recording automatically in a batch.

Files are written below `outputs/recordings/`. Sources share a session ID, but
their clocks are not hardware-synchronized.
