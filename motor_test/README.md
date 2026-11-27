# Temporary encoderless motor demo

## Build and flash

Use ESP-IDF v5.5.2. With motor power disconnected, run these commands in each firmware directory.

| Directory | Target |
| --- | --- |
| `robot_shoulder_module` | `esp32` |
| `robot_elbow_module` | `esp32c3` |

```sh
idf.py set-target <target>
idf.py menuconfig
# Turn on LIMB config → Enable temporary encoderless motor demo.
idf.py -p <serial-port> build flash
```

## Align and run

Support the arm and align each joint with motor power disconnected. Use the project's physical joint datums or verified bench marks. Check that the joints stay at their references from power-up until arming.

| Axis | Required physical start pose | Existing configured travel |
| --- | --- | --- |
| Shoulder up/down | Joint 0°, corresponding to 100° in its existing encoder mapping | −60° to +60° |
| Shoulder left/right | Joint 0°, corresponding to 100° in its existing encoder mapping | −60° to +60° |
| Upper arm rotation | Joint 0°, corresponding to 100° in its existing encoder mapping | −60° to +60° |
| Elbow | Joint 0° at the low end of its configured joint travel, corresponding to 195° in the legacy potentiometer mapping | 0° to +60°, from the legacy potentiometer endpoints and ratio |

The elbow's encoder calibration is missing in the base configuration. Demo mode reuses its legacy joint mapping. Verify the elbow's physical zero, direction, travel and STEP pulse count on the isolated bench before listing it as verified. Validate the other axes before listing them as verified as well.

From the repository root, run `python3 motor_test/motor_control.py` for a CAN-frame preview. For live mode use Python 3 with Tkinter and `python-can`, and bring up the existing SocketCAN interface at **1 Mbit/s**. With a physical power cutoff available and nobody attached to the robot, run the panel and list only the bench-validated axes.

```sh
python3 motor_test/motor_control.py --live --channel can0 --verified updown leftright upper
# Add elbow to --verified only after the elbow bench checks.
```

Select the aligned axes and click **Confirm start pose**. Wait for their estimates to appear, then enter a nearby absolute target and click **Start selected**. The panel keeps Christian's 5° per-test target window, 1°/s speed cap and seven-second test duration. The per-axis **Stop** and **Stop all** buttons let the current burst complete while retaining the estimate for the next test. **Emergency stop** stops all axes and latches them off. Use the physical cutoff if motion continues or a stop cannot be confirmed.

Shoulder counts electrical STEP edges with PCNT. The elbow ESP32-C3 credits a finite RMT waveform only after it completes. A stalled motor, missed motor step, or moved joint can invalidate the physical reference even when pulse counting succeeds. After a suspected stall, interruption, loss of estimates or emergency stop, cut motor power, reset and realign affected axes, then confirm the pose again. Reset and realign before starting a new live GUI session.

## CAN behavior

The demo preserves the standard CAN IDs and payload sizes. The actuation payload remains two little-endian floats (joint target, speed). Positive-speed commands need refreshing within 500 ms. A speed of `+0.0` requests a normal stop. A speed of `-0.0` with the configured reference target is the explicit arming command. The existing empty STOP frame latches an emergency stop. The existing one-float angle frame carries an estimated joint angle, or NaN for an unarmed or faulted axis. Loss of command refresh or an unverifiable pulse burst latches the affected axis off.

## Return to encoder feedback

With motor power disconnected, turn off **Enable temporary encoderless motor demo** in both projects and rebuild and flash. Reconnect the original encoders and shoulder-to-elbow SPI feedback link, and use the original encoder-based controls. Elbow encoder mode still requires its measured calibration and fresh SPI feedback.
