# Robot shoulder node

This program is responsible for controlling three motors in the arm:

- Shoulder up/down modded continuous servo motor
- Shoulder left/right modded continuous servo motor
- Upper arm rotation stepper motor

> [!CAUTION]
> Disconnect motor power before flashing or connecting the serial monitor.

## Steps for building and flashing the program

The shoulder node uses an ESP32-DevKitC V4 with an ESP32-WROVER-IE module.
Use the `esp32` target in ESP-IDF.

```sh
idf.py set-target esp32
idf.py menuconfig # In case you need to configure some settings.
idf.py build flash
```

Once you've set the target and set your configuration in menuconfig, you can just run the last command
from then on.

## Monitoring the program

Connect the ESP32-DevKitC V4 to your computer over USB, then run:

```sh
idf.py monitor
```

## Open-loop CAN motor test

1. With motor power disconnected, unload the axes and place the joints at known starting angles. Set each `.starting_angle` in `kShoulderAxes` in [robot_shoulder_module.c](main/robot_shoulder_module.c) to the corresponding physical angle. The defaults assume all three axes start at zero degrees.
2. Set `kEnableOpenLoopControl` to `true`, then build and flash the shoulder firmware. Connect the CAN transceiver as for normal operation.
3. Apply motor power once the firmware has booted. Send the usual CAN actuation messages `0x220`, `0x221`, and `0x222`. Each message contains two little-endian 32-bit floats in order. Send the target joint angle in degrees, then the speed limit in degrees per second. A speed of zero stops issuing pulses without latching a STOP.
4. The usual STOP messages `0x120`, `0x121`, and `0x122` latch the selected axis off until reboot. The motor task checks the latch on its 10 ms cycle. The usual shoulder angle telemetry IDs carry the estimated joint angles from the counted STEP edges.

Each startup assumes the configured reference angles. A stall, missed motor step, or movement while the driver is disabled can make the estimate drift. Open-loop mode uses the shoulder motor calibration for software joint limits and skips the encoder reader and elbow encoder forwarding.

Set `kEnableOpenLoopControl` to `false` and rebuild to resume encoder feedback control.

## Configuration

### Elbow encoder feedback

Shoulder reads the elbow encoder on `SPI2_HOST` using its chip select on GPIO32.
After each successful read, the 10 ms encoder task sends the new sample to the
elbow ESP on the same bus using a separate, active-low chip select. Shoulder is
the SPI master for both transactions.

Shoulder J8 pins 4–8 are the five active-low chip selects. The assignments
match [the shoulder pin definitions](main/robot_shoulder_module.c).

| J8 pin | Shoulder GPIO | Selected device |
| --- | --- | --- |
| 4 | GPIO27 | Elbow ESP |
| 5 | GPIO26 | Shoulder up/down encoder |
| 6 | GPIO25 | Shoulder left/right encoder |
| 7 | GPIO33 | Upper-arm rotation encoder |
| 8 | GPIO32 | Elbow encoder |

| Signal | Shoulder ESP32 output | Elbow ESP32-C3 input |
| --- | --- | --- |
| SCLK | J8 pin 3 / GPIO14 | J6 pin 4 / GPIO2 |
| MOSI | J8 pin 1 / GPIO13 | J6 pin 2 / GPIO4 |
| CS (active low) | J8 pin 4 / GPIO27 | J6 pin 3 / GPIO3 |
| GND | J8 pin 10 / GND | J1 pin 1 / GND |

The current design uses one-way SPI mode 0 at 500 kHz, with MSB-first transfers. Each CS assertion carries
four bytes in order: `0xA1`, count high byte, count low byte, and CRC-8 over the first three bytes
(polynomial `0x07`, initial value `0x00`, MSB first, final XOR `0x00`). Counts are unsigned 12-bit
encoder counts in the range 0–4095. Elbow converts them to encoder degrees using `counts * 360 / 4096`.
The next successful read provides a new frame if elbow misses one while it starts or rearms its receiver.

### Force reenabling CAN when entering a Bus Off state

The program can be configured to automatically reenable the CAN bus in case a Bus Off state occurs
by running `idf.py menuconfig` and checking `LIMB config ---> Force reenable CAN on bus off`.
The CAN bus is noisy enough that this module risks entering a Bus Off state regularly, so
it's probably a good idea to check it.

> [!warning]
> The Bus Off state happens for a reason, so be careful if you force reenable the bus.
> Read the [CAN driver components readme] for more information.

## Servo motors

The servo motors are both of the model JX Servo PDI-HV2060MG. For the LIMB project, both servo motors have
been modified to support continuous rotation. As such, the
[continuous servo component](/src/esp32/components/continuous_servo/) should be used instead of the
[regular servo component](/src/esp32/components/servo/).

More information regarding the continuous rotation modification can be found
[in the documentation](/docs/servo_modification.md).

## Stepper motor

The stepper motor is a [Joy-IT NEMA17-04](https://joy-it.net/en/products/NEMA17-04)
controlled using a [Pololu DRV8825 stepper motor driver](https://www.pololu.com/product/2133).
The stepper driver is current limited to 1 A by setting the reference voltage to
0.5 V using the process explained on its [product
page](https://www.pololu.com/product/2133).

## Motor joint and speed limits in CAN actuation messages

The [continuous servo](/src/esp32/components/continuous_servo/) and [stepper](/src/esp32/components/stepper/) components
will automatically clamp the angle and speed to the minimum and maximum limits, so it's safe to send
any joint angles and speeds.

## CAN messages

The program receives CAN messages with a recipient ID of `CAN_RECIPIENT_ROBOT_SHOULDER`:

| CAN message name                             | CAN ID |
| --                                           | --     |
| `CAN_ID_ROBOT_SHOULDER_UP_DOWN_STOP`         | 0x120  |
| `CAN_ID_ROBOT_SHOULDER_LEFT_RIGHT_STOP`      | 0x121  |
| `CAN_ID_ROBOT_UPPER_ARM_ROTATION_STOP`       | 0x122  |
| `CAN_ID_ROBOT_SHOULDER_UP_DOWN_ACTUATION`    | 0x220  |
| `CAN_ID_ROBOT_SHOULDER_LEFT_RIGHT_ACTUATION` | 0x221  |
| `CAN_ID_ROBOT_UPPER_ARM_ROTATION_ACTUATION`  | 0x222  |

Furthermore, it sends the CAN messages:

| CAN message name                                | CAN ID |
| --                                              | --     |
| `CAN_ID_ROBOT_ELBOW_UP_DOWN_POTENTIOMETER`      | 0x4A0  |
| `CAN_ID_ROBOT_UPPER_ARM_ROTATION_POTENTIOMETER` | 0x4A1  |
| `CAN_ID_ROBOT_SHOULDER_UP_DOWN_POTENTIOMETER`   | 0x4A2  |

## Location of the module on the arm

The module is located on the back of the torso, as shown in the image:

![Image of the shoulder module located at the back of the torso of the robot arm](/res/shoulder_module.jpg)

## PCB details

The files for the PCB of the module are located in the [`/designs/KiCad/LIMB_Shoulder/`](/designs/KiCad/LIMB_Shoulder/) directory.
