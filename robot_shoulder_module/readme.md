# Robot shoulder node

This program is responsible for controlling three motors in the arm:

- Shoulder up/down modded continuous servo motor
- Shoulder left/right modded continuous servo motor
- Upper arm rotation stepper motor

> [!CAUTION]
> Disconnect motor power before flashing or connecting the serial monitor.
> The current `CAN_TX_GPIO` assignment uses GPIO6, which the ESP32-WROVER-IE uses for SPI flash.
> Set `CAN_TX_GPIO` to the pin wired to the CAN transceiver before running the firmware.

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

## Configuration

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
