# Motor inventory and torque meaning

## What the HT25 files establish

The [HT25 BOM](https://github.com/MDU-C2/LIMB-HT25/blob/main/docs/BOM_Bionic_Arm.xlsx)
lists seven motors in its **Arm** section: three NEMA17, two NEMA23, and two
JX PDI-HV2060MG. The three Hitec HS-422 servos are listed separately under
**Hand** and are not part of that count of seven.

The final firmware documentation does not map all seven BOM entries to joints:

| Motion | Motor named by firmware |
| --- | --- |
| Shoulder up/down | JX PDI-HV2060MG, modified for continuous rotation |
| Shoulder left/right | JX PDI-HV2060MG, modified for continuous rotation |
| Upper-arm rotation | NEMA17-04 through DRV8825 |
| Elbow bend | NEMA17-04 through DRV8825 |
| Lower-arm/wrist rotation | Whadda WPK601 |
| Fingers | Hitec HS-422 system |

Sources: [shoulder firmware](https://github.com/MDU-C2/LIMB-HT25/tree/main/src/esp32/robot_shoulder_module),
[elbow firmware](https://github.com/MDU-C2/LIMB-HT25/tree/main/src/esp32/robot_elbow_module),
and [hand firmware](https://github.com/MDU-C2/LIMB-HT25/tree/main/src/esp32/robot_hand_motor_module).
The third NEMA17 and both NEMA23 motors in the BOM are not assigned by those
final firmware readmes. Check the physical arm, wiring, CAD, and driver boards
before treating all seven BOM motors as installed actuators.

## Datasheet values

| Motor | BOM quantity | Published torque | Other relevant values |
| --- | ---: | --- | --- |
| NEMA17-04 / 42SHD0217-24B | 3 arm | 0.45 Nm holding; 0.3254 Nm maximum dynamic torque at 180 rpm | 1.5 A, 3.3 V, 1.8 deg step |
| NEMA23-03 / 57SHD4934-34B | 2 arm | 3 Nm holding; 2.3229 Nm maximum dynamic torque at 29 rpm | 3 A, 8.4 V, 1.8 deg step |
| JX PDI-HV2060MG | 2 arm | 4.7 Nm stall at 6 V; 6.1 Nm stall at 7.4 V | 0.15/0.13 s per 60 deg; vendor data should be verified |
| Hitec HS-422 | 3 hand | 0.324 Nm stall at 4.8 V; 0.402 Nm stall at 6 V | 0.21/0.16 s per 60 deg |

Datasheets: [NEMA17](https://www.electrokit.com/upload/quick/ea/2d/5b25_41032662-tds.pdf),
[NEMA23](https://www.electrokit.com/upload/quick/e6/0c/2f07_41032667-tds.pdf),
[HS-422](https://www.electrokit.com/upload/product/41002/41002565/hs422.pdf),
and [PDI-HV2060MG](https://servodatabase.com/servo/jx-servo/pdi-hv2060mg).

Holding and stall torque are not safe continuous operating torque. They are
also motor shaft values, while the URDF requires joint torque after gears,
linkages, belts, or lead screws. The transmission ratios and efficiencies are
not recorded well enough to convert these ratings into joint limits. The HT25
firmware readmes also say the NEMA17 DRV8825 drivers were limited to 1 A, below
the motor's 1.5 A datasheet rating. The
larger values in `src/simulation/sim/controller_params.py` therefore remain
clearly labeled simulation controller caps rather than hardware ratings.

## Saved simulation torque

Physics mode can save `outputs/simulation/torque_<timestamp>.csv`. Each row
identifies the logical motion, URDF joint, HT25 motor mapping and published
rating, joint angle and velocity, applied PyBullet motor torque, calculated
gravity torque, inverse dynamics torque, simulation effort limit, and payload
mass. It contains the five driven arm joints. Finger torque is omitted because
the three physical HS-422 servos are not mapped one-to-one to the URDF's finger
joints.

These values are estimates from the current URDF. They are useful for comparing
motions and finding high load regions. They are not measured motor current or
validated hardware torque because the link masses, inertias, and transmissions
still need measurement.
