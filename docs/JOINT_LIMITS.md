# LIMB joint limits and coordinate conventions

The simulator represents the built LIMB prosthesis, not the full anatomical
range of a human arm. Its default limits come from the final HT25 motor
firmware at commit
[`cbbe903`](https://github.com/MDU-C2/LIMB-HT25/tree/cbbe903c66e6b0bf9944b1183f3d3aa2cb8270bc).

| Movement | Simulated range | Firmware evidence |
| --- | ---: | --- |
| Shoulder up/down | 0 to about 75 deg | Firmware maps 0 to 90 deg, but its calibration comment says the assembled arm actually reaches about 75 deg |
| Shoulder left/right | 5 to 40 deg | Potentiometer endpoints are documented as 5 and 40 joint degrees |
| Upper-arm axial rotation | -60 to 60 deg | Potentiometer endpoints map directly to -60 and 60 deg |
| Elbow flexion | 0 to 60 deg | The 72 deg potentiometer interval is geared to 60 joint degrees |
| Forearm/wrist rotation | 0 to 140 deg | Hand-module wrist servo configuration |
| Thumb / index / middle / ring / pinky | 0 to 30 / 85 / 90 / 50 / 90 deg | Hand-module servo configuration |

Primary sources:

- [shoulder and upper-arm firmware](https://github.com/MDU-C2/LIMB-HT25/blob/cbbe903c66e6b0bf9944b1183f3d3aa2cb8270bc/src/esp32/robot_shoulder_module/main/robot_shoulder_module.c)
- [elbow firmware](https://github.com/MDU-C2/LIMB-HT25/blob/cbbe903c66e6b0bf9944b1183f3d3aa2cb8270bc/src/esp32/robot_elbow_module/main/main.c)
- [wrist and finger firmware](https://github.com/MDU-C2/LIMB-HT25/blob/cbbe903c66e6b0bf9944b1183f3d3aa2cb8270bc/src/esp32/robot_hand_motor_module/main/main.c)

The old `LIMB Simulation/simulation.py` ranges are not used as robot limits.
They describe extra degrees of freedom that the final firmware does not drive,
and several of their labels do not match the URDF axes. The separate
`system_config.yaml` limits also contradict the final module calibrations.
Neither the HT25 tree nor Oscar Agren's
[`DMP-arm`](https://github.com/oscaragren/DMP-arm) tree contains a mechanical
range-of-motion report; the PDFs in HT25 are electronics drawings. A later
bench-calibration report should supersede these values if one exists outside
the repositories.

## Left-arm model convention

`left_arm.urdf` is an exact reflection of the working right-arm model across
world X. Positions, rotations, meshes, inertias, joint axes, and every finger
are mirrored together. The elbow command alone is sign-normalized so that
positive degrees always mean flexion.

The imported CAD has zero shoulder elevation with the arm horizontal. Camera
and IMU control uses the anatomical convention where zero flexion is arm-down,
so live control applies `CAD shoulder Y = 90 deg - anatomical flexion` before
the physical limit is enforced. Elbow zero is straight. Wrist zero is the
neutral/reset command used by HT25; there is no unmeasured 70 deg mounting
offset.

## Live control and constraints

The rotation mapping follows the trunk-relative geometry in
[DMP-arm's simple kinematics](https://github.com/oscaragren/DMP-arm/blob/88b0d2ea5df63764ebb56aad8926f92084926fc7/kinematics/simple_kinematics.py).
Its camera axial estimate requires a bent elbow; with a straight elbow there is
no camera axial target. IMU twist adapts gyro integration and relative upper-arm/
forearm orientation from
[HT25's sensor logic](https://github.com/MDU-C2/LIMB-HT25/blob/cbbe903c66e6b0bf9944b1183f3d3aa2cb8270bc/LIMB%20Simulation/Sensors_Calculations/sensor_logic.py).
The signed mounting axes map both sensors into a common bend-X, length-Y frame.
The forearm calculation projects common gyro motion onto its axis at the measured
elbow bend before subtracting it. Upper-arm camera observations correct the gyro
state once per new frame, so IMU movement is retained between observations.

The gyros use decoded degrees/s and device sample timing, rather than copying
HT25's rad/s conversion into the current packet path. Half a second of rest
establishes gyro bias. Stored commands stay within the ranges above to avoid
integration beyond a stop delaying reverse motion. Upper-arm drift can be
corrected by camera; the old camera mapping has no forearm-twist observation.
Wrist drift therefore needs recalibration. Q/E and Left/Right provide manual
fallback for unavailable twist targets. See [mounting checks](SIMULATION.md#mounting-and-twist-calibration).

Runtime arm commands and both loaded URDFs use the ranges above. No extra
constraint fixes the shoulder or wrist in place. The scene's fixed constraints
anchor the cup to the world and, during a physics grasp, attach it to the hand.

Each HT25 finger has one servo, while the CAD hand has multiple revolute joints.
The simulation retains a shared open-to-grip pose rather than claiming measured
independent finger or tendon motion. Its joint poses stay within the URDF limits.
In physics mode, live grip progress is limited by the slowest normalized servo
travel: 90 degrees at 40 degrees/s gives 2.25 seconds for a full close/open.
The kinematic live preview follows the bounded grip target directly.
Thumb, index, middle and
ring use 40 degrees/s; pinky uses 120 degrees/s. These speeds come from the same
hand firmware as the ranges. Keyboard and assisted cup-grasp animation retain
their existing rates.
