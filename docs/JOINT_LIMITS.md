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
