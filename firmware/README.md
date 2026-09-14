# Firmware

## Purpose

This folder will contain AURORA firmware for ESP32 boards and any other embedded controllers used by the arm or its sensing equipment.

## What belongs here

- Confirmed controller applications and reusable embedded components.
- Board-specific pin mapping and configuration kept close to the firmware that consumes it.
- Embedded interfaces, fail-safe behavior, and version information needed to reproduce a controller image.

## What does not belong here

- Computer-side Python, hardware design sources, firmware build output, flashed binaries, SDK caches, or unrelated experiments.
- Multiple unexplained copies of the same driver or component.
- Unreviewed code that can actuate hardware.

## Possible LIMB migration

LIMB candidates include ESP-IDF targets for the robot shoulder, elbow, hand motors, hand pressure sensors, and human lower-arm cuff, together with CAN/TWAI, BLE, ADC, IMU, potentiometer, servo, continuous-servo, stepper, and motor-ramping components. Before selection, the team must confirm which firmware is installed on each board, reconcile duplicate components, and review all actuation and failure behavior.

