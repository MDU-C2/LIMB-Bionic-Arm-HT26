# Documentation

## Purpose

This folder is the home for AURORA's shared technical and project documentation: architecture, communication protocols, setup procedures, decisions, troubleshooting, safety guidance, migration records, and daily updates.

## Start here

- [Development guide](development-guide.md) — where work belongs and how to reuse LIMB work safely.
- [Development resources](resources.md) — official and project references for the current technology and hardware candidates.

## What belongs here

- System and subsystem architecture.
- Confirmed protocol and interface definitions.
- Reproducible setup, calibration, repair, and troubleshooting procedures.
- Design decisions, safety assumptions, migration records, and team updates.

## What does not belong here

- Functional host code or firmware.
- PCB, CAD, and other editable hardware source files.
- Unexplained binary exports, raw datasets, caches, or build output.
- Instructions presented as verified when they have not been tested on AURORA.

## Possible LIMB migration

LIMB contains documentation for CAN messages and data layouts, the Jetson AGX Orin, ESP32 microcontrollers, BLE, ADC behavior, servo modification, simulation, and individual firmware modules. These documents may be migrated after their accuracy is checked against source code and the physical arm. Original paths and any changes must be recorded.

## Migration status

Migration has not started. The files currently in this folder were written for AURORA Step 1; no LIMB document has been copied.
