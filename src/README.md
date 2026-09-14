# Computer-Side Software

## Purpose

This folder will contain AURORA software that runs on a computer rather than an embedded controller: communication, control, sensing, trajectories, simulation, configuration, and software safety mechanisms.

## What belongs here

- Reviewed host-side packages with clear interfaces.
- CAN and BLE adapters, control and sensing pipelines, trajectory handling, and an approved simulator.
- Runtime configuration whose schema and ownership are documented.
- Software safety checks that are part of the host application.

## What does not belong here

- ESP32 or other microcontroller firmware.
- Standalone research prototypes that are not integrated into the main system.
- Tests, throwaway scripts, caches, virtual environments, downloaded models, raw datasets, or build output.
- Code assumed to be active only because it existed in LIMB.

## Possible LIMB migration

Candidates include the layered Python host program, shared packet and queue models, SocketCAN and Bleak adapters, configuration, sensor processing and fusion, vision, trajectory interfaces, and a selected simulation path. The team must first determine which implementation was active and separate reusable modules from experiments, cached assets, datasets, and incomplete paths.

## Current software

The [project GUI](gui/) is the shared launcher and extension point for host
tools. Its [extension guide](gui/README.md) explains how to add tabs and program
buttons. The isolated [PyBullet simulation](simulation/) is documented in the
[simulation guide](../docs/SIMULATION.md). Other host software has not been
migrated.
