# Examples

## Purpose

This folder will contain small, focused examples that teach users how to call confirmed AURORA functionality.

## What belongs here

- Minimal examples for stable, documented public interfaces.
- Safe, explicit examples that state prerequisites, expected output, and whether hardware is involved.
- Examples kept in sync with tests and user documentation.

## What does not belong here

- The main application, reusable libraries, experiments, benchmarks, or debugging scraps.
- Setup utilities or hardware test programs.
- Examples that transmit actuation commands by default or imply unverified functionality works.

## Possible LIMB migration

LIMB contains Python CAN snippets, BLE utilities, ESP-IDF component examples, simulation launch paths, and subsystem demonstrations. They may inform future examples only after the underlying interfaces are selected and tested; hardware-control examples require an additional safety review.

## Migration status

Migration has not started. This folder contains no runnable examples.
