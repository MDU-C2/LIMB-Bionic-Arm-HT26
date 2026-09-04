# Scripts

## Purpose

This folder will contain AURORA utilities for setup, validation, analysis, conversion, and development tasks that do not belong in the main application.

## What belongs here

- Repeatable, documented utilities with safe defaults and clear inputs and outputs.
- Environment validation and non-destructive diagnostic tools.
- Approved data or design conversion helpers whose generated outputs are identified.

## What does not belong here

- Core application logic or firmware.
- One-off commands without documentation, destructive cleanup tools, secrets, machine-specific state, or generated output.
- Scripts that enable CAN hardware, flash controllers, or actuate motors without a separately reviewed procedure and explicit safeguards.

## Possible LIMB migration

LIMB includes a Jetson CAN setup script and many analysis, capture, conversion, calibration, and development utilities spread across subsystem directories. Candidates should be deduplicated, made portable where appropriate, given safe defaults, and separated from active control before migration.

## Migration status

Migration has not started. This folder contains no executable AURORA utility.
