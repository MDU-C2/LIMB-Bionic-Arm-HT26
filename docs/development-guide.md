# Development Guide

This guide explains where work belongs and how to develop AURORA without losing useful LIMB knowledge. The project is still in the inspection stage; no legacy code has been migrated yet.

## Choose the correct folder

| Work | Location | Examples |
| --- | --- | --- |
| Computer-side Python | `src/` | Communication adapters, general control logic, sensing, trajectories, simulation, configuration, and software safety checks. |
| Embedded firmware | `firmware/` | ESP32 applications, embedded drivers, board-specific pin mappings, and fail-safe behavior. |
| Physical system files | `hardware/` | BOM, schematics, PCB sources, wiring, repairs, calibration records, and selected CAD. |
| Experimental work | `research/` | DMP, adaptive control, early sensor-fusion ideas, and other work not yet part of the main system. |
| Tests | `tests/` | Unit, integration, communication, simulation, and clearly labelled hardware-in-the-loop tests. |

Small usage demonstrations belong in `examples/`, reusable development utilities belong in `scripts/`, and explanations or procedures belong in `docs/`.

## Reuse LIMB work carefully

Do not rewrite a subsystem merely because the old repository is untidy. LIMB contains useful implementation and design work that should be understood before the group decides whether to reuse, adapt, or replace it.

Before integrating old code:

1. Read the code, its nearby documentation, configuration, and tests.
2. Record its original LIMB path and intended hardware or software role.
3. Identify dependencies, duplicate implementations, generated files, unfinished sections, and safety assumptions.
4. Test the smallest isolated behavior possible without connecting hardware.
5. Migrate one reviewed subsystem with its documentation and tests.

Finding a file in LIMB does not prove that it is current or working. Mark uncertain conclusions as **unconfirmed**.

## Separate general logic from hardware access

Keep reusable decisions and calculations separate from code that opens CAN or BLE connections, reads pins, or drives motors. For example, general joint-limit checking should not need a live CAN interface. A hardware adapter can translate an approved command into a device-specific message.

This separation makes it easier to test logic without the arm, replace hardware later, and prevent an ordinary unit test from moving a motor.

## Document interfaces and assumptions

Code that exchanges or interprets data must document:

- Units, coordinate frames, sign conventions, and valid ranges.
- CAN identifier, sender, receiver, payload layout, byte order, rate, timeout, and error behavior.
- Configuration value meaning, unit, safe range, default, and source.
- Hardware revision, pin mapping, calibration, timing, and other important assumptions.

Use one unit consistently inside an interface and convert explicitly at its boundary. Avoid unexplained numbers in code or configuration.

Update the relevant README or protocol document whenever behavior or an interface changes. Documentation and tests are part of the change, not later cleanup.

## Hardware and firmware safety

Coordinate with the group before flashing firmware, enabling a CAN interface that can transmit, powering actuators, or testing motors. First confirm the controller, firmware target, wiring, power isolation, limits, expected movement, and emergency-stop method.

Hardware tests must be clearly labelled and must not run as part of an ordinary test suite. Start with actuator power removed and prefer inspection, unit tests, simulation, or listen-only diagnostics before controlled movement.
