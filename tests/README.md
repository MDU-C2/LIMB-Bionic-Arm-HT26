# Tests

## Purpose

This folder will contain AURORA unit, integration, communication, simulation, and safety-validation tests.

## What belongs here

- Deterministic tests for functional code and documented interfaces.
- Clearly labelled integration and hardware-in-the-loop tests with prerequisites and safeguards.
- Fixtures and test data that are small, non-sensitive, and traceable.
- Tests for protocol compatibility, limits, fault handling, and simulation behavior.

## What does not belong here

- Ad hoc actuation scripts disguised as tests.
- Tests that contact real CAN/BLE devices or move hardware without explicit isolation, labelling, and operator controls.
- Application code, firmware, caches, transient reports, or large captured datasets.

## Possible LIMB migration

LIMB has tests for host layers and end-to-end pipelines, node and CAN behavior, DMP reproduction, and numerous ESP-IDF examples or experimental test programs. Each candidate must be classified by scope and hardware effect, made reproducible, and linked to the code it validates.


