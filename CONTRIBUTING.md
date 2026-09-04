# Contributing to AURORA

AURORA is being organized for a team that will inherit and restore an existing robotic system. Contributions should make the system easier to understand and safer to change.

## Before making a change

- Read the README in the folder you plan to change and follow its scope.
- Reuse relevant LIMB work instead of automatically rewriting it.
- Understand a legacy file before migrating it; the presence of a file is not evidence that it is current or working.
- Coordinate with the group before changing CAN messages, firmware interfaces, pin assignments, motor limits, or other hardware behavior.
- Coordinate before flashing firmware or powering motors.

## Migrating legacy work

- Migrate one reviewed subsystem at a time.
- Record the original LIMB repository path for every migrated file or coherent group of files.
- Record what was kept, changed, excluded, and why.
- Confirm dependencies, generated artifacts, hardware assumptions, and licensing or provenance before migration.
- Keep experimental or research work separate from active control code until it has an agreed interface and validation evidence.

## Code, tests, and documentation

- Keep changes focused, understandable, and small enough to review.
- Add appropriate tests when adding functional code.
- Update documentation whenever behavior, interfaces, setup, safety assumptions, or hardware mappings change.
- Do not present unverified setup, simulation, communication, or hardware behavior as working.
- Mark uncertain facts as **unconfirmed** and state how they can be confirmed.

## Repository hygiene

- Put files only in the folder responsible for them.
- Do not add caches, generated build output, editor state, virtual environments, downloaded models, or temporary recordings.
- Avoid committing duplicate exports and backup archives when an editable source and reproducible export process are available.
- Do not mix unrelated cleanup, experiments, and functional changes in one contribution.

## Hardware safety

Changes that can affect physical behavior require extra review. Document the expected movement, limits, failure modes, test setup, emergency-stop method, and result. Start with power removed, then simulation or interface-level checks, and only proceed to controlled powered testing after the group agrees that the setup is safe.
