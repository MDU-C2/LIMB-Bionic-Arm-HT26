# Architecture and working conventions

The [workflow guide](workflow-guide.md) is the operator reference. Keep labels,
defaults, examples and output descriptions consistent with the application.
Module READMEs identify responsibilities and link to that guide.

## Ownership

| Boundary | Owner and entry points |
| --- | --- |
| Desktop | `gui.app.ProjectGui`; console `aurora` and module `aurora.gui.app` |
| Session inputs/results | `gui.session.ApplicationSession`; panels alias its variables, preferences exclude live validity |
| Slow interface jobs | `gui.background_tasks.BackgroundTasks`; capability probes, disk scans and bulk parsing |
| Named specialist tools | `gui.tool_catalog` settings and `gui.workflow_tools.WorkflowTools` adapters to existing services |
| Child lifecycle | `gui.process_manager`; handoff, structured commands/results, bounded logs and shutdown |
| BLE | `sensors.ble_input.BleSensorReader`; one connection, subscribers for graphs/reference/raw capture |
| Serial | `gui.serial_sensor` or the active recorder/controller; explicit handoff between owners |
| Protocol/time | `sensors.ble_protocol`, `imu_protocol`, `clock_sync`; strict versions, units, sequences and epochs |
| Live processing | `sensors.live_input`, `fusion`, `arm_alignment`; separate acquisition/inference from control snapshots |
| Participant collection | `recording.record_cup_trials`, `cup_protocol`, `cup_trials`; commands, state transitions and trial identities |
| Raw camera capture | `recording.cup_input` and `oak_session`; received RGB/depth independent of optional tracking |
| Guided views | `gui.guidance`; local vector drawings consume protocol snapshots without advancing time |
| Simulation | `simulation.interactive`; fixed physics steps, robot mapping/limits and optional displays |
| Saved data/research | `recording.saved_signals`, `emg_processing`, `exports`; optional `ml` training/evaluation |

Live control and participant collection have different acquisition policies:
control publishes recent state; collection preserves received raw input and reports
loss. They share protocol decoding, alignment fitting and fusion. Do not merge
these policies by making raw capture depend on render or inference completion.

The parent releases a BLE/serial/camera owner before a child opens that device.
A failed shutdown must not open a duplicate owner. Subprocesses receive typed
arguments and structured stdin commands; ordinary GUI use requires no CLI text.
`[CUP]` events carry recorder status/protocol/results. `[AURORA]` saved-result
events carry an artifact path and kind, separately from human-readable log text.
Publish an artifact after finalization; an incomplete recording remains a result
whose manifest describes its quality. Display-log overflow must not discard saved
results or process exits. Nested helpers use noninteractive stdin so they cannot
consume the parent's stop commands.

Review of a saved trial and hardware-free replay may run beside an idle cup
recorder. This does not transfer its connection or calibration. Whole-subject
export and training wait until collection finishes; exporting a single saved
trial cannot include the folder where new trials are created.

## State, calibration and coordinates

Session identity, arm, device, output and applicable settings have one set of
shared inputs. Lock conflicting edits while a device-owning operation runs.
Derived results have their own output path and never replace the capture root.
Preferences restore convenience settings only. Current calibration belongs to the
backend's subject, arm, mounting and connection scope. Reconnects and explicit
mounting changes invalidate it. A view must not infer readiness from status text,
render cadence or a saved preference. Pause/reset may retain a valid scope.

The calibration/trial state machines own preparation, countdown, GO and progress.
Both operator and participant render the same movement identifiers and snapshots.
Keep an illustration's direction consistent with `REFERENCE_STEPS` and apply
camera reflection once. The physical cup marker layout does not swap with arm side.

Keep gyro bias, mounting rotation, anatomical reference, camera/body reference,
retargeting and EMG rest/grip separate. Human measurements remain unclipped.
Relative changes cannot silently become anatomical angles; robot position/speed
limits apply in command mapping. Metric camera points require calibrated scale
or matched depth. IMU-only heading and hidden landmarks have limited observability.

BLE acquisition, receipt and mapped times are different. Camera host timestamps
mark receipt before inference. OAK and ESP32 clocks are separate domains. Integrate
new gyro measurements once and expire channels independently. RGB/depth matching
uses device timestamps; video duplicates are not newly acquired frames. See
[Bluetooth timing](bluetooth-sync.md) and [recording formats](recording-guide.md).

## Files and validation

Preserve authorship, relevant licenses, bundled models and original/inherited
datasets. New attempts, repeats and training runs get distinct identities.
Raw data stays separate from derived data. New training runs use
`outputs/training` with `training.json`; changing the bundled model requires a
reviewed selection. Use ignored `outputs/` directories for diagnostic results.

Keep normal runtime imports independent of optional training, camera and physics
packages. Prefer small explicit components; add adapters to existing backends
instead of duplicating an implementation for a button. Internal helpers stay
internal. Streaming file hashes live in `file_utils`, without signal-processing
imports. Compatibility names may forward to that implementation.

From the environment prepared by Setup, run:

```powershell
python scripts/check_project.py
python -m ruff check src tests scripts
python scripts/check_package.py
```

The runtime suite separates optional training prerequisites and hardware jobs.
Native GUI checks need a working desktop/Tcl environment. Package checks build
and install outside the checkout, verify resources and links, and exercise paths
with spaces/non-ASCII characters. Add behavioral tests for changed contracts:
timing, signals, calibration validity, raw completeness, selected settings/results,
cancellation and failure recovery. Replay checks software consistency; independent
physical references are needed to assess tracking accuracy.
