# Source code

The Python package is `aurora/`. The installed command `aurora` and module
`python -m aurora.gui.app` open the same desktop application after Setup.

- `gui/`: interface, shared session, guidance and backend adapters.
- `sensors/`: acquisition, strict decoding, camera tracking, calibration and fusion.
- `recording/`: trial state, raw persistence, saved-signal processing and exports.
- `simulation/`: robot models, command mapping, physics and playback.
- `ml/`: optional movement profiles and EMG training/evaluation.

Follow the [workflow guide](../docs/workflow-guide.md) to use the application.
[Architecture and conventions](../docs/architecture.md) describes ownership,
timestamps, calibration validity and development checks.
