# AURORA app

Run Setup once, then Start. Follow the [workflow guide](../../../docs/workflow-guide.md)
for session settings, illustrated calibration, cup trials, control and review.
The same guide opens under **Help & status ? Open workflow guide**.

`app.py` composes the desktop panels. `session.py` owns shared inputs, preferences
and selected results. `guidance.py` renders recorder/live-setup snapshots in the
operator and participant views. `workflow_tools.py` connects typed forms to existing
services; `process_manager.py` supervises device handoff and child shutdown.
`background_tasks.py` keeps capability checks, scans and parsing off the event loop.
See [architecture](../../../docs/architecture.md) for boundaries and checks.
