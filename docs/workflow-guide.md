# AURORA workflow

Run **Setup** once, then **Start**. Start uses the prepared environment offline.
It does not install packages. **Help & status → Refresh status** checks optional
capabilities in the background. A missing camera, Bluetooth device or training
package leaves the other tools available. See [setup](../scripts/README.md).

## Connect and inspect

1. Open **Connect & inspect**. Enter the subject ID, select the human arm and
   device, and choose the recording output folder. These inputs are shared with
   Record, Control and the applicable Advanced Tools forms.
2. Use **Connect / inspect BLE**, USB monitoring or **Open camera monitor**
   to inspect signals. Monitors do not create trial recordings. Check that the
   upper-arm IMU is `0x6B` and the forearm IMU is `0x6A`.
3. Frame both shoulders, the selected elbow/wrist and the cup path. Show the
   hips when possible. Hidden trunk landmarks can leave shoulder estimates
   unavailable. Keep the camera, chair, table and sensor straps fixed.

The app remembers subject, arm, device, output and common settings in the ignored
`.aurora/preferences.json` below its working root. This file contains preferences,
not valid calibration. A damaged preference file loads defaults and reports the
problem in Activity. Context changes are blocked during recording or control;
finish the session first. Pace and duration can change between cup trials.

The camera preview is mirrored; saved raw video is unmirrored. The operator
illustrations match the mirrored display. The participant view uses the person's
own perspective and highlights the selected arm. Movement drawings are side-view
schematics, not measured angles. Cup diagrams use the participant's table layout.

## Prepare and calibrate

For collection, open **Record → 1. Connect camera + BLE**. Camera previews and
monitors must release their device before the recorder opens it; the app performs
the supported handoff and reports timeouts. Wait for RGB, stereo depth, both IMUs
and raw EMG. **Signals** opens graphs from the recorder's existing connection.

Press **2. Calibrate subject**. **Open participant view** provides a larger window
with F11 full screen and Escape to leave full screen. Closing that view leaves
the recording session running. Both views follow the same backend movement,
countdown and progress; opening a window never starts or advances a step.

Read the displayed movement and position the person, then press **Start step**.
Preparation has no time limit. Each step counts down for three seconds; begin
only on GO. Keep the movements clear of the tabletop.

| Illustration | Movement after GO | Capture |
| --- | --- | --- |
| Still reference | Upper arm beside body, elbow about 90°, forearm forward, thumb up; stay still | 5 s |
| Upper-arm raising | Keep elbow about 90° and thumb up; raise forward first, lower, repeat slowly | 10 s |
| Elbow bending | Keep upper arm still; bend further first, return, repeat slowly | 10 s |
| Forearm turning | Keep upper arm/elbow still; palm down first, then up, repeat comfortably | 10 s |
| Return | Restore the first thumb-up pose and stay still | 5 s |
| Relaxed EMG | Support forearm, open hand, relax muscle | 5 s |
| Firm grip | Support forearm, grip the empty cup firmly and comfortably | 5 s |
| Release | Release cup and relax open hand | 3 s |

The status shows stillness, missing views/signals, fit results and the step to
repeat. Check it before continuing. Failed references remain saved. Normal trials
require passed preparation and fresh stereo arm points; hand/cup detection is
optional. Detailed axes and thresholds belong to diagnostics.

Calibration separates gyro bias, sensor mounting, anatomical reference, camera
body reference, robot retargeting and EMG rest/grip. A measured 90° human elbow
remains 90° even when the robot has 60° travel. Relative change since the reference
and bounded robot commands are separate values. No still pose establishes all
anatomical degrees of freedom. See [fusion and validity](live-control.md).

Preparation remains valid between trials within the same subject, arm, mounting
and connection. A reconnect, moved sensor/camera, changed subject/arm or invalid
signal requires preparation again. Finishing collection releases the connection;
starting a different live-control process creates a new scope. Saved references
remain reviewable and usable for recorded replay, not proof of live validity.

## Record cup transfers

Use a **60 × 60 cm table and an empty cup**. Marker 1 is the open-hand start,
2 the cup pickup, 3 the placement/release circle to the participant's right of 2,
and 4 the empty-hand finish. Selecting the left arm does not move marker 3 to the
left. The illustrated sequence is **1 → 2 → 3 → 4**.

1. Choose natural, slow or fast pace and a capture window. The default is eight
   seconds after GO; `0` means manual stop. Practice first using **Practice (no
   files)** and lengthen the window if needed, particularly for slow trials.
2. Reset cup to 2 and open hand to 1. Press the numbered **Start** trial button.
   Give the start signal at GO. Files open at GO and keep actual source times.
3. Grasp at 2, place and release at 3, move the empty hand to 4, then wait there.
   The progress bar describes the capture window, not verified marker completion.
4. Timed capture saves automatically. **Stop early** keeps a partial trial as
   incomplete; in manual mode **Stop and save** ends capture. **Restart / repeat**
   retains a started attempt and links the repeat. Cancelling before GO creates
   no trial files and does not consume an identity.
5. Observe the rest/reset countdown. Keep each requested pace in separate trials.
   The initial collection plan is three per pace, at least 20 seconds rest, with
   rotated pace order between participants. Record deviations from that plan.
6. **Review current trial** opens the saved manifest automatically. Collection
   can continue in the same session. **Finish subject** releases the devices and
   unlocks the subject settings.

Requested pace, software/operator cues, capture completeness and actual movement
labels are different facts. Review video/signals before assigning labels. Raw
received samples and frames keep saving through optional tracking loss; inspect
reported gaps, overruns and incompleteness. See [saved files](recording-guide.md).

## Control

Open **Control** and choose **Direct pose preview** or **Motor physics**. Preview
follows mapped robot targets; physics adds gravity, collision, force and actuator
speed limits. Intentional motor limits can slow motion even when processing is
responsive. **Start with keyboard** needs no sensors; [keyboard controls and
scene conventions](simulation-guide.md) explain the keys.

**Connect + open live setup** starts paused with the shared subject, arm, device
and output root. Follow the same five mounting movements, then relaxed EMG and
firm grip. Each EMG step waits for its three-second countdown before collecting;
live grip lasts three seconds. Camera grip skips EMG preparation. Disabling stereo
selects configured-axis diagnostics rather than arbitrary mounting calibration.

Return to marker 1 with an open relaxed hand, then **Start interactive control**.
Check isolated elbow bending, upper-arm raising, outward movement and forearm
turning. Watch human measurement, relative change, robot target and return error.
Keep the trunk still when its reference is unavailable. Pause if signs are wrong.

**Pause / reset robot** or R retains valid same-connection calibration. Return to
the reference before resuming. **Sensors moved / repeat preparation** explicitly
invalidates calibration. Reconnects also require preparation. The larger participant
view continues updating while the operator sensor display is hidden.

**Replay a recorded cup trial (no sensors)** uses the current trial when available,
otherwise opens a chooser. Replay consumes saved inputs/calibration through the
shared processing core. It checks software consistency, not anatomical accuracy.

## Review, process and export

Saved cup trials are selected automatically. Raw BLE/USB/camera captures and
movement profiles publish their completed output to the same browser. Use
**Review current trial** to return to the raw cup trial after inspecting a derived
result. **Browse last outputs in Review** opens the most recent derived output.

| Review & export action | Result |
| --- | --- |
| Trial / calibration summary | Completeness, reference, cue and stream/frame counts |
| Plot / export image | Selected signals, optional EMG spectrum, optional PNG |
| Process EMG | New filtered samples, feature windows and `emg.processing.json` |
| Review EMG labels | Reviewed intervals beside raw data, with available video |
| Replay selected cup trial | Recorded control without hardware |
| Export recording bundle | Lossless ZIP with hashes and original status; choose the subject folder to include separate references |

Between trials, plotting, processing, single-trial export and recorded replay
leave the idle cup recorder connected. Wait for countdown/capture to finish first.
Use **Finish subject** before training, specialist/device tools or exporting an
entire subject folder that could receive more trials.

Forms start from the selected input and propose distinct output identities. Saved
processing results become the next selected input. Derived exports, diagnostics
and simulation telemetry default below the selected root's `derived/` folder;
raw trials stay under `SUBJECT/T…`, references under `SUBJECT/references/` and
movement captures under `movement/`. New training runs stay under
`outputs/training` with provenance. Bundled models and inherited data stay unchanged.

## Advanced Tools and recovery

**Advanced Tools** contains Movement profiles, EMG models, DMP / manual joints,
Raw capture, Diagnostics and Firmware. These use named forms, file choosers and
typed settings. Training and specialist checks are optional. Review participant
evaluation before explicit model selection; calibrated EMG amplitude remains
available. DMP fitting/export and playback retain recorded units/timestep;
angles-only inherited fits assume one second. Manual sliders command robot joints.

**Cancel** closes an unstarted form. **Stop active program** requests bounded
shutdown of a running action. Recording stop asks before interrupting collection.
Activity shows progress, errors and the selected output; full logs are in
`outputs/logs`. Failed attempts/results remain inspectable. Check status before
retrying a connection; a timed-out owner must release its device first.

Firmware **Build** and **Flash** are separate. Flash requires the explicit workflow,
supported toolchain and selected port. Normal launch never flashes hardware.
See [troubleshooting](troubleshooting-guide.md), [signal inspection](signal-inspection.md)
and [EMG evaluation](emg-learning.md) for focused references.

Hardware acceptance still requires checking real packet/sample loss, camera depth
pairing, raw video indexes, reconnect/shutdown, movement signs and mounting fits
for both arms, EMG clipping/grip and trials under slow inference. Use independent
angle references to evaluate anatomical accuracy; synthetic trajectories and replay
cannot establish it.
