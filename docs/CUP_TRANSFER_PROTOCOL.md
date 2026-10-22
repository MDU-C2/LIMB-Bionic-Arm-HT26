# Cup transfer protocol

Use the same empty cup, table, marker positions and camera position for everyone.
The table is 60 x 60 cm. Keep the camera in front, with both shoulders, hips,
the selected arm and all four markers visible. Use subject IDs rather than names.
Mark the table and chair positions with tape so the setup can be repeated.

## Explain the task

Say:

> We will record your arm movement and muscle activity while you move this cup.
> Start with your open hand at marker 1. Reach to the cup at 2, pick it up and
> place it in the circle at 3. Let go of the cup, then move your empty hand to 4.
> Keep your trunk still and use the same arm throughout.

The palm faces inward at 1; for the left arm this is toward the subject's right.
Keep the other arm relaxed beside the body so it does not hide the hips.

Demonstrate once. Let the subject practise before saving trials. Confirm that
they understand the recording and have agreed to take part.

## Mount and calibrate

1. Place the `0x6B` IMU on the upper arm and `0x6A` on the forearm. Secure both
   so they do not slip. Keep the selected sensor axes along the arm segments;
   use the same mounting and signed-axis settings for each subject.
2. Seat the subject upright. Their hand rests open at 1, thumb up, elbow
   bent to about 90 degrees and shoulder relaxed. They do not need to copy an exact angle.
3. Enter the subject ID and arm. Press **Connect camera + BLE**.
4. Say: “Keep your arm and body still.” Press **Align IMUs** and wait until
   the app enables relaxed EMG calibration.
5. Say: “Keep your hand open and relax the muscle.” Press **Relaxed EMG**.
6. Say: “When I press this, make a firm grip for three seconds, then relax.”
   Press **Firm grip**. Repeat this step if the app rejects the signal.

Repeat calibration if an electrode or IMU moves, or Bluetooth reconnects.
Do not record while calibration is still running.

## Record the three paces

Choose the pace in the app before each trial. Use the same task in all conditions.

| Pace | Say before “go” |
| --- | --- |
| Natural | “Move the cup at your own comfortable pace, as you normally would.” |
| Slow | “Do the same movement more slowly than your normal pace. Keep it continuous.” |
| Fast | “Do the same movement faster than your normal pace, while keeping control of the cup.” |

Say: “Hand at 1. Ready.” Press **Record** and wait for the recording status,
then say “Go”. Let the subject complete 1 → 2 → 3 → 4 without extra timing cues.
Say “finished” after the empty hand
reaches 4. Press **Stop and save**, or **Mark 4 + save**.

If a second operator is available, they can press **Mark 2** at grasp and
**Mark 3** at release. These are operator observations; review them against
the video before using them as training labels.

Start with three valid trials per pace and 20 seconds between trials. Rotate
the pace order across subjects: natural/slow/fast, slow/fast/natural,
fast/natural/slow. These are collection settings for this protocol; keep them
fixed once collection starts. Put the cup back at 2 and the hand at 1 before
the next trial. Do not record the reset movement as part of the transfer.

If the cup is dropped, the wrong marker is used or a sensor slips, keep that
trial and repeat it. Note the reason separately when reviewing the data.
Finish the subject before changing their ID.

The pace field records the instruction, not a measured speed. Older trials
without this field have unspecified pace. Check actual durations and tracking
quality during analysis. See the [recording guide](RECORDING_GUIDE.md) for files.

The four-marker sequence follows Oscar Ågren, *Personalized, Vision-Based Human-Like
Robotic Arm Control on Real-Time Embedded Systems* (2026), section 7.4.1.
This version uses three explicit pace conditions and manual trial endings.
