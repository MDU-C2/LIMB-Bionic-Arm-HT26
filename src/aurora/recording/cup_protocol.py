# Majd
"""Keep preparation, movement and rest cues separate from movement labels."""

from __future__ import annotations

import math
import time

# Oscar's cup task uses a three-second preparation and eight-second capture.
# Extra rest and grip references belong to subject preparation, not every trial.
BASELINE_SECONDS = 0.0
COUNTDOWN_SECONDS = 3.0
TRIAL_SECONDS = 8.0
REST_SECONDS = 20.0
PACE_PROMPTS = {
    "natural": "Subject moves the cup at a comfortable, natural pace.",
    "slow": "Subject moves more slowly than their natural pace, in one continuous movement.",
    "fast": "Subject moves faster than their natural pace while keeping control of the cup.",
}
START_POSE = "Cup at 2. Subject's open hand at 1, thumb up. Other arm beside the body."
TASK_PROMPT = "Subject: hand 1 -> grasp cup at 2 -> place and release at 3 -> empty hand to 4."
RESET_PROMPT = "Return the cup to 2 and the subject's open hand to 1 before the next trial."

# The reference movements fit sensor-to-arm rotations. Trial cues remain
# separate from the subject's actual grasp/release times.
REFERENCE_STEPS = (
    (
        "STILL",
        5,
        "Upper arm beside the body, elbow about 90 degrees, thumb up. Subject stays still.",
    ),
    (
        "RAISE",
        10,
        "Elbow stays about 90 degrees and thumb up. Subject raises the arm forward first, "
        "then lowers it. Repeat slowly, clear of the table.",
    ),
    (
        "BEND",
        10,
        "Keep the upper arm still beside the body and thumb up. Subject bends the elbow "
        "further first, then returns. Repeat slowly, clear of the table.",
    ),
    (
        "TURN",
        10,
        "Elbow stays about 90 degrees beside the body. Subject turns the palm down first, "
        "then up, through a comfortable range. Repeat slowly.",
    ),
    (
        "RETURN",
        5,
        "Return to the first pose: upper arm beside body, elbow about 90 degrees, thumb up. Hold still.",
    ),
    ("RELAX", 5, "Forearm supported, hand open. Subject relaxes the muscle."),
    ("GRIP", 5, "Forearm supported. Subject holds a firm, comfortable grip around the empty cup."),
    ("RELEASE", 3, "Subject releases the cup and relaxes the open hand."),
)


class CupReferenceGuide:
    """Wait for the operator before timing each saved reference movement."""

    def __init__(self, *, clock=None, steps=REFERENCE_STEPS):
        self.clock = clock or time.monotonic_ns
        self.steps = steps
        self.phase = "reference_prepare"
        self.phase_started_ns = self.clock()
        self.step = 0
        self.completed = False
        self.go_ns = None
        self.practice = False
        self.condition = "reference"
        self.rest_until_ns = 0

    @property
    def active(self):
        return self.phase in ("reference_prepare", "reference_countdown", "reference")

    def start_step(self):
        if self.phase != "reference_prepare":
            raise ValueError("Wait for the current calibration step to finish")
        self.phase = "reference_countdown"
        self.phase_started_ns = self.clock()

    def tick(self):
        if not self.active or self.phase == "reference_prepare":
            return
        now = self.clock()
        duration = (
            COUNTDOWN_SECONDS if self.phase == "reference_countdown" else self.steps[self.step][1]
        )
        if self.active and (now - self.phase_started_ns) / 1e9 >= duration:
            self.phase_started_ns = now
            if self.phase == "reference_countdown":
                self.phase = "reference"
                return
            self.step += 1
            if self.step == len(self.steps):
                self.completed = True
                self.phase = "ready"
            else:
                self.phase = "reference_prepare"

    def finish(self):
        self.phase = "ready"

    def view(self):
        if not self.active:
            cue, duration, instruction = (
                "REFERENCE FINISHED" if self.completed else "REFERENCE CANCELLED",
                0,
                START_POSE,
            )
        else:
            cue, duration, instruction = self.steps[self.step]
        prompt = "Operator guides each step. Raw signals and video are saved."
        if self.phase == "reference_prepare":
            cue = f"{self.step + 1}/{len(self.steps)}: {cue}"
            prompt = "Read the instruction and position the subject. Press Start step when ready."
            remaining = 0
        elif self.phase == "reference_countdown":
            remaining = max(
                0, math.ceil(COUNTDOWN_SECONDS - (self.clock() - self.phase_started_ns) / 1e9)
            )
            cue = f"START IN {remaining}"
            prompt = "Hold the starting pose. Begin only when GO appears."
        else:
            remaining = max(0, math.ceil(duration - (self.clock() - self.phase_started_ns) / 1e9))
            if self.active:
                cue = f"GO: {cue} {remaining} s"
        return {
            "phase": self.phase,
            "movement": self.steps[self.step][0] if self.active else "RETURN",
            "duration_s": COUNTDOWN_SECONDS if self.phase == "reference_countdown" else duration,
            "progress": min(
                1,
                max(
                    0,
                    (self.clock() - self.phase_started_ns)
                    / 1e9
                    / (COUNTDOWN_SECONDS if self.phase == "reference_countdown" else duration),
                ),
            )
            if self.phase in ("reference", "reference_countdown") and duration
            else 0,
            "cue": cue,
            "instruction": instruction,
            "pace_instruction": prompt,
            "condition": "reference",
            "reference_step": self.step,
            "practice": False,
            "remaining_s": remaining,
            "elapsed_movement_s": 0,
        }


class CupTrialGuide:
    """Time operator cues; completion of the window does not label task success."""

    def __init__(
        self,
        *,
        baseline_s=BASELINE_SECONDS,
        countdown_s=COUNTDOWN_SECONDS,
        rest_s=REST_SECONDS,
        duration_s=TRIAL_SECONDS,
        clock=None,
    ):
        for value in (baseline_s, countdown_s, rest_s, duration_s):
            if not math.isfinite(value) or value < 0:
                raise ValueError("Recording cue durations must be finite and non-negative")
        self.baseline_s, self.countdown_s, self.rest_s = baseline_s, countdown_s, rest_s
        self.duration_s = duration_s
        self.clock = clock or time.monotonic_ns
        self.phase = "ready"
        self.phase_started_ns = 0
        self.go_ns = None
        self.rest_until_ns = 0
        self.practice = False
        self.condition = "natural"

    @property
    def active(self):
        return self.phase in ("baseline", "countdown", "movement", "practice")

    def begin(self, condition, *, practice=False):
        if condition not in PACE_PROMPTS:
            raise ValueError("Choose natural, slow or fast pace")
        if self.active or self.clock() < self.rest_until_ns:
            raise ValueError("Finish the current trial and rest before starting again")
        self.condition, self.practice = condition, practice
        self.phase = "baseline" if self.baseline_s else "countdown"
        self.phase_started_ns = self.clock()
        self.go_ns = None

    def tick(self):
        now = self.clock()
        elapsed = (now - self.phase_started_ns) / 1e9
        if self.phase == "baseline" and elapsed >= self.baseline_s:
            self.phase, self.phase_started_ns = "countdown", now
        elif self.phase == "countdown" and elapsed >= self.countdown_s:
            self.phase = "practice" if self.practice else "movement"
            self.phase_started_ns = self.go_ns = now
        elif self.phase == "rest" and now >= self.rest_until_ns:
            self.phase = "ready"

    def finish(self):
        now = self.clock()
        moved = self.go_ns is not None
        self.rest_until_ns = now + round(self.rest_s * 1e9) if moved else now
        self.phase = "rest" if now < self.rest_until_ns else "ready"
        self.phase_started_ns = now
        self.practice = False

    @property
    def window_finished(self):
        """The timer ends a recording window, without labelling the subject's action."""
        return bool(
            self.duration_s
            and self.go_ns is not None
            and (self.clock() - self.go_ns) / 1e9 >= self.duration_s
        )

    def view(self):
        now = self.clock()
        elapsed = (now - self.phase_started_ns) / 1e9
        remaining = 0
        if self.phase == "baseline":
            remaining = max(0, math.ceil(self.baseline_s - elapsed))
            title, instruction = "KEEP STILL", f"Subject rests at marker 1. Baseline: {remaining} s"
        elif self.phase == "countdown":
            remaining = max(0, math.ceil(self.countdown_s - elapsed))
            title, instruction = (
                str(remaining),
                "Subject stays at 1. Operator signals the start at GO.",
            )
        elif self.phase in ("movement", "practice"):
            title = "GO" if elapsed < 1 else "PRACTICE" if self.practice else "RECORDING"
            if self.duration_s:
                remaining = max(0, math.ceil(self.duration_s - elapsed))
                instruction = TASK_PROMPT + f" Capture ends automatically in {remaining} s."
            else:
                instruction = (
                    TASK_PROMPT + " Operator stops the manual recording after the movement."
                )
        elif self.phase == "rest":
            remaining = max(0, math.ceil((self.rest_until_ns - now) / 1e9))
            title, instruction = f"REST {remaining} s", RESET_PROMPT
        else:
            title, instruction = "READY", START_POSE
        duration = {
            "baseline": self.baseline_s,
            "countdown": self.countdown_s,
            "rest": self.rest_s,
            "movement": self.duration_s,
            "practice": self.duration_s,
        }.get(self.phase, 0)
        return {
            "phase": self.phase,
            "movement": "CUP",
            "duration_s": duration,
            "progress": min(1, max(0, elapsed / duration)) if duration else 0,
            "cue": title,
            "instruction": instruction,
            "pace_instruction": PACE_PROMPTS[self.condition],
            "condition": self.condition,
            "practice": self.practice,
            "remaining_s": remaining,
            "elapsed_movement_s": round((now - self.go_ns) / 1e9, 1)
            if self.go_ns is not None and self.phase in ("movement", "practice")
            else 0,
        }
