"""Local vector illustrations shared by operator and participant views."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from aurora.recording.cup_protocol import REFERENCE_STEPS

TITLES = {
    "STILL": "Hold the reference pose",
    "RAISE": "Raise the upper arm forward",
    "BEND": "Bend the elbow",
    "TURN": "Turn the forearm: palm down first",
    "RETURN": "Return to the first pose",
    "RELAX": "Relax the open hand",
    "GRIP": "Hold a firm, comfortable grip",
    "RELEASE": "Release and relax",
    "CUP": "Hand 1 → cup 2 → release 3 → empty hand 4",
}


def illustration_key(protocol):
    key = protocol.get("movement")
    if key in TITLES:
        return key
    index = protocol.get("reference_step")
    return (
        REFERENCE_STEPS[index][0]
        if type(index) is int and 0 <= index < len(REFERENCE_STEPS)
        else "CUP"
    )


def display_sign(side, *, camera_view=False, mirrored=False):
    """Participant's left is screen-left; a front camera reverses that once."""
    return (-1 if side == "left" else 1) * (-1 if camera_view else 1) * (-1 if mirrored else 1)


class GuidancePane(ttk.Frame):
    def __init__(self, parent, *, camera_view=False, mirrored=False, large=False):
        super().__init__(parent, padding=8)
        self.camera_view, self.mirrored = camera_view, mirrored
        self.snapshot, self.side = {}, "left"
        self.title = ttk.Label(self, font=("Segoe UI", 22 if large else 13, "bold"), wraplength=850)
        self.title.pack(anchor="w")
        self.canvas = tk.Canvas(self, width=500, height=260, bg="#f4f7fb", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.cue = ttk.Label(self, font=("Segoe UI", 28 if large else 17, "bold"))
        self.cue.pack(anchor="w")
        self.instruction = ttk.Label(self, wraplength=800, font=("Segoe UI", 16 if large else 11))
        self.instruction.pack(anchor="w", fill="x", pady=5)
        self.progress_label = ttk.Label(self)
        self.progress_label.pack(anchor="w")
        self.progress = ttk.Progressbar(self, maximum=1)
        self.progress.pack(fill="x")
        self.detail = ttk.Label(self, wraplength=850)
        self.detail.pack(anchor="w", pady=4)
        self.canvas.bind("<Configure>", lambda event: self.draw())
        self.bind("<Configure>", self._resize_text)

    def _resize_text(self, _event):
        width = max(240, self.winfo_width() - 24)
        for label in (self.title, self.cue, self.instruction, self.detail):
            label.configure(wraplength=width)

    def show(self, protocol, side="left", message=""):
        snapshot = dict(protocol)
        if (snapshot, side) == (self.snapshot, self.side) and self.detail.cget("text") == message:
            return
        self.snapshot, self.side = snapshot, side
        key = illustration_key(snapshot)
        self.title.configure(text=f"Your {side} arm · {TITLES[key]}")
        self.cue.configure(text=snapshot.get("cue", "PREPARE"))
        self.instruction.configure(
            text=snapshot.get("instruction", "Follow the operator. Begin on GO.")
        )
        self.progress["value"] = snapshot.get("progress", 0)
        self.progress_label.configure(
            text="Capture window progress; review marker completion afterwards."
            if key == "CUP"
            else "Step progress"
        )
        self.progress.configure(takefocus=False)
        self.detail.configure(text=message)
        self.draw()

    def draw(self):
        c = self.canvas
        c.delete("all")
        width, height = max(500, c.winfo_width()), max(260, c.winfo_height())
        sx, sy = width / 500, height / 260
        sign = display_sign(self.side, camera_view=self.camera_view, mirrored=self.mirrored)

        def point(x, y):
            return ((250 + sign * (x - 250)) * sx, y * sy)

        def line(points, **kwargs):
            c.create_line(*(v for p in points for v in point(*p)), **kwargs)

        def text(x, y, value, **kwargs):
            c.create_text(*point(x, y), text=value, font=("Segoe UI", 12), **kwargs)

        key = illustration_key(self.snapshot)
        if key == "CUP":
            # Schematic participant layout; marker 3 stays physically right of 2
            # for either selected arm. Only the displayed camera transform reverses it.
            def table_point(x, y):
                factor = (-1 if self.camera_view else 1) * (-1 if self.mirrored else 1)
                return ((250 + factor * (x - 250)) * sx, y * sy)

            c.create_rectangle(
                *table_point(55, 20), *table_point(445, 235), outline="#52647a", width=3
            )
            markers = [(115, 190), (180, 95), (365, 95), (295, 190)]
            for a, b in zip(markers, markers[1:], strict=False):
                c.create_line(
                    *table_point(*a), *table_point(*b), fill="#376dcc", width=3, arrow="last"
                )
            for number, (x, y) in enumerate(markers, 1):
                px, py = table_point(x, y)
                c.create_oval(
                    px - 20, py - 20, px + 20, py + 20, fill="#d9ecff", outline="#376dcc", width=2
                )
                c.create_text(
                    px,
                    py,
                    text=str(number),
                    font=("Segoe UI", 19, "bold"),
                    tags=(f"marker{number}",),
                )
            for x, y in ((180, 55), (365, 55)):
                c.create_polygon(
                    *(
                        v
                        for p in (
                            (x - 10, y - 14),
                            (x + 10, y - 14),
                            (x + 7, y + 8),
                            (x - 7, y + 8),
                        )
                        for v in table_point(*p)
                    ),
                    fill="white",
                    outline="#52647a",
                    width=2,
                )
                c.create_line(
                    *table_point(x + 10, y - 10),
                    *table_point(x + 17, y - 8),
                    *table_point(x + 16, y),
                    *table_point(x + 9, y + 2),
                    fill="#52647a",
                    width=2,
                )
            for x, y, label in (
                (115, 218, "Open hand"),
                (180, 126, "Pick up"),
                (365, 126, "Place / release"),
                (295, 218, "Empty hand"),
            ):
                c.create_text(*table_point(x, y), text=label, font=("Segoe UI", 11))
            text(250, 250, "60 × 60 cm · empty cup · schematic positions")
            return
        # A sagittal arm diagram: selected shoulder/upper arm, elbow and forearm.
        c.create_oval(*point(228, 18), *point(272, 62), outline="#52647a", width=3)
        line([(250, 65), (250, 220)], fill="#8d9aab", width=12)
        shoulder, elbow, wrist = (285, 85), (285, 165), (365, 165)
        line([(215, 85), (215, 210)], fill="#c1c9d3", width=8)
        line([shoulder, elbow, wrist], fill="#b8c5d8", width=9, dash=(4, 3))
        if key == "RAISE":
            elbow, wrist = (365, 85), (365, 15)
            line(
                [(310, 170), (368, 125), (397, 55)],
                fill="#d17d18",
                width=4,
                smooth=True,
                arrow="last",
            )
            text(135, 145, "Elbow stays\nabout 90°")
        elif key == "BEND":
            wrist = (330, 105)
            line(
                [(385, 165), (380, 120), (345, 95)],
                fill="#d17d18",
                width=4,
                smooth=True,
                arrow="last",
            )
            text(135, 145, "Upper arm\nstays still")
        elif key == "TURN":
            line(
                [(360, 133), (400, 137), (407, 180), (365, 190)],
                fill="#d17d18",
                width=4,
                smooth=True,
                arrow="last",
            )
            text(135, 140, "Palm down first,\nthen up")
        elif key == "RETURN":
            line(
                [(405, 90), (415, 140), (385, 160)],
                fill="#d17d18",
                width=4,
                smooth=True,
                arrow="last",
            )
            text(135, 140, "Thumb up\nHold still")
        else:
            text(
                130,
                125,
                "Forearm supported"
                if key in {"RELAX", "GRIP", "RELEASE"}
                else "Elbow about 90°\nThumb up · still",
            )
        line(
            [shoulder, elbow, wrist],
            fill="#376dcc",
            width=10,
            capstyle="round",
            tags=("active_arm",),
        )
        for x, y in (shoulder, elbow, wrist):
            px, py = point(x, y)
            c.create_oval(px - 6, py - 6, px + 6, py + 6, fill="#183f80", outline="")
        wx, wy = wrist
        if key == "GRIP":
            line(
                [
                    (wx + 8, wy - 25),
                    (wx + 40, wy - 25),
                    (wx + 35, wy + 18),
                    (wx + 12, wy + 18),
                    (wx + 8, wy - 25),
                ],
                fill="#52647a",
                width=3,
            )
            line(
                [(wx, wy), (wx + 28, wy - 10), (wx + 32, wy + 3), (wx + 8, wy + 10)],
                fill="#d17d18",
                width=5,
                smooth=True,
            )
        else:
            for offset in range(-2, 3):
                line([(wx + 4, wy), (wx + 28, wy + offset * 8)], fill="#376dcc", width=3)
        caption = "Side view; forward is toward the hand"
        text(250, 242, caption + " · illustration, not measured angles")


class GuidanceViews:
    """Both windows render the same snapshot; neither advances the protocol."""

    def __init__(self, parent):
        self.parent = parent
        self.operator = GuidancePane(parent, camera_view=True, mirrored=True)
        self.participant = self.window = None
        self.protocol, self.side, self.message = {}, "left", ""

    def update(self, protocol, side="left", message=""):
        self.protocol, self.side, self.message = dict(protocol), side, message
        self.operator.show(protocol, side, message)
        if self.window is not None and self.window.winfo_exists():
            self.participant.show(protocol, side)

    def open_participant(self):
        if self.window is not None and self.window.winfo_exists():
            self.window.lift()
            return
        self.window = tk.Toplevel(self.parent)
        self.window.title("AURORA | Participant guidance")
        self.window.geometry("1000x750")
        self.window.minsize(700, 650)
        self.participant = GuidancePane(self.window, large=True)
        self.participant.pack(fill="both", expand=True)
        ttk.Button(self.window, text="Full screen (F11)", command=self.toggle_fullscreen).pack(
            side="left", padx=12, pady=8
        )
        ttk.Button(self.window, text="Close participant view", command=self.window.destroy).pack(
            side="right", padx=12
        )
        self.window.bind("<F11>", lambda event: self.toggle_fullscreen())
        self.window.bind("<Escape>", lambda event: self.window.attributes("-fullscreen", False))
        self.update(self.protocol, self.side, self.message)

    def toggle_fullscreen(self):
        self.window.attributes("-fullscreen", not self.window.attributes("-fullscreen"))
