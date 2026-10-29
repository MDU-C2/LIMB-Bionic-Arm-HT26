# Majd, Christian
"""Shared GUI layout and theme."""

from __future__ import annotations

import contextlib
import tkinter as tk
from tkinter import ttk

from aurora.gui.project_support import (
    ACCENT,
    ACCENT_ACTIVE,
    BACKGROUND,
    CARD,
    ERROR,
    INK,
    MUTED,
    SUCCESS,
    TAB_DEFINITIONS,
)


class AppLayoutMixin:
    """Build the shared desktop layout and styles."""

    def _configure_styles(self) -> None:
        """Configure the common color, typography, and widget styles."""
        with contextlib.suppress(tk.TclError):
            self.style.theme_use("clam")
        self.window.configure(background=BACKGROUND)
        self.style.configure("App.TFrame", background=BACKGROUND)
        self.style.configure("Page.TFrame", background=BACKGROUND)
        self.style.configure("Card.TFrame", background=CARD, relief="solid", borderwidth=1)
        self.style.configure(
            "Header.TLabel", background=BACKGROUND, foreground=INK, font=("Segoe UI", 17, "bold")
        )
        self.style.configure(
            "HeaderSub.TLabel", background=BACKGROUND, foreground=MUTED, font=("Segoe UI", 10)
        )
        self.style.configure(
            "PageTitle.TLabel",
            background=BACKGROUND,
            foreground=INK,
            font=("Segoe UI", 13, "bold"),
        )
        self.style.configure(
            "PageSub.TLabel", background=BACKGROUND, foreground=MUTED, font=("Segoe UI", 10)
        )
        self.style.configure(
            "CardTitle.TLabel", background=CARD, foreground=INK, font=("Segoe UI", 11, "bold")
        )
        self.style.configure(
            "CardText.TLabel", background=CARD, foreground=MUTED, font=("Segoe UI", 9)
        )
        self.style.configure("Card.TLabel", background=CARD, foreground=INK)
        self.style.configure("Card.TCheckbutton", background=CARD, foreground=INK)
        self.style.map("Card.TCheckbutton", background=[("active", CARD)])
        self.style.configure(
            "Accent.TButton", foreground="white", background=ACCENT, padding=(12, 6)
        )
        self.style.map(
            "Accent.TButton",
            background=[("active", ACCENT_ACTIVE), ("disabled", "#a8b5df")],
        )
        self.style.configure("Secondary.TButton", padding=(10, 5))
        self.style.configure("Danger.TButton", foreground=ERROR, padding=(10, 5))
        self.style.configure("TNotebook", background=BACKGROUND, borderwidth=0)
        self.style.configure("TNotebook.Tab", padding=(16, 7), font=("Segoe UI", 10))
        self.style.map(
            "TNotebook.Tab",
            background=[("selected", CARD), ("!selected", "#e5e9f2")],
            foreground=[("selected", INK), ("!selected", MUTED)],
        )
        self.style.configure("Treeview", rowheight=27, font=("Segoe UI", 9))
        self.style.configure("Treeview.Heading", font=("Segoe UI", 9, "bold"))

    def _build_header(self) -> None:
        """Build the application title and global status indicator."""
        header = ttk.Frame(self, style="App.TFrame")
        header.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        header.columnconfigure(0, weight=1)
        ttk.Label(header, text="AURORA Control Center", style="Header.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            header,
            text="Connect, inspect, calibrate, record or control, then review",
            style="HeaderSub.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(2, 0))
        self.status_label = tk.Label(
            header,
            textvariable=self.status,
            bg="#e8f5ee",
            fg=SUCCESS,
            font=("Segoe UI", 9, "bold"),
            padx=12,
            pady=6,
        )
        self.status_label.grid(row=0, column=1, rowspan=2, sticky="e")
        ttk.Label(header, textvariable=self.ble_sensor_connection, style="HeaderSub.TLabel").grid(
            row=2, column=0, columnspan=2, sticky="w", pady=(5, 0)
        )

        ttk.Label(
            header, textvariable=self.session.notice, style="HeaderSub.TLabel", wraplength=900
        ).grid(row=3, column=0, columnspan=2, sticky="w", pady=(4, 0))

    def _build_notebook(self) -> None:
        """Create registered tabs in their declared order."""
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=1, column=0, sticky="nsew")
        self.tabs: dict[str, ttk.Frame] = {}
        for definition in TAB_DEFINITIONS:
            if definition.key in self.tabs:
                raise ValueError(f"Duplicate GUI tab key: {definition.key}")
            builder = getattr(self, definition.builder_name, None)
            if not callable(builder):
                raise ValueError(
                    f"GUI tab {definition.key!r} has no builder {definition.builder_name!r}"
                )
            container, page = self._new_tab(definition.title)
            self.tabs[definition.key] = container
            builder(page)

    def _new_tab(self, title: str) -> tuple[ttk.Frame, ttk.Frame]:
        """Create a scrollable notebook page for normal and high-DPI screens."""
        container = ttk.Frame(self.notebook, style="Page.TFrame")
        container.columnconfigure(0, weight=1)
        container.rowconfigure(0, weight=1)
        canvas = tk.Canvas(container, bg=BACKGROUND, highlightthickness=0)
        scrollbar = ttk.Scrollbar(container, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        page = ttk.Frame(canvas, style="Page.TFrame", padding=(14, 11))
        page_window = canvas.create_window((0, 0), window=page, anchor="nw")
        page.bind(
            "<Configure>",
            lambda _event: canvas.configure(scrollregion=canvas.bbox("all")),
        )
        canvas.bind(
            "<Configure>",
            lambda event: canvas.itemconfigure(page_window, width=event.width),
        )

        def scroll_page(event) -> None:
            direction = -1 if event.delta > 0 else 1
            canvas.yview_scroll(direction, "units")

        container.bind("<Enter>", lambda _event: canvas.bind_all("<MouseWheel>", scroll_page))
        container.bind("<Leave>", lambda _event: canvas.unbind_all("<MouseWheel>"))
        self.notebook.add(container, text=title)
        return container, page

    def _heading(self, tab: ttk.Frame, title: str, description: str) -> None:
        """Add a standard title and explanatory subtitle to a tab."""
        ttk.Label(tab, text=title, style="PageTitle.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(
            tab,
            text=description,
            style="PageSub.TLabel",
            wraplength=850,
            justify="left",
        ).grid(row=1, column=0, sticky="ew", pady=(2, 10))

    def _card(self, parent: ttk.Frame, row: int, title: str, description: str) -> ttk.Frame:
        """Create a standard bordered content card inside a tab."""
        card = ttk.Frame(parent, style="Card.TFrame", padding=12)
        card.grid(row=row, column=0, sticky="ew", pady=(0, 9))
        card.columnconfigure(0, weight=1)
        ttk.Label(card, text=title, style="CardTitle.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(
            card,
            text=description,
            style="CardText.TLabel",
            wraplength=820,
            justify="left",
        ).grid(row=1, column=0, sticky="ew", pady=(3, 12))
        return card
