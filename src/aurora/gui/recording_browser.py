# Majd, Christian
"""Saved recording browser."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from aurora.gui.emg_tools import EmgToolsMixin
from aurora.gui.project_support import (
    RECORDING_EXTENSIONS,
    REPOSITORY_ROOT,
    display_path,
    project_path,
)


class RecordingBrowserMixin(EmgToolsMixin):
    """Browse, filter, and open saved recordings."""

    def _build_recordings_tab(self, tab: ttk.Frame) -> None:
        """Populate the searchable recording-file browser."""
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(3, weight=1)
        self._heading(
            tab,
            "Browse recordings",
            "Find captured video, sensor data, and saved trajectories without searching folders.",
        )
        toolbar = ttk.Frame(tab, style="Page.TFrame")
        toolbar.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        toolbar.columnconfigure(1, weight=1)
        ttk.Label(toolbar, text="Folder", style="PageSub.TLabel").grid(row=0, column=0, padx=(0, 7))
        ttk.Entry(toolbar, textvariable=self.recordings_root).grid(row=0, column=1, sticky="ew")
        ttk.Button(toolbar, text="Browse", command=self._browse_recordings_root).grid(
            row=0, column=2, padx=(8, 0)
        )
        ttk.Button(toolbar, text="Refresh", command=self._refresh_recordings).grid(
            row=0, column=3, padx=(8, 0)
        )
        ttk.Label(toolbar, text="Filter", style="PageSub.TLabel").grid(
            row=1, column=0, padx=(0, 7), pady=(8, 0)
        )
        ttk.Entry(toolbar, textvariable=self.recordings_filter).grid(
            row=1, column=1, columnspan=3, sticky="ew", pady=(8, 0)
        )

        tree_card = ttk.Frame(tab, style="Card.TFrame", padding=8)
        tree_card.grid(row=3, column=0, sticky="nsew")
        tree_card.columnconfigure(0, weight=1)
        tree_card.rowconfigure(0, weight=1)
        self.recordings_tree = ttk.Treeview(
            tree_card, columns=("type", "size", "modified"), selectmode="browse", height=7
        )
        for column, title in (
            ("#0", "File"),
            ("type", "Type"),
            ("size", "Size"),
            ("modified", "Modified"),
        ):
            self.recordings_tree.heading(column, text=title)
        self.recordings_tree.column("#0", width=470, minwidth=220)
        self.recordings_tree.column("type", width=90, anchor="center")
        self.recordings_tree.column("size", width=90, anchor="e")
        self.recordings_tree.column("modified", width=145, anchor="center")
        scrollbar = ttk.Scrollbar(tree_card, orient="vertical", command=self.recordings_tree.yview)
        self.recordings_tree.configure(yscrollcommand=scrollbar.set)
        self.recordings_tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.recordings_tree.bind("<Double-1>", lambda _event: self._open_selected_recording())

        bottom = ttk.Frame(tab, style="Page.TFrame")
        bottom.grid(row=4, column=0, sticky="ew", pady=(10, 0))
        bottom.columnconfigure(0, weight=1)
        self.recordings_status = ttk.Label(bottom, text="", style="PageSub.TLabel")
        self.recordings_status.grid(row=0, column=0, sticky="w")
        ttk.Button(bottom, text="Open folder", command=self._open_recordings_root).grid(
            row=0, column=1, padx=(8, 0)
        )
        ttk.Button(bottom, text="Plot / export image", command=self._plot_selected_recording).grid(
            row=0, column=2, padx=(8, 0)
        )
        ttk.Button(
            bottom,
            text="Open selected",
            style="Accent.TButton",
            command=self._open_selected_recording,
        ).grid(row=0, column=3, padx=(8, 0))
        ttk.Button(bottom, text="Process EMG", command=self._process_selected_emg).grid(
            row=1, column=2, pady=(8, 0)
        )
        ttk.Button(bottom, text="Review EMG labels", command=self._review_selected_emg).grid(
            row=1, column=3, padx=(8, 0), pady=(8, 0)
        )
        ttk.Button(
            bottom,
            text="EMG model tools (Advanced)",
            command=lambda: self.workflow.open_advanced("emg"),
        ).grid(row=2, column=3, padx=(8, 0), pady=(8, 0))
        ttk.Label(
            bottom,
            text="Plot the raw signals first. Processing saves separate files.",
            style="PageSub.TLabel",
        ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))
        self.workflow.build_review_actions(bottom)

    def _browse_recordings_root(self) -> None:
        current = project_path(self.recordings_root.get())
        initial = current if current.is_dir() else REPOSITORY_ROOT
        selected = filedialog.askdirectory(title="Select recordings folder", initialdir=initial)
        if selected:
            self.recordings_root.set(display_path(Path(selected)))
            self._refresh_recordings()

    def _refresh_recordings(self) -> None:
        root_value = self.recordings_root.get().strip()
        root = project_path(root_value) if root_value else None
        self.recordings_status.configure(text="Scanning recordings...")

        def scan(cancelled):
            files, metadata, error, truncated = [], {}, "", False
            exists = root is not None and root.is_dir()
            if exists:
                try:
                    for path in root.rglob("*"):
                        if cancelled.is_set():
                            return None
                        try:
                            if (
                                path.suffix.lower() not in RECORDING_EXTENSIONS
                                or not path.is_file()
                            ):
                                continue
                            stat = path.stat()
                        except OSError:
                            continue
                        files.append(path)
                        metadata[path] = (stat.st_size, stat.st_mtime)
                        if len(files) >= 2000:
                            truncated = True
                            break
                    files.sort(key=lambda path: metadata[path][1], reverse=True)
                except OSError as failure:
                    error = f"Could not read folder: {failure}"
            return files, metadata, error, truncated, exists

        def ready(result, error):
            if error is not None:
                self.recordings_scan_error = f"Could not scan folder: {error}"
            else:
                (
                    self.recording_files,
                    self.recording_metadata,
                    self.recordings_scan_error,
                    self.recordings_scan_truncated,
                    self._recording_root_exists,
                ) = result
            self._render_recordings()

        self.background_tasks.submit("recordings", scan, ready)

    def _schedule_recordings_filter(self) -> None:
        if self.recordings_filter_after is not None:
            self.after_cancel(self.recordings_filter_after)
        self.recordings_filter_after = self.after(180, self._render_scheduled_recordings)

    def _render_scheduled_recordings(self) -> None:
        self.recordings_filter_after = None
        self._render_recordings()

    def _render_recordings(self) -> None:
        if not hasattr(self, "recordings_tree"):
            return
        selected_paths = {
            self.recording_tree_paths[item]
            for item in self.recordings_tree.selection()
            if item in self.recording_tree_paths
        }
        self.recordings_tree.delete(*self.recordings_tree.get_children())
        self.recording_tree_paths.clear()
        root_value = self.recordings_root.get().strip()
        root = project_path(root_value) if root_value else None
        filter_text = self.recordings_filter.get().strip().casefold()
        visible = [
            path
            for path in self.recording_files
            if not filter_text or filter_text in display_path(path).casefold()
        ]
        for path in visible:
            try:
                relative = str(path.relative_to(root)) if root is not None else path.name
                size_bytes, modified_timestamp = self.recording_metadata[path]
                modified = datetime.fromtimestamp(modified_timestamp).strftime("%Y-%m-%d %H:%M")
            except (KeyError, OSError, ValueError):
                continue
            item = self.recordings_tree.insert(
                "",
                "end",
                text=relative,
                values=(path.suffix.lstrip(".").upper(), self._format_size(size_bytes), modified),
            )
            self.recording_tree_paths[item] = path
            if path in selected_paths:
                self.recordings_tree.selection_add(item)
        if self.recordings_scan_error:
            message = self.recordings_scan_error
        elif root is None:
            message = "Choose a recordings folder"
        elif not getattr(self, "_recording_root_exists", False):
            message = f"Folder does not exist yet: {display_path(root)}"
        elif not self.recording_files:
            message = "No supported recordings found"
        elif self.recordings_scan_truncated:
            message = "Showing the first 2,000 recording files; choose a narrower folder if needed"
        elif filter_text:
            message = f"Showing {len(visible)} of {len(self.recording_files)} files"
        else:
            message = f"{len(visible)} recording file(s)"
        self.recordings_status.configure(text=message)

    @staticmethod
    def _format_size(size: int) -> str:
        value = float(size)
        for unit in ("B", "KB", "MB", "GB"):
            if value < 1024 or unit == "GB":
                return f"{value:.0f} {unit}" if unit == "B" else f"{value:.1f} {unit}"
            value /= 1024
        return f"{size} B"

    def _open_selected_recording(self) -> None:
        selected = self.recordings_tree.selection()
        if not selected:
            messagebox.showinfo("Select a recording", "Choose a file from the list first.")
            return
        path = self.recording_tree_paths.get(selected[0])
        if path is not None:
            self._open_existing(path)

    def _plot_selected_recording(self) -> None:
        self.workflow.show_tool("plot")

    def _process_selected_emg(self) -> None:
        self.workflow.show_tool("process_emg")

    def _open_recordings_root(self) -> None:
        root_value = self.recordings_root.get().strip()
        if not root_value:
            messagebox.showerror("Folder required", "Choose a recording output folder.")
            return
        root = project_path(root_value)
        if not root.exists():
            try:
                root.mkdir(parents=True)
            except OSError as error:
                messagebox.showerror("Could not create folder", str(error))
                return
        self._open_existing(root)
