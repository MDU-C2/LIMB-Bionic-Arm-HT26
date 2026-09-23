"""Concurrent child-process supervision for the AURORA desktop GUI."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
from tkinter import messagebox

from project_support import REPOSITORY_ROOT


@dataclass
class ManagedProcess:
    """One child process together with the UI state needed to supervise it."""

    key: str
    name: str
    kind: str
    process: subprocess.Popen[str]
    stopping: bool = False


class ProcessManagerMixin:
    """Give a Tk widget safe supervision of multiple concurrent programs."""

    def _initialize_process_manager(self) -> None:
        self.processes: dict[str, ManagedProcess] = {}
        self.log_queue: queue.Queue[tuple[str, object]] = queue.Queue()

    def _active_processes(self, kind: str | None = None) -> list[ManagedProcess]:
        active = [
            managed for managed in self.processes.values()
            if managed.process.poll() is None and (kind is None or managed.kind == kind)
        ]
        return sorted(active, key=lambda managed: managed.name)

    def _start_program(
        self,
        name: str,
        kind: str,
        command: list[str],
        *,
        working_directory: Path = REPOSITORY_ROOT,
        environment: dict[str, str] | None = None,
        parallel: bool = False,
        process_key: str | None = None,
    ) -> bool:
        """Start a managed child and stream labelled output to Activity."""
        active = self._active_processes()
        if active and not parallel:
            names = ", ".join(item.name for item in active)
            messagebox.showinfo("Program running", f"Stop the active program(s) first: {names}")
            return False

        key = process_key or f"{kind}:{name}"
        existing = self.processes.get(key)
        if existing is not None and existing.process.poll() is None:
            messagebox.showinfo("Program running", f"{existing.name} is already running.")
            return False

        merged_environment = os.environ.copy()
        if environment:
            merged_environment.update(environment)
        merged_environment["PYTHONUTF8"] = "1"
        merged_environment["PYTHONIOENCODING"] = "utf-8"
        try:
            creation_flags = 0
            start_new_session = os.name != "nt"
            if os.name == "nt":
                creation_flags = (
                    subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
                )
            process = subprocess.Popen(
                command,
                cwd=working_directory,
                env=merged_environment,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                creationflags=creation_flags,
                start_new_session=start_new_session,
            )
        except OSError as error:
            messagebox.showerror(f"Could not start {name}", str(error))
            self._set_status("Start failed", "error")
            return False

        managed = ManagedProcess(key, name, kind, process)
        self.processes[key] = managed
        count = len(self._active_processes())
        self._set_status(f"{count} program{'s' if count != 1 else ''} running", "running")
        self._append_log(f"\n[{name}] > {self._display_command(command)}\n")
        self._update_controls()
        self._refresh_info()
        threading.Thread(
            target=self._watch_process,
            args=(managed,),
            daemon=True,
        ).start()
        return True

    def _send_process_input(self, key: str, value: str) -> bool:
        """Write a short command to a running child process."""
        managed = self.processes.get(key)
        if (
            managed is None
            or managed.process.poll() is not None
            or managed.process.stdin is None
        ):
            return False
        try:
            managed.process.stdin.write(value)
            managed.process.stdin.flush()
            return True
        except (BrokenPipeError, OSError, ValueError):
            return False

    def _watch_process(self, managed: ManagedProcess) -> None:
        """Wait for a child without touching Tk from the worker thread."""
        if managed.process.stdout is not None:
            for line in managed.process.stdout:
                self.log_queue.put(("line", (managed.key, managed.name, line)))
        self.log_queue.put(
            ("exit", (managed.key, managed.process, managed.name, managed.process.wait()))
        )

    def _drain_log_queue(self) -> None:
        """Move worker output and completion events into Tk widgets."""
        try:
            while True:
                event, value = self.log_queue.get_nowait()
                if event == "line":
                    key, name, line = value
                    self._append_log(f"[{name}] {line}")
                    managed = self.processes.get(key)
                    if (
                        managed is not None
                        and managed.kind == "recording"
                        and str(line).startswith("[CAPTURE] ")
                    ):
                        self._set_status(
                            f"{name}: {str(line).removeprefix('[CAPTURE] ').strip()}",
                            "running",
                        )
                    continue

                key, process, name, return_code = value
                managed = self.processes.get(key)
                if managed is None or managed.process is not process:
                    continue
                stopped = managed.stopping
                del self.processes[key]
                if stopped:
                    outcome, state = "stopped", "ready"
                elif return_code == 0:
                    outcome, state = "finished", "ready"
                else:
                    outcome, state = f"failed (code {return_code})", "error"
                self._append_log(f"[{name} {outcome}]\n")
                active = self._active_processes()
                if active:
                    self._set_status(
                        f"{len(active)} program{'s' if len(active) != 1 else ''} running",
                        "running",
                    )
                else:
                    self._set_status(f"{name} {outcome}", state)
                self._update_controls()
                self._refresh_recordings()
                self._refresh_info()
        except queue.Empty:
            pass
        self.after(100, self._drain_log_queue)

    def stop_program(self) -> None:
        """Gracefully stop every active program after one confirmation."""
        active = self._active_processes()
        if not active or not self._confirm_sensitive_stop(active):
            return
        self._stop_processes(active)

    def _stop_processes(self, processes: list[ManagedProcess]) -> None:
        """Request graceful stops and schedule a process-tree fallback."""
        for managed in processes:
            if managed.stopping or managed.process.poll() is not None:
                continue
            managed.stopping = True
            self._append_log(f"[Stopping {managed.name}]\n")
            if not self._request_process_stop(managed.process):
                self._append_log(f"[{managed.name}] Could not send a stop signal.\n")
                continue
            self.after(4000, lambda process=managed.process: self._kill_process_tree_if_running(process))
        self._set_status("Stopping active programs", "running")
        self._update_controls()

    def _confirm_sensitive_stop(self, processes: list[ManagedProcess]) -> bool:
        """Confirm stops that can leave hardware or files incomplete."""
        recordings = [item for item in processes if item.kind == "recording"]
        if recordings:
            return messagebox.askyesno(
                "Stop recordings",
                f"Stop {len(recordings)} recording source(s)? They will have a few seconds "
                "to finish writing their files.",
            )
        if any(item.name == "Firmware flash" for item in processes):
            return messagebox.askyesno(
                "Stop firmware flash",
                "Interrupting a flash can leave the controller without working firmware. "
                "Stop anyway?",
            )
        return True

    @staticmethod
    def _request_process_stop(process: subprocess.Popen[str]) -> bool:
        """Send the platform's graceful termination signal to a child."""
        try:
            if os.name == "nt":
                process.send_signal(signal.CTRL_BREAK_EVENT)
            else:
                os.killpg(process.pid, signal.SIGINT)
            return True
        except (OSError, ValueError):
            try:
                process.terminate()
                return True
            except OSError:
                return False

    @staticmethod
    def _kill_process_tree_if_running(process: subprocess.Popen[str]) -> None:
        """Force-stop a child and descendants after the grace period."""
        if process.poll() is not None:
            return
        try:
            if os.name == "nt":
                subprocess.run(
                    ["taskkill", "/PID", str(process.pid), "/T", "/F"],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=6,
                    check=False,
                    creationflags=subprocess.CREATE_NO_WINDOW,
                )
            else:
                os.killpg(process.pid, signal.SIGKILL)
        except (OSError, subprocess.TimeoutExpired):
            try:
                process.kill()
            except OSError:
                pass

    def _close_processes(self) -> bool:
        """Stop all children during window close; return false if cancelled."""
        active = self._active_processes()
        if not active:
            return True
        if not self._confirm_sensitive_stop(active):
            return False
        for managed in active:
            managed.stopping = True
            self._request_process_stop(managed.process)
        for managed in active:
            try:
                managed.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self._kill_process_tree_if_running(managed.process)
        return True
