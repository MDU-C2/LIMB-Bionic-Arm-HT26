# Majd
"""Concurrent child-process supervision for the AURORA desktop GUI."""

from __future__ import annotations

import contextlib
import json
import os
import queue
import re
import signal
import subprocess
import threading
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from tkinter import messagebox

from aurora.gui.project_support import REPOSITORY_ROOT, python_runtime_environment


@dataclass
class ManagedProcess:
    """One child process together with the UI state needed to supervise it."""

    key: str
    name: str
    kind: str
    process: subprocess.Popen[str]
    stopping: bool = False
    log_path: Path | None = None
    output_tail: list[str] = field(default_factory=list)
    stop_input: str | None = None
    output_path: Path | None = None


class ProcessManagerMixin:
    """Give a Tk widget safe supervision of multiple concurrent programs."""

    def _idle_cup_session(self):
        active = self._active_processes()
        return (
            bool(active)
            and all(item.key == "recording:cup_trials" for item in active)
            and self.session.cup.phase in {"ready", "rest"}
            and not self.session.cup.status["recording"]
        )

    def _initialize_process_manager(self) -> None:
        self.processes: dict[str, ManagedProcess] = {}
        self.log_queue: queue.Queue[tuple[str, object]] = queue.Queue(maxsize=512)
        self.process_exits: queue.Queue[tuple[str, object]] = queue.Queue(maxsize=32)
        self.log_overruns = 0
        self.reported_log_overruns = 0
        self._closing_processes = False

    def _queue_process_event(self, event):
        if event[0] in {"exit", "result", "cup_result"}:
            # Only the finished child's watcher waits here; acquisition never does.
            self.process_exits.put(event)
            return
        try:
            self.log_queue.put_nowait(event)
        except queue.Full:
            self.log_overruns += 1
            with contextlib.suppress(queue.Empty):
                self.log_queue.get_nowait()
            with contextlib.suppress(queue.Full):
                self.log_queue.put_nowait(event)

    def _active_processes(self, kind: str | None = None) -> list[ManagedProcess]:
        active = [
            managed
            for managed in self.processes.values()
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
        stop_input: str | None = None,
        output_path: Path | None = None,
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

        if kind in {"recording", "simulation", "firmware"}:

            def retry():
                return self._start_program(
                    name,
                    kind,
                    command,
                    working_directory=working_directory,
                    environment=environment,
                    parallel=parallel,
                    process_key=process_key,
                    stop_input=stop_input,
                    output_path=output_path,
                )

            stop_serial = getattr(self, "_stop_serial_dashboard", None)
            if stop_serial is not None and not stop_serial():
                release_serial = getattr(self, "_release_serial_for", None)
                if release_serial is not None:
                    return release_serial(retry)
                self._append_log(
                    "[USB monitor] Waiting for the port to close. Try again shortly.\n"
                )
                return False

        stop_ble = (
            getattr(self, "_stop_ble_dashboard", None)
            if kind in {"recording", "simulation", "firmware"}
            else None
        )
        if stop_ble is not None and not stop_ble():
            release_ble = getattr(self, "_release_ble_for", None)
            if release_ble is None:
                self._append_log(
                    "[BLE monitor] Waiting for the connection to close. Try again shortly.\n"
                )
                return False
            if release_ble(retry):
                return True
        if stop_ble is not None and hasattr(self, "ble_sensor_connection"):
            self.ble_sensor_connection.set(f"BLE live view paused while {name} runs")

        merged_environment = os.environ.copy()
        if environment:
            merged_environment.update(environment)
        merged_environment["PYTHONUTF8"] = "1"
        if stop_input is not None:
            merged_environment["AURORA_GUI_CONTROL"] = "1"
        merged_environment["PYTHONIOENCODING"] = "utf-8"
        merged_environment = python_runtime_environment(Path(command[0]), merged_environment)
        try:
            creation_flags = 0
            start_new_session = os.name != "nt"
            if os.name == "nt":
                creation_flags = subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
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
        managed.stop_input = stop_input
        managed.output_path = Path(output_path) if output_path else None
        log_name = re.sub(r"[^A-Za-z0-9_-]+", "_", name).strip("_")
        managed.log_path = (
            REPOSITORY_ROOT / "outputs/logs" / f"{datetime.now():%Y%m%d_%H%M%S_%f}_{log_name}.log"
        )
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
        if managed is None or managed.process.poll() is not None or managed.process.stdin is None:
            return False
        try:
            managed.process.stdin.write(value)
            managed.process.stdin.flush()
            return True
        except (BrokenPipeError, OSError, ValueError):
            return False

    def _watch_process(self, managed: ManagedProcess) -> None:
        """Wait for a child without touching Tk from the worker thread."""
        log = None
        try:
            if managed.log_path is not None:
                try:
                    managed.log_path.parent.mkdir(parents=True, exist_ok=True)
                    log = managed.log_path.open("w", encoding="utf-8", buffering=1)
                except OSError:
                    pass
            if managed.process.stdout is not None:
                for line in managed.process.stdout:
                    managed.output_tail.append(line.rstrip())
                    managed.output_tail = managed.output_tail[-60:]
                    if log is not None:
                        try:
                            log.write(line)
                        except OSError:
                            with contextlib.suppress(OSError):
                                log.close()
                            log = None
                    event = "result" if line.startswith("[AURORA] ") else "line"
                    if managed.key == "recording:cup_trials" and line.startswith("[CUP] "):
                        try:
                            if json.loads(line.removeprefix("[CUP] ")).get("event") in {
                                "saved",
                                "error",
                            }:
                                event = "cup_result"
                        except (ValueError, AttributeError):
                            pass
                    self._queue_process_event(
                        (
                            event,
                            (managed.key, managed.name, line),
                        )
                    )
        finally:
            if log is not None:
                with contextlib.suppress(OSError):
                    log.close()
        return_code = managed.process.wait()
        for stream in (managed.process.stdout, managed.process.stdin):
            if stream is not None:
                with contextlib.suppress(OSError):
                    stream.close()
        self._queue_process_event(
            ("exit", (managed.key, managed.process, managed.name, return_code))
        )

    def _drain_log_queue(self) -> None:
        """Move worker output and completion events into Tk widgets."""
        lost = getattr(self, "log_overruns", 0) - getattr(self, "reported_log_overruns", 0)
        if lost:
            self.reported_log_overruns = self.log_overruns
            self._append_log(
                f"[Activity] Skipped {lost} display messages. Check the child log files.\n"
            )
        events = []
        for source, limit in ((self.log_queue, 200), (self.process_exits, 32)):
            for _ in range(limit):
                try:
                    events.append(source.get_nowait())
                except queue.Empty:
                    break
        try:
            for event, value in events:
                if event in {"line", "result", "cup_result"}:
                    key, name, line = value
                    if event == "result":
                        try:
                            result = json.loads(line.removeprefix("[AURORA] "))
                            self.workflow.accept_result(
                                Path(result["path"]), result.get("kind", "recording")
                            )
                        except (ValueError, TypeError, KeyError, OSError) as error:
                            self._append_log(f"[{name}] Could not select saved output: {error}\n")
                        continue
                    if key == "recording:cup_trials" and str(line).startswith("[CUP] "):
                        self._handle_cup_event(str(line))
                        continue
                    self._append_log(f"[{name}] {line}")
                    managed = self.processes.get(key)
                    if (
                        managed is not None
                        and managed.kind == "recording"
                        and str(line).startswith("[CAPTURE] ")
                    ):
                        cue = str(line).removeprefix("[CAPTURE] ").strip()
                        if hasattr(self, "recording_grip_status"):
                            self.recording_grip_status.set(cue)
                        self._set_status(
                            f"{name}: {cue}",
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
                if managed.output_path is not None and managed.output_path.exists():
                    try:
                        self.workflow.accept_result(managed.output_path, managed.kind)
                    except (ValueError, OSError) as error:
                        self._append_log(f"[{name}] Could not select saved output: {error}\n")
                if key == "recording:cup_trials":
                    self.cup_state.reset()
                    self.cup_cue.set("PREPARE")
                    self._cup_pace_changed()
                    self.ble_sensor_connection.set("BLE: not connected")
                    self.cup_status.set(
                        "Session stopped. Saved captures remain available for review."
                        if stopped
                        else "Subject finished. Enter the next subject ID."
                        if return_code == 0
                        else "Recording stopped. Check the error in Activity."
                    )
                    self.session.notice.set(
                        "Connection closed. Saved references remain reviewable; reconnecting starts a new calibration scope."
                    )
                    self._refresh_cup_guidance()
                if hasattr(self, "workflow"):
                    self.workflow.action_status.set(
                        f"{name} {outcome}. "
                        + (
                            f"Output: {self.workflow.last_output.get()}"
                            if self.workflow.last_output.get()
                            else "See Activity for the result and errors."
                        )
                    )
                if return_code and not stopped:
                    self._show_process_failure(managed, return_code)
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
                if managed.kind == "firmware" and not self.serial_sensor_reader.running:
                    # Native USB commonly disappears and returns while the C3
                    # resets. Give Windows time to enumerate the refreshed port.
                    self.after(750, self._refresh_recording_ports)
        except queue.Empty:
            pass
        self.after(100, self._drain_log_queue)

    def _show_process_failure(self, managed: ManagedProcess, return_code: int) -> None:
        errors = [
            line
            for line in managed.output_tail
            if any(word in line.lower() for word in ("error", "failed", "unavailable", "could not"))
        ]
        detail = "\n".join(errors[-5:] or managed.output_tail[-8:])[-1800:]
        log = ""
        if managed.log_path is not None and managed.log_path.is_file():
            log = f"\n\nLog: {managed.log_path.relative_to(REPOSITORY_ROOT)}"
        messagebox.showerror(f"{managed.name} stopped", f"Exit code {return_code}\n\n{detail}{log}")

    def stop_program(self) -> None:
        """Gracefully stop every active program after one confirmation."""
        active = self._active_processes()
        if not active:
            stop_ble = getattr(self, "_stop_ble_dashboard", None)
            if stop_ble is not None:
                stop_ble()
                self._update_controls()
            return
        if not self._confirm_sensitive_stop(active):
            return
        self._stop_processes(active)

    def _stop_processes(self, processes: list[ManagedProcess]) -> None:
        """Request graceful stops and schedule a process-tree fallback."""
        for managed in processes:
            if managed.stopping or managed.process.poll() is not None:
                continue
            managed.stopping = True
            self._append_log(f"[Stopping {managed.name}]\n")
            cooperative = managed.stop_input is not None and self._send_process_input(
                managed.key, managed.stop_input
            )
            if not cooperative and not self._request_process_stop(managed.process):
                self._append_log(f"[{managed.name}] Could not send a stop signal.\n")
                continue
            self.after(
                4000,
                lambda process=managed.process: threading.Thread(
                    target=self._kill_process_tree_if_running, args=(process,), daemon=True
                ).start(),
            )
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
            with contextlib.suppress(OSError):
                process.kill()

    def _close_processes(self) -> bool:
        """Stop all children during window close; return false if cancelled."""
        active = self._active_processes()
        if not active:
            return True
        if not self._closing_processes:
            if not self._confirm_sensitive_stop(active):
                return False
            self._closing_processes = True
            self._stop_processes(active)
        self.after(100, self.close)
        return False
