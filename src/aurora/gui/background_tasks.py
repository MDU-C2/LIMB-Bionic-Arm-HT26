"""Bounded background jobs; only the Tk caller handles completed results."""

from __future__ import annotations

import queue
import threading
from dataclasses import dataclass, field


@dataclass
class Job:
    key: str
    work: object
    callback: object
    cancelled: threading.Event = field(default_factory=threading.Event)


class BackgroundTasks:
    def __init__(self):
        self.pending = queue.Queue(maxsize=8)
        self.completed = queue.Queue(maxsize=8)
        self.jobs = {}
        self.closed = threading.Event()
        for _ in range(2):
            threading.Thread(target=self._run, daemon=True).start()

    def submit(self, key, work, callback):
        """Replace a pending result for this key. Work may check cancellation."""
        if self.closed.is_set():
            return
        previous = self.jobs.get(key)
        if previous is not None:
            retained = []
            while True:
                try:
                    candidate = self.pending.get_nowait()
                except queue.Empty:
                    break
                if candidate is not previous and not candidate.cancelled.is_set():
                    retained.append(candidate)
            for candidate in retained:
                self.pending.put_nowait(candidate)
        job = Job(key, work, callback)
        try:
            self.pending.put_nowait(job)
        except queue.Full:
            callback(None, RuntimeError("Background jobs are busy; try again shortly"))
            return
        if previous is not None:
            previous.cancelled.set()
            previous.callback = None
        self.jobs[key] = job

    def _run(self):
        while not self.closed.is_set():
            try:
                job = self.pending.get(timeout=0.1)
            except queue.Empty:
                continue
            if job.cancelled.is_set():
                continue
            try:
                result, error = job.work(job.cancelled), None
            except Exception as failure:
                result, error = None, failure
            while not self.closed.is_set() and not job.cancelled.is_set():
                try:
                    self.completed.put((job, result, error), timeout=0.1)
                    break
                except queue.Full:
                    continue

    def drain(self):
        """Run callbacks on the UI thread, ignoring cancelled/stale results."""
        while True:
            try:
                job, result, error = self.completed.get_nowait()
            except queue.Empty:
                return
            if not job.cancelled.is_set() and self.jobs.get(job.key) is job:
                del self.jobs[job.key]
                job.work = None
                callback, job.callback = job.callback, None
                callback(result, error)

    def close(self):
        self.closed.set()
        for job in self.jobs.values():
            job.cancelled.set()
            # Drop Tk references on its owning thread, even if work is stuck.
            job.callback = None
        self.jobs.clear()

    def cancel(self, key):
        """Release a closed dialog's callback on the UI thread."""
        job = self.jobs.pop(key, None)
        if job is not None:
            job.cancelled.set()
            job.callback = None
