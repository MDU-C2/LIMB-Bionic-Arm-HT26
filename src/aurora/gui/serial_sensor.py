# Majd, Christian
"""Background serial reader and packet parsing for the GUI sensor dashboard."""

from __future__ import annotations

import contextlib
import json
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any

SerialEvent = tuple[str, object]


@dataclass(frozen=True)
class SerialLine:
    text: str
    host_monotonic_ns: int
    connection_generation: int


def decode_sensor_packet(text: str) -> dict[str, Any] | None:
    """Decode one ESP32 JSON line, ignoring boot messages and malformed data."""
    try:
        value = json.loads(text.strip())
    except (json.JSONDecodeError, TypeError):
        return None
    return value if isinstance(value, dict) else None


class SerialSensorReader:
    """Continuously read one serial port without blocking Tk's event loop."""

    def __init__(self) -> None:
        self.events: queue.Queue[SerialEvent] = queue.Queue(maxsize=512)
        self.queue_overruns = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.port = ""
        self.baud = 0
        self.connection_generation = 0

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, port: str, baud: int) -> None:
        """Start reading, replacing a previous connection if necessary."""
        if not self.stop(timeout=0):
            raise RuntimeError("Previous serial connection is still stopping; retry shortly")
        try:
            while True:
                self.events.get_nowait()
        except queue.Empty:
            pass
        self.port = port
        self.baud = baud
        self._stop.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self, timeout=0.8) -> bool:
        """Request shutdown and briefly wait for the read timeout to expire."""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
            if thread.is_alive():
                return False
        self._thread = None
        return True

    def _emit(self, event):
        try:
            self.events.put_nowait(event)
        except queue.Full:
            self.queue_overruns += 1
            with contextlib.suppress(queue.Empty):
                self.events.get_nowait()
            self.events.put_nowait(event)

    def _read_loop(self) -> None:
        try:
            import serial
        except ImportError:
            self._emit(
                (
                    "fatal",
                    "Serial support is unavailable in the current Python runtime. "
                    "Run Setup explicitly, then open the app with Start.",
                )
            )
            return

        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.25) as device:
                    self.connection_generation += 1
                    self._emit(("connected", (self.port, self.baud, self.connection_generation)))
                    while not self._stop.is_set():
                        raw = device.readline()
                        receipt_ns = time.monotonic_ns()
                        if raw:
                            self._emit(
                                (
                                    "line",
                                    SerialLine(
                                        raw.decode("utf-8", errors="replace").strip(),
                                        receipt_ns,
                                        self.connection_generation,
                                    ),
                                )
                            )
            except (serial.SerialException, OSError) as error:
                if not self._stop.is_set():
                    self._emit(("error", str(error)))
                    self._stop.wait(2.0)

        self._emit(("stopped", self.port))
