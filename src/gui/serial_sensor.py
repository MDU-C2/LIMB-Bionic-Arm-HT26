"""Background serial reader and packet parsing for the GUI sensor dashboard."""

from __future__ import annotations

import json
import queue
import threading
from typing import Any


SerialEvent = tuple[str, object]


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
        self.events: queue.Queue[SerialEvent] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.port = ""
        self.baud = 0

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, port: str, baud: int) -> None:
        """Start reading, replacing a previous connection if necessary."""
        self.stop()
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

    def stop(self) -> None:
        """Request shutdown and briefly wait for the read timeout to expire."""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.8)
        self._thread = None

    def _read_loop(self) -> None:
        try:
            import serial
        except ImportError:
            self.events.put(
                (
                    "fatal",
                    "pyserial is missing. Start the GUI with the aurora-simulation environment.",
                )
            )
            return

        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.25) as device:
                    self.events.put(("connected", (self.port, self.baud)))
                    while not self._stop.is_set():
                        raw = device.readline()
                        if raw:
                            self.events.put(
                                ("line", raw.decode("utf-8", errors="replace").strip())
                            )
            except (serial.SerialException, OSError) as error:
                if not self._stop.is_set():
                    self.events.put(("error", str(error)))
                    self._stop.wait(2.0)

        self.events.put(("stopped", self.port))
