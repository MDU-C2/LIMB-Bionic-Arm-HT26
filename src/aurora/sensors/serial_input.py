# Majd, Christian
"""Legacy USB packet input for the live simulator."""

from __future__ import annotations

import contextlib
import queue
import threading
import time

from aurora.sensors.emg_protocol import extract_emg
from aurora.sensors.imu_protocol import ImuStreamDecoder, extract_imus


class SerialPacketReader:
    """Queue acquired ESP32 samples for one control consumer."""

    def __init__(self, serial_module, port: str, baud: int) -> None:
        self.serial_module = serial_module
        self.port = port
        self.baud = baud
        self.running = False
        self.packets: queue.Queue[dict[str, object]] = queue.Queue(maxsize=2048)
        self.queue_overruns = 0
        self.thread: threading.Thread | None = None
        self.error = ""
        self.decoder = ImuStreamDecoder()

    def start(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.drain()
        self.decoder.reset()
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            if self.thread.is_alive():
                raise RuntimeError("Serial worker did not stop; connection remains owned")

    def latest(self) -> dict[str, object] | None:
        try:
            return self.packets.get_nowait()
        except queue.Empty:
            return None

    def drain(self) -> list[dict[str, object]]:
        """Consume every queued sample once; report overload rather than hiding it."""
        packets = []
        while (packet := self.latest()) is not None:
            packets.append(packet)
        return packets

    def _read_loop(self) -> None:
        while self.running:
            try:
                with self.serial_module.Serial(self.port, self.baud, timeout=0.25) as device:
                    self.decoder.reset()
                    self.error = ""
                    while self.running:
                        raw = device.readline()
                        receipt_ns = time.monotonic_ns()
                        if not raw:
                            continue
                        packet = self.decoder.feed(
                            raw.decode("utf-8", errors="ignore"), host_monotonic_ns=receipt_ns
                        )
                        if not isinstance(packet, dict):
                            continue
                        packet["host_monotonic_ns"] = receipt_ns
                        packet["connection_generation"] = self.decoder.connection_generation
                        if not extract_imus(packet) and not extract_emg(packet).get("connected"):
                            continue
                        if self.packets.full():
                            self.queue_overruns += 1
                            self.error = "Serial control queue overflow; samples were lost"
                            with contextlib.suppress(queue.Empty):
                                self.packets.get_nowait()
                        self.packets.put_nowait(packet)
            except (self.serial_module.SerialException, OSError) as error:
                self.error = str(error)
                if self.running:
                    time.sleep(1.0)
