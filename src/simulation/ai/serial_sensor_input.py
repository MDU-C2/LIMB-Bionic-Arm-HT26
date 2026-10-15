# Majd, Christian
"""Legacy USB packet input for the live simulator."""

from __future__ import annotations

import contextlib
import queue
import threading
import time

from emg_protocol import extract_emg
from imu_protocol import ImuStreamDecoder, extract_imus


class SerialPacketReader:
    """Keep the newest ESP32 packet for live control."""

    def __init__(self, serial_module, port: str, baud: int) -> None:
        self.serial_module = serial_module
        self.port = port
        self.baud = baud
        self.running = False
        self.packets: queue.Queue[dict[str, object]] = queue.Queue(maxsize=1)
        self.thread: threading.Thread | None = None
        self.error = ""
        self.decoder = ImuStreamDecoder()

    def start(self) -> None:
        self.decoder.reset()
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def latest(self) -> dict[str, object] | None:
        try:
            return self.packets.get_nowait()
        except queue.Empty:
            return None

    def _read_loop(self) -> None:
        while self.running:
            try:
                with self.serial_module.Serial(self.port, self.baud, timeout=0.25) as device:
                    self.error = ""
                    while self.running:
                        raw = device.readline()
                        if not raw:
                            continue
                        packet = self.decoder.feed(raw.decode("utf-8", errors="ignore"))
                        if not isinstance(packet, dict):
                            continue
                        if not extract_imus(packet) and not extract_emg(packet).get("connected"):
                            continue
                        if self.packets.full():
                            with contextlib.suppress(queue.Empty):
                                self.packets.get_nowait()
                        self.packets.put_nowait(packet)
            except (self.serial_module.SerialException, OSError) as error:
                self.error = str(error)
                if self.running:
                    time.sleep(1.0)


