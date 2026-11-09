"""Write raw BLE notifications outside acquisition callbacks."""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass

from aurora.sensors.ble_protocol import BleRecorder
from aurora.sensors.clock_sync import ClockSynchronizer
from aurora.time_utils import utc_now


@dataclass(frozen=True)
class ReceivedPacket:
    sensor: str
    payload: bytes
    host_time: str
    host_monotonic_ns: int
    connection_generation: int
    clock_sync: object


class PacketWriter:
    """One file owner with bounded buffering and explicit overflow accounting."""

    def __init__(
        self,
        session,
        *,
        clock_sync,
        connection_generation,
        capacity=2048,
        sensors=None,
        legacy_emg_prefix=False,
        legacy_emg_fixed=False,
    ):
        options = {"legacy_emg_prefix": legacy_emg_prefix, "legacy_emg_fixed": legacy_emg_fixed}
        if sensors is not None:
            options["sensors"] = sensors
        self.recorder = BleRecorder(session, **options)
        self.queue = queue.Queue(maxsize=capacity)
        self.clock_sync = clock_sync
        self.connection_generation = connection_generation
        self.queue_overruns = 0
        self.queue_high_water = 0
        self.error = ""
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def handle(self, sensor, data, *, host_time=None, host_monotonic_ns=None):
        if self.stopping.is_set():
            raise RuntimeError("Recording writer is stopping")
        packet = ReceivedPacket(
            sensor,
            bytes(data),
            host_time or utc_now(),
            time.monotonic_ns() if host_monotonic_ns is None else host_monotonic_ns,
            self.connection_generation,
            self.clock_sync.snapshot()
            if isinstance(self.clock_sync, ClockSynchronizer)
            else self.clock_sync,
        )
        try:
            self.queue.put_nowait(packet)
            self.queue_high_water = max(self.queue_high_water, self.queue.qsize())
        except queue.Full:
            self.queue_overruns += 1
            self.error = "Raw BLE writer queue overflow; recording is incomplete"

    def flush(self, timeout=5.0):
        """Wait for already received packets when explicitly requested by a caller."""
        barrier = threading.Event()
        try:
            self.queue.put(barrier, timeout=timeout)
        except queue.Full as error:
            raise RuntimeError("Raw BLE writer flush queue timed out") from error
        if not barrier.wait(timeout):
            raise RuntimeError("Raw BLE writer flush timed out")

    def _run(self):
        epoch = None
        try:
            while not self.stopping.is_set() or not self.queue.empty():
                try:
                    packet = self.queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                if isinstance(packet, threading.Event):
                    for file in (self.recorder.raw_file, *self.recorder.files.values()):
                        if file is not None:
                            file.flush()
                    packet.set()
                    continue
                if epoch != packet.connection_generation:
                    self.recorder.reset_sequence_tracking()
                    epoch = packet.connection_generation
                self.recorder.connection_generation = epoch
                self.recorder.clock_sync = packet.clock_sync
                try:
                    self.recorder.handle(
                        packet.sensor,
                        packet.payload,
                        host_time=packet.host_time,
                        host_monotonic_ns=packet.host_monotonic_ns,
                    )
                except Exception as error:
                    self.error = f"Raw BLE write failed: {error}"
        finally:
            try:
                self.recorder.close()
            except Exception as error:
                self.error = f"Raw BLE close failed: {error}"

    def close(self, timeout=5.0):
        self.stopping.set()
        self.thread.join(timeout)
        if self.thread.is_alive():
            raise RuntimeError("Raw BLE writer did not finish; files remain owned by its worker")
        self.recorder.close()

    def summary(self):
        return {
            "packet_counts": dict(self.recorder.counts),
            "packet_health": self.recorder.packet_health(),
            "imu_units": self.recorder.imu_units,
            "write_error": self.error,
            "raw_packet_queue_overruns": self.queue_overruns,
            "raw_packet_queue_high_water": self.queue_high_water,
        }
