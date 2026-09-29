"""Map independent BLE device clocks onto one host monotonic timeline.

The exchange follows the same four timestamps used by NTP: host transmit ``t0``,
device receive/transmit ``t1``/``t2``, and host receive ``t3``.  Repeating the
exchange lets the mapper estimate both offset and oscillator drift.  Every
sensor node gets its own mapper, while all mapped values share the host's
``time.monotonic_ns()`` domain and can therefore be joined with camera data.
"""

from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import asdict, dataclass
import statistics
import struct
import time
from typing import Any


TIME_SYNC_UUID = "27011525-1212-efde-1523-785feabcd122"
SYNC_PROTOCOL_VERSION = 1
SYNC_REQUEST = struct.Struct("<HHIQ")
SYNC_RESPONSE = struct.Struct("<HHIQQQ")


@dataclass(frozen=True)
class ClockExchange:
    """One validated four-timestamp clock exchange."""

    exchange_id: int
    host_midpoint_ns: int
    device_midpoint_ns: int
    round_trip_ns: int


class ClockSynchronizer:
    """Robust affine mapping from device microseconds to host nanoseconds."""

    def __init__(self, *, max_samples: int = 64, max_round_trip_ms: float = 500.0):
        self.samples: deque[ClockExchange] = deque(maxlen=max_samples)
        self.max_round_trip_ns = int(max_round_trip_ms * 1_000_000)
        self._pending: dict[int, int] = {}
        self._next_exchange_id = 1
        self._reference_device_ns = 0.0
        self._reference_host_ns = 0.0
        self._rate = 1.0

    @property
    def ready(self) -> bool:
        return bool(self.samples)

    def make_request(self, host_tx_ns: int | None = None) -> tuple[int, bytes]:
        """Create a versioned request and retain its actual local send time."""
        if host_tx_ns is None:
            host_tx_ns = time.monotonic_ns()
        exchange_id = self._next_exchange_id
        self._next_exchange_id = (exchange_id + 1) & 0xFFFFFFFF or 1
        self._pending[exchange_id] = host_tx_ns
        if len(self._pending) > 128:
            self._pending.pop(next(iter(self._pending)))
        return exchange_id, SYNC_REQUEST.pack(
            SYNC_PROTOCOL_VERSION, 0, exchange_id, host_tx_ns
        )

    def cancel_request(self, exchange_id: int) -> None:
        self._pending.pop(exchange_id, None)

    def handle_response(
        self, payload: bytes | bytearray, host_rx_ns: int | None = None
    ) -> ClockExchange | None:
        """Validate a response, add its measurement, and update the fit."""
        if len(payload) != SYNC_RESPONSE.size:
            return None
        version, _flags, exchange_id, echoed_tx, device_rx_us, device_tx_us = (
            SYNC_RESPONSE.unpack(payload)
        )
        host_tx_ns = self._pending.pop(exchange_id, None)
        if (
            version != SYNC_PROTOCOL_VERSION
            or host_tx_ns is None
            or echoed_tx != host_tx_ns
            or device_tx_us < device_rx_us
        ):
            return None
        if host_rx_ns is None:
            host_rx_ns = time.monotonic_ns()
        host_elapsed_ns = host_rx_ns - host_tx_ns
        device_elapsed_ns = (device_tx_us - device_rx_us) * 1000
        round_trip_ns = host_elapsed_ns - device_elapsed_ns
        if (
            host_elapsed_ns < 0
            or round_trip_ns < 0
            or round_trip_ns > self.max_round_trip_ns
        ):
            return None

        exchange = ClockExchange(
            exchange_id=exchange_id,
            host_midpoint_ns=(host_tx_ns + host_rx_ns) // 2,
            device_midpoint_ns=(device_rx_us + device_tx_us) * 500,
            round_trip_ns=round_trip_ns,
        )
        self.samples.append(exchange)
        self._fit()
        return exchange

    def _fit(self) -> None:
        # Lowest-latency exchanges have the smallest uncertainty from path
        # asymmetry. Keep enough points across time to measure clock drift.
        selected = sorted(self.samples, key=lambda sample: sample.round_trip_ns)[
            : min(16, len(self.samples))
        ]
        device_points = [sample.device_midpoint_ns for sample in selected]
        host_points = [sample.host_midpoint_ns for sample in selected]
        offsets = [host - device for host, device in zip(host_points, device_points)]
        self._rate = 1.0
        reference_device = statistics.fmean(device_points)
        reference_host = reference_device + statistics.median(offsets)

        device_span = max(device_points) - min(device_points)
        if len(selected) >= 4 and device_span >= 1_000_000_000:
            mean_device = reference_device
            mean_host = statistics.fmean(host_points)
            variance = sum(
                (device - mean_device) ** 2 for device in device_points
            )
            covariance = sum(
                (device - mean_device) * (host - mean_host)
                for device, host in zip(device_points, host_points)
            )
            rate = covariance / variance if variance else 1.0
            # A larger result is almost certainly delay asymmetry rather than
            # crystal drift; keep the offset solution until more probes arrive.
            if 0.998 <= rate <= 1.002:
                self._rate = rate
                reference_host = mean_host

        self._reference_device_ns = reference_device
        self._reference_host_ns = reference_host

    def to_host_ns(self, device_time_us: int | str | None) -> int | None:
        """Convert one acquisition timestamp, or return None before first sync."""
        if not self.ready or not isinstance(device_time_us, int):
            return None
        device_ns = device_time_us * 1000
        return round(
            self._reference_host_ns
            + (device_ns - self._reference_device_ns) * self._rate
        )

    def summary(self) -> dict[str, Any]:
        """Return serializable diagnostics for the recording metadata."""
        if not self.ready:
            return {
                "available": False,
                "clock_domain": "host_monotonic_ns",
                "exchange_count": 0,
            }
        latest_device_ns = self.samples[-1].device_midpoint_ns
        mapped_latest = round(
            self._reference_host_ns
            + (latest_device_ns - self._reference_device_ns) * self._rate
        )
        minimum_round_trip_ns = min(
            sample.round_trip_ns for sample in self.samples
        )
        return {
            "available": True,
            "clock_domain": "host_monotonic_ns",
            "exchange_count": len(self.samples),
            "rate": self._rate,
            "drift_ppm": (self._rate - 1.0) * 1_000_000,
            "offset_ns_at_latest_exchange": mapped_latest - latest_device_ns,
            "minimum_round_trip_ns": minimum_round_trip_ns,
            "estimated_uncertainty_ns": minimum_round_trip_ns // 2,
            "latest_exchange": asdict(self.samples[-1]),
        }


class BleClockSyncClient:
    """Run initial and periodic clock probes over a Bleak connection."""

    def __init__(
        self,
        client: Any,
        synchronizer: ClockSynchronizer,
        *,
        initial_probes: int = 8,
        refresh_seconds: float = 5.0,
        response_timeout: float = 0.75,
    ) -> None:
        self.client = client
        self.synchronizer = synchronizer
        self.initial_probes = initial_probes
        self.refresh_seconds = refresh_seconds
        self.response_timeout = response_timeout
        self._waiters: dict[int, asyncio.Future[ClockExchange]] = {}
        self._maintenance: asyncio.Task[None] | None = None
        self._subscribed = False

    def _notification(self, _sender: Any, data: bytearray) -> None:
        exchange = self.synchronizer.handle_response(
            data, host_rx_ns=time.monotonic_ns()
        )
        if exchange is None:
            return
        waiter = self._waiters.get(exchange.exchange_id)
        if waiter is not None and not waiter.done():
            waiter.set_result(exchange)

    async def _probe(self) -> bool:
        exchange_id, request = self.synchronizer.make_request()
        waiter: asyncio.Future[ClockExchange] = (
            asyncio.get_running_loop().create_future()
        )
        self._waiters[exchange_id] = waiter
        try:
            await self.client.write_gatt_char(
                TIME_SYNC_UUID, request, response=True
            )
            await asyncio.wait_for(waiter, timeout=self.response_timeout)
            return True
        except asyncio.TimeoutError:
            return False
        finally:
            self._waiters.pop(exchange_id, None)
            self.synchronizer.cancel_request(exchange_id)

    async def start(self) -> bool:
        """Negotiate sync if the connected peripheral exposes the new UUID."""
        if not callable(getattr(self.client, "write_gatt_char", None)):
            return False
        try:
            await self.client.start_notify(TIME_SYNC_UUID, self._notification)
            self._subscribed = True
            for probe in range(self.initial_probes):
                await self._probe()
                if probe + 1 < self.initial_probes:
                    await asyncio.sleep(0.04)
        except Exception:
            await self.close()
            return False
        if not self.synchronizer.ready:
            await self.close()
            return False
        self._maintenance = asyncio.create_task(self._maintain())
        return True

    async def _maintain(self) -> None:
        try:
            while True:
                await asyncio.sleep(self.refresh_seconds)
                await self._probe()
        except asyncio.CancelledError:
            raise
        except Exception:
            # Saved samples remain usable with the last valid affine fit.
            return

    async def close(self) -> None:
        if self._maintenance is not None:
            self._maintenance.cancel()
            try:
                await self._maintenance
            except asyncio.CancelledError:
                pass
            self._maintenance = None
        if self._subscribed and callable(
            getattr(self.client, "stop_notify", None)
        ):
            try:
                await self.client.stop_notify(TIME_SYNC_UUID)
            except Exception:
                pass
        self._subscribed = False
