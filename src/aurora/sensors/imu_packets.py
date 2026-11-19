"""Pure current and fixed legacy IMU wire adapters."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

HIGH_RATE_IMU_PACKET = struct.Struct("<HIQQ12h")
TIMESTAMPED_IMU_PACKET = struct.Struct("<HIQ12h")
LEGACY_IMU_PACKET = struct.Struct("<9fI")


@dataclass(frozen=True)
class ImuPacket:
    sequence: int
    channels: tuple[tuple[tuple[float, ...], ...], ...]
    sample_times_us: tuple[int | None, ...]
    units: tuple[str, str]
    gyro_saturated_roles: tuple[int, ...] = ()


def decode_imu(data: bytes) -> ImuPacket:
    if len(data) in (HIGH_RATE_IMU_PACKET.size, TIMESTAMPED_IMU_PACKET.size):
        high_rate = len(data) == HIGH_RATE_IMU_PACKET.size
        if high_rate:
            flags, sequence, first_us, second_us, *values = HIGH_RATE_IMU_PACKET.unpack(data)
            times = (first_us, second_us)
        else:
            flags, sequence, device_us, *values = TIMESTAMPED_IMU_PACKET.unpack(data)
            times = (device_us, device_us)
        version = flags >> 8
        if version == 1:
            if flags & 0xFC:
                raise ValueError("unsupported IMU connected mask")
            mask = flags & 3
        elif not high_rate and flags in (0, 1):
            mask = 3
        else:
            raise ValueError(f"unsupported IMU flags/version {flags:#06x}")
        channels = []
        saturated = []
        for index in range(2):
            raw = values[index * 6 : index * 6 + 6]
            scale = (1000, 1000, 1000, 100, 100, 100) if high_rate else (1000,) * 6
            normalized = tuple(value / factor for value, factor in zip(raw, scale, strict=True))
            channels.append((normalized,) if mask & (1 << index) else ())
            if mask & (1 << index) and any(value in (-32768, 32767) for value in raw[3:6]):
                saturated.append(index)
        return ImuPacket(sequence, tuple(channels), times, ("g", "dps"), tuple(saturated))
    if len(data) == LEGACY_IMU_PACKET.size:
        *values, sequence = LEGACY_IMU_PACKET.unpack(data)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("nonfinite legacy IMU values")
        return ImuPacket(
            sequence, ((tuple(values),),), (None,), ("firmware units", "firmware units")
        )
    if 28 <= len(data) <= 244 and (len(data) - 4) % 24 == 0:
        sequence = int.from_bytes(data[:4], "little")
        values = struct.unpack(f"<{(len(data) - 4) // 4}f", data[4:])
        if not all(math.isfinite(value) for value in values):
            raise ValueError("nonfinite legacy IMU values")
        samples = tuple(tuple(values[index : index + 6]) for index in range(0, len(values), 6))
        return ImuPacket(sequence, (samples,), (None,), ("mg", "mdps"))
    raise ValueError("unknown IMU packet layout")
