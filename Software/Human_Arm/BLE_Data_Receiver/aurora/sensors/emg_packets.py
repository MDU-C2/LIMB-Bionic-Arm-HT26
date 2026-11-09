"""Pure EMG wire decoding. Unknown layouts never become legacy samples."""

from __future__ import annotations

import struct
from dataclasses import dataclass

CURRENT_EMG_PACKET = struct.Struct("<HIQH")
HIGH_RATE_EMG_HEADER = struct.Struct("<HIQIHH")
LEGACY_EMG_PACKET = struct.Struct("<40HI")
TIMESTAMPED_EMG_PACKET = struct.Struct("<HIQ80H")
ADC_MAX = 4095


@dataclass(frozen=True)
class EmgPacket:
    sequence: int
    channels: tuple[tuple[int, ...], ...]
    first_time_us: int | None = None
    period_us: int | None = None
    offset: int = 0


def decode_emg(
    data: bytes, *, legacy_prefix: bool = False, legacy_fixed: bool = False
) -> EmgPacket:
    """Decode v1 and timestamped LIMB layouts; headerless legacy is opt-in.

    Headerless bytes cannot identify their protocol. A caller with known legacy
    provenance must select that adapter instead of probing arbitrary payloads.
    """
    if legacy_prefix and legacy_fixed:
        raise ValueError("Choose one legacy EMG layout")
    if legacy_fixed:
        if len(data) != LEGACY_EMG_PACKET.size:
            raise ValueError("invalid legacy fixed EMG packet")
        *values, sequence = LEGACY_EMG_PACKET.unpack(data)
        return EmgPacket(sequence, (tuple(values),))
    if legacy_prefix:
        if not (6 <= len(data) <= 84 and (len(data) - 4) % 2 == 0):
            raise ValueError("invalid legacy prefix EMG packet")
        sequence = int.from_bytes(data[:4], "little")
        return EmgPacket(sequence, (struct.unpack(f"<{(len(data) - 4) // 2}H", data[4:]),))
    if len(data) == CURRENT_EMG_PACKET.size:
        flags, sequence, device_us, value = CURRENT_EMG_PACKET.unpack(data)
        if flags >> 8 != 1 or flags & 0xFE:
            raise ValueError("unsupported current EMG flags/version")
        if flags & 1 and value > ADC_MAX:
            raise ValueError("Current EMG samples must be 12-bit ADC values")
        return EmgPacket(sequence, ((value,),) if flags & 1 else (), device_us)
    if len(data) == TIMESTAMPED_EMG_PACKET.size:
        flags, sequence, device_us, *values = TIMESTAMPED_EMG_PACKET.unpack(data)
        if flags not in (0, 1):
            raise ValueError("unsupported timestamped legacy EMG flags")
        return EmgPacket(sequence, (tuple(values[:40]), tuple(values[40:])), device_us)
    if len(data) >= HIGH_RATE_EMG_HEADER.size:
        flags, sequence, first_us, period_us, count, offset = HIGH_RATE_EMG_HEADER.unpack_from(data)
        shaped = period_us > 0 and 0 < count <= 40 and len(data) == 22 + 2 * count
        if flags >> 8 != 1 or flags & 0xFE:
            raise ValueError(
                f"unsupported high-rate EMG flags/version {flags:#06x}; select known legacy layouts explicitly"
            )
        if not shaped:
            raise ValueError("invalid high-rate EMG count, period or payload size")
        values = struct.unpack_from(f"<{count}H", data, HIGH_RATE_EMG_HEADER.size)
        if any(value > ADC_MAX for value in values):
            raise ValueError("Current EMG samples must be 12-bit ADC values")
        return EmgPacket(sequence, (values,), first_us, period_us, offset)
    raise ValueError("unknown EMG packet layout")
