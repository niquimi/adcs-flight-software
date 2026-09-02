"""Shared SIL binary protocol matching SIL/cpp headers (little-endian, packed)."""

from __future__ import annotations

import struct
import zlib
from typing import Sequence

# Matches SIL/cpp/sensor_packet.h (14 floats → 56 bytes).
# Order: timestamp, gyro[3], mag[3], css[6] (±X ±Y ±Z raw), batteryLevel
SENSOR_FMT = "<14f"
SENSOR_PACKET_SIZE = struct.calcsize(SENSOR_FMT)

# Matches SIL/cpp/command_packets.h (all floats for ESP32-native layout).
COMMAND_HEALTH_CHECK = 0x53494C43  # "SILC"
PACKET_VERSION = 1
PACKET_RW_TORQUE_CMD = 1
PACKET_MTB_DIPOLE_CMD = 2
PACKET_FSW_STATUS = 3
PACKET_BOOT_CONFIG = 4
PACKET_TELECOMMAND = 5
PACKET_FSW_ATTITUDE = 6

# CommandHeader: uint32 + uint16 + uint16 + uint32 + float = 16 bytes
COMMAND_HEADER_FMT = "<IHHIf"
# Full command: header + float[3] + crc32 = 32 bytes
COMMAND_PACKET_FMT = COMMAND_HEADER_FMT + "3fI"
COMMAND_HEADER_SIZE = struct.calcsize(COMMAND_HEADER_FMT)
COMMAND_PACKET_SIZE = struct.calcsize(COMMAND_PACKET_FMT)

assert COMMAND_HEADER_SIZE == 16
assert COMMAND_PACKET_SIZE == 32
assert SENSOR_PACKET_SIZE == 56


def crc32(data: bytes) -> int:
    """CRC-32 matching SIL/cpp/crc32.h (zlib / Ethernet polynomial)."""
    return zlib.crc32(data) & 0xFFFFFFFF


def pack_boot_config(boot_standby_s: float) -> bytes:
    """32-byte plant→FSW config sent once after TCP connect (PACKET_BOOT_CONFIG)."""
    header = struct.pack(
        COMMAND_HEADER_FMT,
        COMMAND_HEALTH_CHECK,
        PACKET_VERSION,
        PACKET_BOOT_CONFIG,
        0,
        0.0,
    )
    payload = struct.pack("<3f", float(boot_standby_s), 0.0, 0.0)
    body = header + payload
    return body + struct.pack("<I", crc32(body))


def pack_telecommand(
    opcode: int,
    arg0: int = 0,
    arg1: int = 0,
    timestamp_s: float = 0.0,
    sequence: int = 0,
) -> bytes:
    """32-byte operator TC (PACKET_TELECOMMAND) for the :5558 console link."""
    header = struct.pack(
        COMMAND_HEADER_FMT,
        COMMAND_HEALTH_CHECK,
        PACKET_VERSION,
        PACKET_TELECOMMAND,
        int(sequence) & 0xFFFFFFFF,
        float(timestamp_s),
    )
    payload = struct.pack(
        "<BBB9x",
        int(opcode) & 0xFF,
        int(arg0) & 0xFF,
        int(arg1) & 0xFF,
    )
    body = header + payload
    return body + struct.pack("<I", crc32(body))


def pack_sensor_packet(
    timestamp_s: float,
    gyro: Sequence[float] | list[float],
    mag_nt: Sequence[float] | list[float],
    css: Sequence[float] | list[float],
    battery_level: float,
) -> bytes:
    css6 = list(css)[:6] + [0.0] * max(0, 6 - len(css))
    return struct.pack(
        SENSOR_FMT,
        float(timestamp_s),
        float(gyro[0]),
        float(gyro[1]),
        float(gyro[2]),
        float(mag_nt[0]),
        float(mag_nt[1]),
        float(mag_nt[2]),
        float(css6[0]),
        float(css6[1]),
        float(css6[2]),
        float(css6[3]),
        float(css6[4]),
        float(css6[5]),
        float(battery_level),
    )


def unpack_command_packet(data: bytes) -> dict:
    """Parse a 32-byte command packet. Raises ValueError on bad framing/CRC."""
    if len(data) != COMMAND_PACKET_SIZE:
        raise ValueError(f"Expected {COMMAND_PACKET_SIZE} bytes, got {len(data)}")

    health, version, packet_type, sequence, timestamp_s = struct.unpack_from(
        COMMAND_HEADER_FMT, data, 0
    )
    crc, = struct.unpack_from("<I", data, 28)
    payload_without_crc = data[:-4]
    expected_crc = crc32(payload_without_crc)
    if health != COMMAND_HEALTH_CHECK:
        raise ValueError(f"Bad healthCheck 0x{health:08X} (expected 0x{COMMAND_HEALTH_CHECK:08X})")
    if version != PACKET_VERSION:
        raise ValueError(f"Unsupported packet version {version}")
    if crc != expected_crc:
        raise ValueError(f"CRC mismatch: got 0x{crc:08X}, expected 0x{expected_crc:08X}")

    if packet_type == PACKET_FSW_STATUS:
        mode, flags, tmr_count, bias = struct.unpack_from("<BBHf", data, 16)
        if mode > 3:
            raise ValueError(f"Unknown FSW mode id {mode}")
        return {
            "packet_type": packet_type,
            "sequence": sequence,
            "timestamp_s": timestamp_s,
            "mode": mode,
            "flags": flags,
            "tmr_mismatch_count": tmr_count,
            "gyro_bias_degph": bias,
        }

    if packet_type not in (
        PACKET_RW_TORQUE_CMD,
        PACKET_MTB_DIPOLE_CMD,
        PACKET_FSW_ATTITUDE,
    ):
        raise ValueError(f"Unknown packet_type {packet_type}")

    v0, v1, v2 = struct.unpack_from("<3f", data, 16)
    return {
        "packet_type": packet_type,
        "sequence": sequence,
        "timestamp_s": timestamp_s,
        "values": (v0, v1, v2),
    }
