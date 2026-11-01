"""Pure frame helpers for the M701 seven-in-one RS-485 sensor."""

from __future__ import annotations

import struct


READ_FUNCTION = 0x03
READ_ALL_START_ADDRESS = 0x0002
READ_ALL_REGISTER_COUNT = 7
READ_ALL_BYTE_COUNT = 14
READ_ALL_RESPONSE_LENGTH = 19


def modbus_crc16(data: bytes) -> int:
    """Return the Modbus RTU CRC value for *data*."""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def append_crc(data: bytes) -> bytes:
    """Append the wire-order Modbus CRC (low byte, then high byte)."""
    crc = modbus_crc16(data)
    return data + bytes((crc & 0xFF, crc >> 8))


def _validate_address(address: int) -> int:
    address = int(address)
    if not 0 <= address <= 7:
        raise ValueError('M701 address must be between 0 and 7')
    return address


def build_read_all_request(address: int) -> bytes:
    """Build the documented request for all seven M701 readings."""
    address = _validate_address(address)
    request = bytes((
        address,
        READ_FUNCTION,
        READ_ALL_START_ADDRESS >> 8,
        READ_ALL_START_ADDRESS & 0xFF,
        READ_ALL_REGISTER_COUNT >> 8,
        READ_ALL_REGISTER_COUNT & 0xFF,
    ))
    return append_crc(request)


def parse_read_all_response(frame: bytes, expected_address: int) -> dict:
    """Validate and decode one complete seven-value response frame."""
    expected_address = _validate_address(expected_address)
    frame = bytes(frame)
    if len(frame) != READ_ALL_RESPONSE_LENGTH:
        raise ValueError(
            f'wrong frame length: expected {READ_ALL_RESPONSE_LENGTH}, got {len(frame)}')
    if frame[0] != expected_address:
        raise ValueError(
            f'wrong address: expected {expected_address}, got {frame[0]}')
    if frame[1] != READ_FUNCTION:
        raise ValueError(
            f'wrong function: expected 0x03, got 0x{frame[1]:02X}')
    if frame[2] != READ_ALL_BYTE_COUNT:
        raise ValueError(
            f'wrong byte count: expected 14, got {frame[2]}')

    received_crc = frame[-2] | (frame[-1] << 8)
    expected_crc = modbus_crc16(frame[:-2])
    if received_crc != expected_crc:
        raise ValueError(
            f'CRC mismatch: expected 0x{expected_crc:04X}, got 0x{received_crc:04X}')

    data = frame[3:-2]
    unsigned = struct.unpack('>7H', data)
    def signed_magnitude(raw):
        # The M701 manual explicitly defines 0x8064 as -100 (-10.0 C),
        # so its sign bit is not standard two's-complement encoding.
        return -(raw & 0x7FFF) if raw & 0x8000 else raw

    temperature_raw = signed_magnitude(unsigned[5])
    humidity_raw = signed_magnitude(unsigned[6])
    return {
        'co2_ppm': unsigned[0],
        'formaldehyde_ug_m3': unsigned[1],
        'tvoc_ug_m3': unsigned[2],
        'pm25_ug_m3': unsigned[3],
        'pm10_ug_m3': unsigned[4],
        'temperature_c': temperature_raw / 10.0,
        'humidity_rh': humidity_raw / 10.0,
    }


def extract_read_all_response(buffer: bytes, expected_address: int):
    """Find a valid response in a buffer that may contain CH340 echo/noise."""
    expected_address = _validate_address(expected_address)
    buffer = bytes(buffer)
    header = bytes((expected_address, READ_FUNCTION, READ_ALL_BYTE_COUNT))
    for start in range(max(0, len(buffer) - READ_ALL_RESPONSE_LENGTH + 1)):
        if buffer[start:start + 3] != header:
            continue
        candidate = buffer[start:start + READ_ALL_RESPONSE_LENGTH]
        if len(candidate) < READ_ALL_RESPONSE_LENGTH:
            return None
        try:
            parse_read_all_response(candidate, expected_address)
        except ValueError:
            continue
        return candidate
    return None
