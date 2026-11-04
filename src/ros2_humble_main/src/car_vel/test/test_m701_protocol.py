#!/usr/bin/env python3
"""Protocol regression tests for the M701 RS-485 air-quality module."""

from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from car_vel.m701_protocol import (  # noqa: E402
    build_read_all_request,
    extract_read_all_response,
    parse_read_all_response,
)


class M701ProtocolTests(unittest.TestCase):

    def test_builds_documented_read_all_request_with_modbus_crc(self):
        self.assertEqual(
            build_read_all_request(1),
            bytes.fromhex('01 03 00 02 00 07 A5 C8'),
        )

    def test_decodes_all_seven_documented_values(self):
        frame = bytes.fromhex(
            '01 03 0E 01 E2 00 05 00 24 00 2D 00 38 01 31 02 86 A0 0C'
        )
        values = parse_read_all_response(frame, expected_address=1)
        self.assertEqual(values['co2_ppm'], 482)
        self.assertEqual(values['formaldehyde_ug_m3'], 5)
        self.assertEqual(values['tvoc_ug_m3'], 36)
        self.assertEqual(values['pm25_ug_m3'], 45)
        self.assertEqual(values['pm10_ug_m3'], 56)
        self.assertEqual(values['temperature_c'], 30.5)
        self.assertEqual(values['humidity_rh'], 64.6)

    def test_decodes_negative_temperature_as_signed_tenths(self):
        payload = bytes.fromhex(
            '01 03 0E 01 90 00 00 00 00 00 00 00 00 80 64 00 00'
        )
        from car_vel.m701_protocol import append_crc
        values = parse_read_all_response(append_crc(payload), 1)
        self.assertEqual(values['temperature_c'], -10.0)

    def test_rejects_bad_crc_and_wrong_byte_count(self):
        valid = bytes.fromhex(
            '01 03 0E 01 E2 00 05 00 24 00 2D 00 38 01 31 02 86 A0 0C'
        )
        with self.assertRaisesRegex(ValueError, 'CRC'):
            parse_read_all_response(valid[:-1] + b'\x00', 1)
        with self.assertRaisesRegex(ValueError, 'byte count'):
            parse_read_all_response(valid[:2] + b'\x0C' + valid[3:], 1)

    def test_extracts_response_after_ch340_echo_and_noise(self):
        request = bytes.fromhex('01 03 00 02 00 07 A5 C8')
        response = bytes.fromhex(
            '01 03 0E 01 E2 00 05 00 24 00 2D 00 38 01 31 02 86 A0 0C'
        )
        self.assertEqual(
            extract_read_all_response(b'\x99' + request + response, 1),
            response,
        )


if __name__ == '__main__':
    unittest.main()
