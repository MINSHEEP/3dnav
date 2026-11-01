#!/usr/bin/env python3
"""Regression tests for bounded localization PCD loading."""

from pathlib import Path
import struct
import sys
import tempfile


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.pcd_io import load_pcd_xyz  # noqa: E402


HEADER = (
    'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
    'COUNT 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA {mode}\n')


def test_binary_pcd_loads_complete_xyz():
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'map.pcd'
        path.write_bytes(
            HEADER.format(mode='binary').encode('ascii') +
            b''.join(struct.pack('<fff', *point) for point in (
                (1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0))))

        points = load_pcd_xyz(path)

        assert points.shape == (3, 3)
        assert points[-1].tolist() == [7.0, 8.0, 9.0]


def test_ascii_pcd_loads_complete_xyz():
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'map.pcd'
        path.write_text(
            HEADER.format(mode='ascii') +
            '1 2 3\n4 5 6\n7 8 9\n', encoding='ascii')

        points = load_pcd_xyz(path)

        assert points.shape == (3, 3)


def test_truncated_binary_pcd_is_rejected():
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'map.pcd'
        path.write_bytes(
            HEADER.format(mode='binary').encode('ascii') + b'broken')
        try:
            load_pcd_xyz(path)
        except ValueError as error:
            assert 'payload' in str(error)
        else:
            raise AssertionError('truncated PCD must be rejected')
