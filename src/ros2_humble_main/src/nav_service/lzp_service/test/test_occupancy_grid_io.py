#!/usr/bin/env python3
"""Regression tests for lossless frozen OccupancyGrid map saving."""

from pathlib import Path
import json
import sys
import tempfile
import unittest

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.occupancy_grid_io import (  # noqa: E402
    OccupancyGridSnapshotError,
    write_occupancy_grid_snapshot,
)


class OccupancyGridIoTests(unittest.TestCase):
    def test_origin_precision_and_trinary_pixels_are_preserved(self):
        snapshot = {
            'frame_id': 'odom',
            'stamp_ns': 123,
            'width': 2,
            'height': 2,
            'resolution': 0.05,
            'origin': [-13.5, -31.65, 0.0],
            # ROS rows run bottom-to-top.  PGM rows must run top-to-bottom.
            'data': np.array([-1, 0, 20, 65], dtype=np.int8),
        }

        with tempfile.TemporaryDirectory() as directory:
            geometry = write_occupancy_grid_snapshot(
                snapshot, Path(directory), 'ceshi7',
                free_threshold=0.196,
                occupied_threshold=0.65,
            )
            yaml_path = Path(directory) / 'ceshi7.yaml'
            pgm_path = Path(directory) / 'ceshi7.pgm'
            yaml_lines = dict(
                line.split(':', 1) for line in
                yaml_path.read_text('utf-8').splitlines()
            )
            saved_origin = json.loads(yaml_lines['origin'].strip())
            magic, width, height, maximum, payload = (
                pgm_path.read_bytes().split(None, 4)
            )
            self.assertEqual(
                (magic, width, height, maximum),
                (b'P5', b'2', b'2', b'255'),
            )
            pixels = np.frombuffer(payload, dtype=np.uint8).reshape(
                (2, 2)).tolist()

        self.assertEqual(saved_origin, [-13.5, -31.65, 0.0])
        self.assertEqual(geometry['origin'], [-13.5, -31.65, 0.0])
        self.assertEqual(geometry['width'], 2)
        self.assertEqual(geometry['height'], 2)
        self.assertEqual(pixels, [[254, 0], [205, 254]])

    def test_bad_data_length_creates_no_partial_files(self):
        snapshot = {
            'frame_id': 'odom',
            'stamp_ns': 123,
            'width': 2,
            'height': 2,
            'resolution': 0.05,
            'origin': [0.0, 0.0, 0.0],
            'data': np.array([0, 0, 0], dtype=np.int8),
        }

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            with self.assertRaisesRegex(
                    OccupancyGridSnapshotError, 'data length'):
                write_occupancy_grid_snapshot(
                    snapshot, output, 'broken',
                    free_threshold=0.196,
                    occupied_threshold=0.65,
                )
            self.assertEqual(list(output.iterdir()), [])

    def test_out_of_range_occupancy_value_is_rejected(self):
        snapshot = {
            'frame_id': 'odom',
            'stamp_ns': 123,
            'width': 1,
            'height': 1,
            'resolution': 0.05,
            'origin': [0.0, 0.0, 0.0],
            'data': np.array([101], dtype=np.int16),
        }
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(
                    OccupancyGridSnapshotError, 'outside'):
                write_occupancy_grid_snapshot(
                    snapshot, Path(directory), 'broken',
                    free_threshold=0.196,
                    occupied_threshold=0.65,
                )

    def test_unsigned_value_cannot_wrap_into_unknown(self):
        snapshot = {
            'frame_id': 'odom',
            'stamp_ns': 123,
            'width': 1,
            'height': 1,
            'resolution': 0.05,
            'origin': [0.0, 0.0, 0.0],
            'data': np.array([65535], dtype=np.uint16),
        }
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(
                    OccupancyGridSnapshotError, 'outside'):
                write_occupancy_grid_snapshot(
                    snapshot, Path(directory), 'broken',
                    free_threshold=0.196,
                    occupied_threshold=0.65,
                )

    def test_service_uses_one_frozen_snapshot_without_map_saver_process(self):
        source = (
            PACKAGE_ROOT / 'lzp_service' / 'map_save_service.py'
        ).read_text('utf-8')
        self.assertIn("'data': np.asarray(message.data", source)
        self.assertIn('write_occupancy_grid_snapshot(', source)
        self.assertNotIn("'map_saver_cli'", source)
        self.assertNotIn('subprocess.run(', source)


if __name__ == '__main__':
    unittest.main()
