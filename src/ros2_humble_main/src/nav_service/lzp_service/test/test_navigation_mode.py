#!/usr/bin/env python3
"""Pure contracts for persisted 2-D/3-D navigation selection."""

import json
from pathlib import Path
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.navigation_mode import (  # noqa: E402
    MAP_2D,
    TERRAIN_3D,
    NavigationModeStore,
    normalize_navigation_mode,
    uses_terrain,
)


class NavigationModeTests(unittest.TestCase):

    def test_mode_normalization_is_strict_and_semantic(self):
        self.assertEqual(normalize_navigation_mode('map_2d'), MAP_2D)
        self.assertEqual(normalize_navigation_mode(' 2D '), MAP_2D)
        self.assertEqual(normalize_navigation_mode('terrain_3d'), TERRAIN_3D)
        self.assertEqual(normalize_navigation_mode('3d'), TERRAIN_3D)
        self.assertFalse(uses_terrain(MAP_2D))
        self.assertTrue(uses_terrain(TERRAIN_3D))
        with self.assertRaises(ValueError):
            normalize_navigation_mode('normal')

    def test_store_defaults_and_persists_atomically(self):
        with self.subTest('fresh store'):
            from tempfile import TemporaryDirectory
            with TemporaryDirectory() as directory:
                path = Path(directory) / 'navigation_mode.json'
                store = NavigationModeStore(path)
                self.assertEqual(store.snapshot()['mode'], TERRAIN_3D)
                self.assertEqual(store.snapshot()['version'], 0)

                changed = store.set(MAP_2D, updated_by='settings-page')
                self.assertEqual(changed['mode'], MAP_2D)
                self.assertEqual(changed['version'], 1)
                self.assertEqual(changed['updated_by'], 'settings-page')
                self.assertFalse(list(path.parent.glob('*.tmp')))
                self.assertEqual(
                    json.loads(path.read_text('utf-8'))['mode'], MAP_2D)

                restored = NavigationModeStore(path).snapshot()
                self.assertEqual(restored['mode'], MAP_2D)
                self.assertEqual(restored['version'], 1)

    def test_setting_the_selected_mode_is_idempotent(self):
        from tempfile import TemporaryDirectory
        with TemporaryDirectory() as directory:
            store = NavigationModeStore(Path(directory) / 'navigation_mode.json')
            first = store.set(TERRAIN_3D, updated_by='settings-page')
            second = store.set(TERRAIN_3D, updated_by='settings-page')
            self.assertEqual(first['version'], 0)
            self.assertEqual(second['version'], 0)

    def test_speed_gears_persist_without_raising_legacy_speed(self):
        from tempfile import TemporaryDirectory
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'navigation_mode.json'
            self.assertEqual(NavigationModeStore(path).snapshot()['speed_mps'], 0.25)
            for speed in (0.15, 0.30, 0.45, 0.25):
                with self.subTest(speed=speed):
                    store = NavigationModeStore(path)
                    self.assertEqual(store.set_speed(speed), speed)
                    self.assertEqual(
                        NavigationModeStore(path).snapshot()['speed_mps'], speed)
            with self.assertRaises(ValueError):
                store.set_speed(0.50)
            self.assertEqual(NavigationModeStore(path).snapshot()['speed_mps'], 0.25)

    def test_corrupt_persisted_mode_fails_safe_to_terrain_mode(self):
        from tempfile import TemporaryDirectory
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'navigation_mode.json'
            path.write_text('{broken', encoding='utf-8')
            snapshot = NavigationModeStore(path).snapshot()
            self.assertEqual(snapshot['mode'], TERRAIN_3D)
            self.assertEqual(snapshot['version'], 0)
            self.assertTrue(snapshot['load_error'])


if __name__ == '__main__':
    unittest.main()
