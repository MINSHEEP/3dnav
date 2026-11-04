#!/usr/bin/env python3
"""Pure contracts for persisted and synchronized costmap inflation."""

import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.costmap_inflation import (  # noqa: E402
    DEFAULT_INFLATION_RADIUS_M,
    MAX_INFLATION_RADIUS_M,
    MIN_INFLATION_RADIUS_M,
    InflationRadiusStore,
    InflationSyncState,
    normalize_inflation_radius,
)


class InflationRadiusTests(unittest.TestCase):

    def test_radius_validation_is_finite_bounded_and_millimetre_stable(self):
        self.assertEqual(
            normalize_inflation_radius(MIN_INFLATION_RADIUS_M), 0.3)
        self.assertEqual(normalize_inflation_radius(0.5554), 0.555)
        self.assertEqual(
            normalize_inflation_radius(MAX_INFLATION_RADIUS_M), 1.5)
        for value in (
            MIN_INFLATION_RADIUS_M - 0.001,
            MAX_INFLATION_RADIUS_M + 0.001,
            math.nan,
            math.inf,
            -math.inf,
            None,
            'invalid',
            True,
        ):
            with self.subTest(value=value), self.assertRaises(ValueError):
                normalize_inflation_radius(value)

    def test_store_defaults_persists_atomically_and_is_idempotent(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'inflation_radius.json'
            store = InflationRadiusStore(path)
            self.assertEqual(
                store.snapshot()['radius_m'], DEFAULT_INFLATION_RADIUS_M)
            self.assertEqual(store.snapshot()['version'], 0)

            changed = store.set(0.8, updated_by='settings-page')
            self.assertEqual(changed['radius_m'], 0.8)
            self.assertEqual(changed['version'], 1)
            self.assertEqual(changed['updated_by'], 'settings-page')
            unchanged = store.set(0.8, updated_by='settings-page')
            self.assertEqual(unchanged['version'], 1)
            self.assertFalse(list(path.parent.glob('*.tmp')))
            self.assertEqual(
                json.loads(path.read_text('utf-8'))['radius_m'], 0.8)

            restored = InflationRadiusStore(path).snapshot()
            self.assertEqual(restored['radius_m'], 0.8)
            self.assertEqual(restored['version'], 1)

    def test_corrupt_or_unsafe_persisted_value_falls_back_to_current_default(self):
        for payload in ('{broken', '{"radius_m": 0.1}'):
            with self.subTest(payload=payload), TemporaryDirectory() as directory:
                path = Path(directory) / 'inflation_radius.json'
                path.write_text(payload, encoding='utf-8')
                snapshot = InflationRadiusStore(path).snapshot()
                self.assertEqual(
                    snapshot['radius_m'], DEFAULT_INFLATION_RADIUS_M)
                self.assertEqual(snapshot['version'], 0)
                self.assertTrue(snapshot['load_error'])

    def test_sync_requires_both_targets_and_ignores_stale_results(self):
        state = InflationSyncState({'radius_m': 0.7, 'version': 2})
        self.assertFalse(state.ready)
        self.assertTrue(state.needs('global'))
        self.assertTrue(state.mark_result(
            'global', radius_m=0.7, version=2, success=True))
        self.assertFalse(state.ready)
        self.assertTrue(state.mark_result(
            'local', radius_m=0.7, version=2, success=True))
        self.assertTrue(state.ready)

        state.configure({'radius_m': 0.9, 'version': 3})
        self.assertFalse(state.ready)
        self.assertFalse(state.mark_result(
            'global', radius_m=0.7, version=2, success=True))
        self.assertTrue(state.needs('global'))
        self.assertTrue(state.mark_result(
            'global', radius_m=0.9, version=3,
            success=False, reason='parameter rejected'))
        status = state.snapshot()
        self.assertFalse(status['applied'])
        self.assertIn('parameter rejected', status['targets']['global']['error'])


if __name__ == '__main__':
    unittest.main()
