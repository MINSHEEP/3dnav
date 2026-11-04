#!/usr/bin/env python3
"""Regression tests for the browser-managed campus topology store."""

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.campus_topology_store import (  # noqa: E402
    CampusTopologyConflict,
    CampusTopologyStore,
    CampusTopologyValidationError,
)


def valid_topology():
    return {
        'version': 1,
        'automatic_navigation': {
            'enabled': True,
            'enabled_modes': ['rough_ground', 'obstacle', 'ramp'],
        },
        'capability_profile': {
            'max_up_step_m': 0.15,
            'max_down_step_m': 0.12,
            'max_up_slope_deg': 20.0,
            'max_down_slope_deg': 18.0,
            'max_cross_slope_deg': 12.0,
            'max_gap_width_m': 0.10,
            'max_roughness_m': 0.06,
            'min_clearance_width_m': 0.65,
            'min_confidence': 0.70,
            'normal_speed_mps': 0.25,
            'caution_speed_mps': 0.15,
            'special_speed_mps': 0.10,
        },
        'areas': [
            {'area_id': 'yard', 'map_name': 'yard', 'floor_id': 'ground',
             'environment': 'outdoor'},
            {'area_id': 'ramp_top', 'map_name': 'yard', 'floor_id': 'upper',
             'environment': 'outdoor'},
        ],
        'transitions': [{
            'transition_id': 'yard_ramp',
            'source_area': 'yard',
            'target_area': 'ramp_top',
            'distance_m': 3.0,
            'elevation_change_m': 0.5,
            'calibrated': True,
            'bidirectional': True,
            'source_pose': {'x': 1.0, 'y': 2.0, 'z': 0.0, 'yaw': 0.0},
            'target_pose': {'x': 4.0, 'y': 2.0, 'z': 0.5, 'yaw': 0.0},
            'terrain': {
                'terrain_type': 'ramp', 'slope_deg': 9.5,
                'cross_slope_deg': 1.0, 'step_height_m': 0.0,
                'gap_width_m': 0.0, 'roughness_m': 0.01,
                'clearance_width_m': 1.0, 'confidence': 0.95,
                'negative_obstacle': False, 'slippery': False,
            },
        }],
    }


class CampusTopologyStoreTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / 'campus_topology.json'
        self.store = CampusTopologyStore(self.path)

    def tearDown(self):
        self.temp.cleanup()

    def test_first_read_creates_empty_versioned_topology(self):
        value = self.store.load()
        self.assertEqual(value['revision'], 0)
        self.assertEqual(value['areas'], [])
        self.assertEqual(value['transitions'], [])
        self.assertTrue(value['automatic_navigation']['enabled'])
        self.assertNotIn(
            'stairs_up', value['automatic_navigation']['enabled_modes'])
        self.assertTrue(self.path.exists())

    def test_save_is_atomic_and_increments_revision(self):
        value = self.store.save(valid_topology(), expected_revision=0)
        self.assertEqual(value['revision'], 1)
        self.assertEqual(self.store.load()['transitions'][0]['transition_id'], 'yard_ramp')
        self.assertEqual(json.loads(self.path.read_text(encoding='utf-8'))['revision'], 1)

    def test_restricted_flat_site_is_off_by_default_and_requires_boolean(self):
        self.assertFalse(self.store.load()['automatic_navigation']['restricted_flat_site'])
        value = valid_topology()
        value['automatic_navigation']['restricted_flat_site'] = True
        self.store.save(value, expected_revision=0)
        self.assertTrue(self.store.load()['automatic_navigation']['restricted_flat_site'])
        value['automatic_navigation']['restricted_flat_site'] = 'true'
        with self.assertRaises(CampusTopologyValidationError):
            self.store.save(value, expected_revision=1)

    def test_stale_browser_revision_is_rejected(self):
        self.store.save(valid_topology(), expected_revision=0)
        with self.assertRaises(CampusTopologyConflict):
            self.store.save(valid_topology(), expected_revision=0)

    def test_calibrated_transition_requires_both_poses_and_confidence(self):
        value = valid_topology()
        value['transitions'][0].pop('target_pose')
        with self.assertRaises(CampusTopologyValidationError):
            self.store.save(value, expected_revision=0)
        value = valid_topology()
        value['transitions'][0]['terrain']['confidence'] = 0.2
        with self.assertRaises(CampusTopologyValidationError):
            self.store.save(value, expected_revision=0)

    def test_endpoint_yaw_uses_radians_and_rejects_degrees(self):
        value = valid_topology()
        value['transitions'][0]['source_pose']['yaw'] = 90.0
        with self.assertRaisesRegex(
                CampusTopologyValidationError, 'yaw must be in radians'):
            self.store.save(value, expected_revision=0)

    def test_same_map_connector_geometry_must_match_straight_execution(self):
        value = valid_topology()
        value['areas'][1]['map_name'] = 'yard'
        value['transitions'][0]['distance_m'] = 8.0
        with self.assertRaisesRegex(
                CampusTopologyValidationError, 'distance conflicts'):
            self.store.save(value, expected_revision=0)
        value = valid_topology()
        value['areas'][1]['map_name'] = 'yard'
        value['transitions'][0]['target_pose']['yaw'] = math.pi
        with self.assertRaisesRegex(
                CampusTopologyValidationError, 'must face through'):
            self.store.save(value, expected_revision=0)

    def test_duplicate_ids_and_dangling_area_references_are_rejected(self):
        value = valid_topology()
        value['areas'].append(dict(value['areas'][0]))
        with self.assertRaises(CampusTopologyValidationError):
            self.store.save(value, expected_revision=0)

    def test_automatic_navigation_rejects_stairs_and_unknown_modes(self):
        for invalid_mode in ('stairs_up', 'stairs_down', 'flight'):
            value = valid_topology()
            value['automatic_navigation']['enabled_modes'].append(
                invalid_mode)
            with self.assertRaisesRegex(
                    CampusTopologyValidationError,
                    'automatic terrain mode is not allowed'):
                self.store.save(value, expected_revision=0)
        value = valid_topology()
        value['transitions'][0]['target_area'] = 'missing'
        with self.assertRaises(CampusTopologyValidationError):
            self.store.save(value, expected_revision=0)


if __name__ == '__main__':
    unittest.main()
