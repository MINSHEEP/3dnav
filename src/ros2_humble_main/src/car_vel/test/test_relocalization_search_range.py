#!/usr/bin/env python3
"""Regression contract for the shared 3-D relocalization search window."""

from pathlib import Path
import unittest


SOURCE_ROOT = Path(__file__).resolve().parents[2]
CAR_VEL_ROOT = SOURCE_ROOT / 'car_vel'
ICP_ROOT = SOURCE_ROOT / 'registration' / 'icp_registration'


class RelocalizationSearchRangeTests(unittest.TestCase):

    def test_coordinator_and_navigation_launch_share_expanded_window(self):
        coordinator = (
            CAR_VEL_ROOT / 'car_vel' / 'relocalization_coordinator.py'
        ).read_text(encoding='utf-8')
        launch = (
            CAR_VEL_ROOT / 'launch' / 'car_vel.launch.py'
        ).read_text(encoding='utf-8')

        self.assertIn("self.declare_parameter('search_xy', 3.0)", coordinator)
        self.assertIn(
            "self.declare_parameter('search_yaw_deg', 90.0)", coordinator)
        self.assertIn("self._positive_float('search_xy', 3.0)", coordinator)
        self.assertIn(
            "self._positive_float('search_yaw_deg', 90.0)", coordinator)
        self.assertIn("'search_xy': 3.0", launch)
        self.assertIn("'search_yaw_deg': 90.0", launch)

    def test_icp_defaults_hard_limits_and_yaw_seeds_cover_new_window(self):
        config = (ICP_ROOT / 'config' / 'icp.yaml').read_text(
            encoding='utf-8')
        server = (ICP_ROOT / 'src' / 'icp_registration.cpp').read_text(
            encoding='utf-8')

        for setting in (
                'default_search_xy: 3.0',
                'default_search_yaw_deg: 90.0',
                'hard_max_search_xy: 3.0',
                'hard_max_search_yaw_deg: 90.0',
                'ndt_yaw_hypotheses: 5'):
            self.assertIn(setting, config)

        self.assertIn(
            'node_.declare_parameter<double>("default_search_xy", 3.0)',
            server)
        self.assertIn(
            'node_.declare_parameter<double>("default_search_yaw_deg", 90.0)',
            server)
        self.assertIn(
            'node_.declare_parameter<double>("hard_max_search_xy", 3.0)',
            server)
        self.assertIn(
            'node_.declare_parameter<double>("hard_max_search_yaw_deg", 90.0)',
            server)
        self.assertIn('double default_search_xy_{3.0};', server)
        self.assertIn(
            'double default_search_yaw_{degreesToRadians(90.0)};', server)
        self.assertIn('double hard_max_search_xy_{3.0};', server)
        self.assertIn(
            'double hard_max_search_yaw_{degreesToRadians(90.0)};', server)

    def test_matching_quality_gates_are_not_doubled(self):
        config = (ICP_ROOT / 'config' / 'icp.yaml').read_text(
            encoding='utf-8')

        for unchanged_gate in (
                'maximum_rmse: 0.20',
                'minimum_overlap: 0.55',
                'maximum_vertical_correction: 0.35',
                'maximum_tilt_deg: 8.0'):
            self.assertIn(unchanged_gate, config)

    def test_sampling_keeps_all_frames_and_point_count_gate(self):
        config = (ICP_ROOT / 'config' / 'icp.yaml').read_text(encoding='utf-8')
        for setting in ('accumulation_frames: 5', 'minimum_consistent_frames: 5',
                        'fine_leaf_size: 0.15', 'minimum_source_points: 700',
                        'minimum_rough_source_points: 250', 'source_max_range: 8.0',
                        'default_timeout_sec: 30.0', 'maximum_timeout_sec: 30.0'):
            self.assertIn(setting, config)
        coordinator = (CAR_VEL_ROOT / 'car_vel' / 'relocalization_coordinator.py').read_text(encoding='utf-8')
        launch = (CAR_VEL_ROOT / 'launch' / 'car_vel.launch.py').read_text(encoding='utf-8')
        self.assertIn("self.declare_parameter('relocalize_timeout_sec', 30.0)", coordinator)
        self.assertIn("'relocalize_timeout_sec': 30.0", launch)


if __name__ == '__main__':
    unittest.main()
