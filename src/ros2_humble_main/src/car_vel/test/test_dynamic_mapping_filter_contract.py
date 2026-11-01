#!/usr/bin/env python3
"""Contracts for mapping-only temporal dynamic obstacle filtering."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[2]
FILTER = ROOT / 'mapper' / 'octomap_server2' / 'src' / 'mapping_cloud_filter.cpp'
OCTOMAP_CMAKE = ROOT / 'mapper' / 'octomap_server2' / 'CMakeLists.txt'
LIO_NODE = ROOT / 'lio' / 'small_point_lio' / 'src' / 'small_point_lio_node.cpp'
LIO_GRID = ROOT / 'lio' / 'small_point_lio' / 'src' / 'util' / 'pointcloud_mapping.cpp'
LIO_CONFIG = ROOT / 'lio' / 'small_point_lio' / 'config' / 'mid360.yaml'
LIO_CMAKE = ROOT / 'lio' / 'small_point_lio' / 'CMakeLists.txt'
LIO_LAUNCH = (
    ROOT / 'lio' / 'small_point_lio' / 'launch' /
    'small_point_lio.launch.py'
)
LAUNCH = ROOT / 'car_vel' / 'launch' / 'mapping.launch.py'
FRONTEND = ROOT / 'car_vel' / 'car_vel' / 'web_frontend' / 'mapping.html'
MAP_SAVE = (
    ROOT / 'nav_service' / 'lzp_service' / 'lzp_service' /
    'map_save_service.py'
)
LAUNCH_MANAGER = (
    ROOT / 'nav_service' / 'lzp_service' / 'lzp_service' /
    'launch_manager_service.py'
)


class DynamicMappingFilterContractTest(unittest.TestCase):

    def test_filter_uses_bounded_multiframe_voxel_confirmation(self):
        source = FILTER.read_text('utf-8')
        for token in (
            'static_confirmation_observations',
            'static_confirmation_seconds',
            'max_observation_gap_frames',
            'candidate_stale_frames',
            'confirmed_stale_frames',
            'max_tracked_voxels',
            'capacity_exceeded ? "capacity_exceeded"',
        ):
            self.assertIn(token, source)

    def test_filter_splits_static_and_dynamic_without_touching_navigation_input(self):
        source = FILTER.read_text('utf-8')
        launch = LAUNCH.read_text('utf-8')
        self.assertIn('"/cloud_mapping_dynamic"', source)
        self.assertIn(
            'publish_cloud(dynamic_cloud, message->header, dynamic_cloud_publisher_)',
            source)
        self.assertIn(
            'dynamic_cloud_publisher_->get_subscription_count() > 0U',
            source)
        self.assertIn("'input_cloud_topic': '/cloud_mapping_filtered'", launch)
        self.assertIn("'input_topic': '/cloud_registered'", launch)
        self.assertIn("'dynamic_output_topic': '/cloud_mapping_dynamic'", launch)

    def test_status_exposes_filter_counts_and_ratio(self):
        source = FILTER.read_text('utf-8')
        for field in (
            'input_points', 'static_points', 'dynamic_points',
            'candidate_voxels', 'confirmed_voxels',
            'filtered_ratio', 'capacity_exceeded',
            'dynamic_cloud_subscribed',
        ):
            self.assertIn(field, source)

    def test_install_contract_rejects_a_stale_mapping_filter_binary(self):
        cmake = OCTOMAP_CMAKE.read_text('utf-8')
        manager = LAUNCH_MANAGER.read_text('utf-8')
        self.assertIn('mapping_cloud_filter_modes_v3', cmake)
        self.assertIn('mapping_filter_contract.txt', cmake)
        self.assertIn(
            "MAPPING_FILTER_CONTRACT = 'mapping_cloud_filter_modes_v3'",
            manager)
        self.assertIn("'mapping_filter_contract.txt'", manager)
        self.assertIn("'mapping_dynamic_filter_mode', 'protected'", manager)
        self.assertIn("'dynamic_filter_mode:='", manager)

    def test_saved_pcd_requires_continuous_stability(self):
        node = LIO_NODE.read_text('utf-8')
        grid = LIO_GRID.read_text('utf-8')
        config = LIO_CONFIG.read_text('utf-8')
        for token in (
            'localization_pcd_max_observation_gap_frames',
            'localization_pcd_min_stable_span_frames',
        ):
            self.assertIn(token, node)
            self.assertIn(token + ':', config)
        self.assertIn('max_observation_gap_frames', grid)
        self.assertIn('min_stable_span_frames', grid)
        self.assertIn('reset_observations', grid)
        self.assertIn(
            'test_pointcloud_mapping', LIO_CMAKE.read_text('utf-8'))

    def test_saved_pcd_density_capacity_is_at_least_twice_legacy(self):
        """A mostly-surface map must have >=2x spatial sampling capacity."""
        config = LIO_CONFIG.read_text('utf-8')
        node = LIO_NODE.read_text('utf-8')

        def yaml_number(name):
            match = re.search(rf'^\s*{name}:\s*([0-9.]+)\s*$', config, re.M)
            self.assertIsNotNone(match, name)
            return float(match.group(1))

        voxel_size = yaml_number('localization_pcd_voxel_size')
        observations = yaml_number('localization_pcd_min_observations')
        stable_span = yaml_number('localization_pcd_min_stable_span_frames')
        surface_density_gain = (0.20 / voxel_size) ** 2

        self.assertGreaterEqual(surface_density_gain, 2.0)
        self.assertLessEqual(observations, 6)
        self.assertLessEqual(stable_span, 15)
        self.assertIn(
            f'declare_parameter<double>("localization_pcd_voxel_size", {voxel_size:.2f})',
            node)
        self.assertIn(
            f'declare_parameter<int64_t>("localization_pcd_min_observations", {int(observations)})',
            node)

        # Deterministic 10 m x 10 m surface replay: compare occupied XY cells
        # using the same floor-to-voxel rule as PointcloudMapping.
        side_samples = range(1000)
        legacy_axis = {int((index * 0.01) // 0.20) for index in side_samples}
        dense_axis = {int((index * 0.01) // voxel_size) for index in side_samples}
        replay_gain = (len(dense_axis) / len(legacy_axis)) ** 2
        self.assertGreaterEqual(replay_gain, 2.0)

    def test_ground_uses_bounded_fast_confirmation_in_both_mapping_chains(self):
        source = FILTER.read_text('utf-8')
        launch = LAUNCH.read_text('utf-8')
        node = LIO_NODE.read_text('utf-8')
        grid = LIO_GRID.read_text('utf-8')
        config = LIO_CONFIG.read_text('utf-8')

        octomap_tokens = (
            'ground_preservation_enabled',
            'ground_confirmation_observations',
            'ground_confirmation_seconds',
            'consecutive_ground_observations',
            'confirmed_as_ground',
            'ground_protected_voxels',
        )
        for token in octomap_tokens:
            self.assertIn(token, source)
        self.assertIn('entry.second.is_ground_candidate()', source)
        self.assertIn("'ground_preservation_enabled': True", launch)
        self.assertIn("'ground_confirmation_observations': 3", launch)

        lio_parameters = (
            'localization_pcd_ground_preservation_enabled',
            'localization_pcd_ground_cell_size',
            'localization_pcd_ground_max_height_above_envelope',
            'localization_pcd_ground_max_slope_deg',
            'localization_pcd_ground_min_neighbor_cells',
            'localization_pcd_ground_min_observations',
            'localization_pcd_ground_min_stable_span_frames',
        )
        for token in lio_parameters:
            self.assertIn(token, node)
            self.assertIn(token + ':', config)
        for token in (
            'classify_ground_candidates',
            'ground_observations',
            'is_ground_persistent',
            'ground_protected_voxels',
        ):
            self.assertIn(token, grid)

        ground_observations = re.search(
            r'^\s*localization_pcd_ground_min_observations:\s*(\d+)',
            config, re.M)
        normal_observations = re.search(
            r'^\s*localization_pcd_min_observations:\s*(\d+)',
            config, re.M)
        self.assertIsNotNone(ground_observations)
        self.assertIsNotNone(normal_observations)
        self.assertGreaterEqual(int(ground_observations.group(1)), 2)
        self.assertLess(
            int(ground_observations.group(1)),
            int(normal_observations.group(1)))

    def test_frontend_can_show_dynamic_cloud_and_filter_status(self):
        html = FRONTEND.read_text('utf-8')
        for token in (
            'id="btn-view-dynamic"',
            "subscribe('/cloud_mapping_dynamic'",
            "subscribe('/mapping_cloud_filter/status'",
            'filter.ground_protected_voxels',
            '动态障碍点云',
            '动态过滤',
        ):
            self.assertIn(token, html)

    def test_three_filter_modes_are_wired_through_both_mapping_chains(self):
        source = FILTER.read_text('utf-8')
        launch = LAUNCH.read_text('utf-8')
        lio_launch = LIO_LAUNCH.read_text('utf-8')
        node = LIO_NODE.read_text('utf-8')
        grid = LIO_GRID.read_text('utf-8')
        config = LIO_CONFIG.read_text('utf-8')

        for token in ('protected', 'bypass', 'strict'):
            self.assertIn(f'"{token}"', source)
            self.assertIn(token.capitalize(), grid)
        self.assertIn('dynamic_filter_mode', source)
        self.assertIn('filter_mode', source)
        self.assertIn('filter_mode_ == FilterMode::Bypass', source)
        self.assertIn('filter_mode_ == FilterMode::Protected', source)
        self.assertIn(
            '"dynamic_filter_mode", dynamic_filter_mode_name_', source)
        self.assertIn("default_value='protected'", launch)
        self.assertIn("LaunchConfiguration('dynamic_filter_mode')", launch)

        self.assertIn('localization_pcd_filter_mode', node)
        self.assertIn('localization_pcd_filter_mode: protected', config)
        self.assertIn('localization_pcd_filter_mode_arg', lio_launch)
        self.assertIn(
            "'localization_pcd_filter_mode': dynamic_filter_mode", launch)
        self.assertIn('PointcloudMappingFilterMode::Bypass', grid)
        self.assertIn('PointcloudMappingFilterMode::Protected', grid)
        self.assertIn('snapshot.filter_mode', node)
        self.assertIn(
            '"localization_pcd_filter_mode",', node)

    def test_mode_is_visible_in_saved_metadata_and_mapping_frontend(self):
        metadata = MAP_SAVE.read_text('utf-8')
        html = FRONTEND.read_text('utf-8')
        self.assertIn("'mode': str(lio_snapshot.get(", metadata)
        self.assertIn("'filter_mode', 'protected'", metadata)
        self.assertIn('filter.filter_mode', html)
        self.assertIn('旁路', html)
        self.assertIn('严格', html)
        self.assertIn('地面保护', html)


if __name__ == '__main__':
    unittest.main()
