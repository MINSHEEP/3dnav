#!/usr/bin/env python3
"""Contracts for starting Nav2 without a persistent default map."""

from pathlib import Path
import re
import unittest


ROS_ROOT = Path(__file__).resolve().parents[2]
NAVIGATION_ROOT = ROS_ROOT / 'navigation' / 'robot_navigation2'
NAV2_PARAMS = NAVIGATION_ROOT / 'config' / 'nav2_params.yaml'
NAVIGATION_LAUNCH = NAVIGATION_ROOT / 'launch' / 'navigation.launch.py'
NAVIGATION_WRAPPER = NAVIGATION_ROOT / 'launch' / 'navigation2.launch.py'
MAP_SWITCHER = (
    ROS_ROOT / 'nav_service' / 'lzp_service' / 'lzp_service'
    / 'map_switcher_service.py'
)
INSPECTION_PAGE = (
    ROS_ROOT / 'car_vel' / 'car_vel' / 'web_frontend' / 'inspection.html'
)


class DeferredMapLoadingContractTests(unittest.TestCase):

    def test_map_server_has_no_persistent_default_map(self):
        source = NAV2_PARAMS.read_text(encoding='utf-8')
        map_server = source.split('map_server:', 1)[1].split(
            'map_saver:', 1)[0]

        self.assertRegex(map_server, r'(?m)^\s+yaml_filename:\s*["\']{2}\s*$')
        self.assertNotRegex(map_server, r'(?m)^\s+yaml_filename:\s*/')

    def test_launch_map_argument_is_optional_and_rewrites_yaml_filename(self):
        source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')
        declaration = re.search(
            r"DeclareLaunchArgument\(\s*'map',(?P<body>[\s\S]*?)\)",
            source,
        )

        self.assertIsNotNone(declaration)
        self.assertIn("default_value=''", declaration.group('body'))
        self.assertIn("'yaml_filename': map_yaml_path", source)
        self.assertIn('param_rewrites=param_substitutions', source)

    def test_active_navigation_wrapper_never_derives_a_startup_map(self):
        source = NAVIGATION_WRAPPER.read_text(encoding='utf-8')

        self.assertNotIn('get_map_path_from_yaml', source)
        self.assertNotIn("import yaml", source)
        self.assertRegex(
            source,
            r"DeclareLaunchArgument\(\s*'map',\s*default_value=''",
        )
        self.assertIn("'map': map_yaml_path", source)

    def test_live_map_switch_never_mutates_startup_parameters(self):
        source = MAP_SWITCHER.read_text(encoding='utf-8')

        self.assertNotIn('update_map_path_in_yaml', source)
        self.assertNotIn('nav2_params.yaml', source)
        self.assertNotIn("get_package_share_directory('robot_navigation2')", source)
        self.assertIn('self.active_map_name = None', source)
        self.assertIn('self.active_map_name = map_name', source)
        self.assertIn('self.load_map_cli.call_async(request)', source)

    def test_inspection_draft_restore_requires_manual_map_load(self):
        source = INSPECTION_PAGE.read_text(encoding='utf-8')
        restore_body = source.split(
            'async function restoreInspectionDraft()', 1)[1].split(
                'function parseJson(', 1)[0]

        self.assertNotIn('loadMap(', restore_body)
        self.assertIn('pendingDraftRouteId =', restore_body)
        self.assertIn('请点击“加载地图”', restore_body)


if __name__ == '__main__':
    unittest.main()
