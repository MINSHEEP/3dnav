#!/usr/bin/env python3
"""Deployment and frontend contracts for adjustable obstacle inflation."""

from pathlib import Path
import re
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FRONTEND_ROOT = PACKAGE_ROOT / 'car_vel' / 'web_frontend'
WORKSPACE_SRC = PACKAGE_ROOT.parent
LZP_ROOT = WORKSPACE_SRC / 'nav_service' / 'lzp_service'
INTERFACE_ROOT = WORKSPACE_SRC / 'nav_service' / 'lzp_interface'
NAV2_PARAMS = (
    WORKSPACE_SRC / 'navigation' / 'robot_navigation2' /
    'config' / 'nav2_params.yaml'
)


class InflationRadiusContractTests(unittest.TestCase):

    def test_interface_and_dependency_are_explicit(self):
        cmake = (INTERFACE_ROOT / 'CMakeLists.txt').read_text('utf-8')
        service = (
            INTERFACE_ROOT / 'srv' / 'SetInflationRadius.srv'
        ).read_text('utf-8')
        package = (LZP_ROOT / 'package.xml').read_text('utf-8')
        self.assertIn('srv/SetInflationRadius.srv', cmake)
        for field in (
            'float64 radius_m', 'bool success', 'bool applied',
            'string message', 'uint64 version',
        ):
            self.assertIn(field, service)
        self.assertIn('<depend>rcl_interfaces</depend>', package)

    def test_route_manager_owns_both_costmap_parameter_updates_and_gate(self):
        route = (
            LZP_ROOT / 'lzp_service' / 'route_navigation_manager.py'
        ).read_text('utf-8')
        for token in (
            "'/global_costmap/global_costmap/set_parameters'",
            "'/local_costmap/local_costmap/set_parameters'",
            "'inflation_layer.inflation_radius'",
            "'/navigation/inflation_radius'",
            "'/navigation/set_inflation_radius'",
            'InflationRadiusStore',
            'InflationSyncState',
        ):
            self.assertIn(token, route)
        self.assertGreaterEqual(
            route.count('if not self._inflation_ready_for_navigation():'), 3)

    def test_launch_exposes_the_persisted_setting_path(self):
        launch = (
            PACKAGE_ROOT / 'launch' / 'launch_manager.launch.py'
        ).read_text('utf-8')
        self.assertGreaterEqual(
            launch.count("'inflation_radius_config_path'"), 2)

    def test_settings_page_has_bounded_input_status_and_typed_service(self):
        page = (FRONTEND_ROOT / 'settings.html').read_text('utf-8')
        common = (FRONTEND_ROOT / 'robot_ui_common.js').read_text('utf-8')
        for token in (
            'id="inflation-radius-range"',
            'id="inflation-radius-input"',
            'id="btn-save-inflation-radius"',
            "'/navigation/inflation_radius'",
            "'/navigation/set_inflation_radius'",
            'UI.TYPES.setInflationRadius',
            'min="0.30"',
            'max="1.50"',
            '当前推荐值为 0.55 米',
        ):
            self.assertIn(token, page)
        self.assertIn('SetInflationRadius', common)

    def test_static_nav2_default_remains_unchanged_for_both_costmaps(self):
        params = NAV2_PARAMS.read_text('utf-8')
        active_defaults = re.findall(
            r'^\s+inflation_radius:\s*0\.55\s*$', params, re.MULTILINE)
        self.assertEqual(len(active_defaults), 2)


if __name__ == '__main__':
    unittest.main()
