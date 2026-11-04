#!/usr/bin/env python3
"""Deployment and UI contracts for selectable inspection navigation."""

from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FRONTEND_ROOT = PACKAGE_ROOT / 'car_vel' / 'web_frontend'
WORKSPACE_SRC = PACKAGE_ROOT.parent
LZP_ROOT = WORKSPACE_SRC / 'nav_service' / 'lzp_service'
INTERFACE_ROOT = WORKSPACE_SRC / 'nav_service' / 'lzp_interface'


class NavigationModeContractTests(unittest.TestCase):
    def test_ros_interface_has_a_dedicated_navigation_mode(self):
        cmake = (INTERFACE_ROOT / 'CMakeLists.txt').read_text('utf-8')
        service = (
            INTERFACE_ROOT / 'srv' / 'SetNavigationMode.srv'
        ).read_text('utf-8')
        start_route = (
            INTERFACE_ROOT / 'srv' / 'StartRoute.srv'
        ).read_text('utf-8')
        route_status = (
            INTERFACE_ROOT / 'msg' / 'RouteExecutionStatus.msg'
        ).read_text('utf-8')
        self.assertIn('srv/SetNavigationMode.srv', cmake)
        self.assertIn('string mode', service)
        self.assertIn('string navigation_mode', start_route)
        self.assertIn('string navigation_mode', route_status)

    def test_settings_page_selects_modes_and_explains_shared_localization(self):
        page = (FRONTEND_ROOT / 'settings.html').read_text('utf-8')
        common = (FRONTEND_ROOT / 'robot_ui_common.js').read_text('utf-8')
        self.assertIn('id="btn-nav-mode-2d"', page)
        self.assertIn('id="btn-nav-mode-3d"', page)
        self.assertIn("'/navigation/mode'", page)
        self.assertIn("'/navigation/set_mode'", page)
        self.assertIn('UI.TYPES.setNavigationMode', page)
        self.assertIn('三维精确定位', page)
        self.assertIn('SetNavigationMode', common)

    def test_inspection_page_draws_and_clears_real_global_plan(self):
        page = (FRONTEND_ROOT / 'inspection.html').read_text('utf-8')
        self.assertIn("'/navigation/global_path'", page)
        self.assertIn("'/navigation/global_path/status'", page)
        self.assertIn('mapCanvas.setGlobalPath(', page)
        self.assertIn('mapCanvas.clearGlobalPath()', page)
        self.assertIn('navigation_mode', page)

    def test_mode_is_frozen_by_inspection_and_enforced_by_route_manager(self):
        inspection = (
            LZP_ROOT / 'lzp_service' / 'inspection_task_manager.py'
        ).read_text('utf-8')
        route = (
            LZP_ROOT / 'lzp_service' / 'route_navigation_manager.py'
        ).read_text('utf-8')
        for source in (inspection, route):
            self.assertIn("'/navigation/mode'", source)
        self.assertIn('request.navigation_mode', inspection)
        self.assertIn("self._state['navigation_mode']", inspection)
        self.assertIn("'/navigation/set_mode'", route)
        self.assertIn('SetNavigationMode', route)
        self.assertIn('_active_navigation_mode', route)
        launch = (
            PACKAGE_ROOT / 'launch' / 'launch_manager.launch.py'
        ).read_text('utf-8')
        self.assertIn("'navigation_mode_config_path'", launch)

    def test_all_terrain_adapters_are_mode_aware(self):
        files = [
            LZP_ROOT / 'lzp_service' / 'terrain_costmap_node.py',
            LZP_ROOT / 'lzp_service' / 'terrain_path_node.py',
            PACKAGE_ROOT / 'car_vel' / 'velocity_arbiter.py',
            LZP_ROOT / 'lzp_service' / 'automatic_terrain_navigation_node.py',
        ]
        for path in files:
            source = path.read_text('utf-8')
            self.assertIn("'/navigation/mode'", source, str(path))
            self.assertTrue(
                'map_2d' in source or 'MAP_2D' in source,
                str(path),
            )


if __name__ == '__main__':
    unittest.main()
