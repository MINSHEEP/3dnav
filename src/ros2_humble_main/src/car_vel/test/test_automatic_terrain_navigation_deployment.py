#!/usr/bin/env python3
"""Static contracts for automatic same-map terrain navigation."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
SERVICE = ROOT / 'nav_service' / 'lzp_service'
CAR = ROOT / 'car_vel'


class AutomaticTerrainNavigationDeploymentTest(unittest.TestCase):
    def test_supervisor_is_installed_and_started_with_navigation(self):
        setup = (SERVICE / 'setup.py').read_text(encoding='utf-8')
        launch = (CAR / 'launch' / 'car_vel.launch.py').read_text(
            encoding='utf-8')
        self.assertIn(
            'automatic_terrain_navigation_node = '
            'lzp_service.automatic_terrain_navigation_node:main', setup)
        self.assertIn(
            "executable='automatic_terrain_navigation_node'", launch)

    def test_velocity_arbiter_enforces_live_terrain_guard(self):
        arbiter = (CAR / 'car_vel' / 'velocity_arbiter.py').read_text(
            encoding='utf-8')
        self.assertIn("'/automatic_terrain_navigation/safe'", arbiter)
        self.assertIn("'/automatic_terrain_navigation/speed_limit'", arbiter)
        self.assertIn('constrain_navigation_velocity', arbiter)

    def test_global_grid_uses_bidirectional_automatic_limits(self):
        launch = (CAR / 'launch' / 'car_vel.launch.py').read_text(
            encoding='utf-8')
        mapping_launch = (CAR / 'launch' / 'mapping.launch.py').read_text(
            encoding='utf-8')
        self.assertIn("'special_slope_deg': 18.0", launch)
        self.assertIn("'special_step_m': 0.12", launch)
        self.assertIn("'roughness_caution_m': 0.05", launch)
        self.assertIn("'special_slope_deg': 18.0", mapping_launch)
        self.assertIn("'special_step_m': 0.12", mapping_launch)
        self.assertIn("'roughness_caution_m': 0.05", mapping_launch)
        self.assertNotIn('caution_roughness_m', launch + mapping_launch)
        self.assertIn("'special_cost': 100", launch)

    def test_browser_config_is_persisted_and_published_to_supervisor(self):
        store = (SERVICE / 'lzp_service' /
                 'campus_topology_store.py').read_text(encoding='utf-8')
        navigation_service = (SERVICE / 'lzp_service' /
                              'terrain_navigation_service.py').read_text(
                                  encoding='utf-8')
        self.assertIn("'automatic_navigation'", store)
        self.assertIn("'/automatic_terrain_navigation/config'", navigation_service)
        self.assertIn('_automatic_config_publisher.publish', navigation_service)


if __name__ == '__main__':
    unittest.main()
