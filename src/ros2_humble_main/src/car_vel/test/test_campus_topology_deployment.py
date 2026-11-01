#!/usr/bin/env python3
"""Static deployment contracts for browser-managed campus topology."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
INTERFACE = ROOT / 'nav_service' / 'lzp_interface'
SERVICE = ROOT / 'nav_service' / 'lzp_service'
CAR_VEL = ROOT / 'car_vel'


class CampusTopologyDeploymentTest(unittest.TestCase):
    def test_interface_and_entry_points_are_installed(self):
        cmake = (INTERFACE / 'CMakeLists.txt').read_text(encoding='utf-8')
        setup = (SERVICE / 'setup.py').read_text(encoding='utf-8')
        self.assertIn('srv/ManageCampusTopology.srv', cmake)
        self.assertIn(
            'terrain_observer_node = lzp_service.terrain_observer_node:main',
            setup)

    def test_launch_uses_writable_default_and_live_observer(self):
        launch = (CAR_VEL / 'launch' / 'launch_manager.launch.py').read_text(
            encoding='utf-8')
        navigation = (CAR_VEL / 'launch' / 'car_vel.launch.py').read_text(
            encoding='utf-8')
        self.assertIn('~/.ros/robot_navigation/campus_topology.json', launch)
        self.assertIn("executable='terrain_observer_node'", navigation)
        self.assertIn("'/terrain_observation'", navigation)


if __name__ == '__main__':
    unittest.main()
