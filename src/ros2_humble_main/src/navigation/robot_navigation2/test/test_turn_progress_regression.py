#!/usr/bin/env python3
"""Regression contracts for turn-in-place navigation progress on ROS 2 Humble."""

import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parents[1]
NAV_PARAMS = PACKAGE_ROOT / 'config' / 'nav2_params.yaml'
PLUGIN_SOURCE = PACKAGE_ROOT / 'src' / 'pose_progress_checker.cpp'
PLUGIN_HEADER = PACKAGE_ROOT / 'include' / 'robot_navigation2' / 'pose_progress_checker.hpp'
PLUGIN_XML = PACKAGE_ROOT / 'pose_progress_checker_plugin.xml'
CMDVEL_BRIDGE = WORKSPACE_SRC / 'car_vel' / 'car_vel' / 'cmdvel_bridge_ros2.py'


def shortest_angle(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


def survives_progress_check(samples, radius, angle, timeout):
    """Replay the ProgressChecker baseline rules over (time, x, y, yaw)."""
    baseline = samples[0]
    for sample in samples[1:]:
        distance = math.hypot(sample[1] - baseline[1], sample[2] - baseline[2])
        turned = abs(shortest_angle(sample[3], baseline[3]))
        if distance > radius or (angle is not None and turned > angle):
            baseline = sample
        elif sample[0] - baseline[0] > timeout:
            return False
    return True


class TurnProgressRegressionTest(unittest.TestCase):
    def test_recorded_turn_is_progress_when_rotation_is_counted(self):
        # Minimized from the 2026-08-06 bag: about 7 cm of translation while
        # rotating roughly 180 degrees during an 18-second window.
        samples = [
            (0.0, 0.000, 0.000, math.radians(85.3)),
            (4.0, 0.015, 0.010, math.radians(144.4)),
            (8.0, 0.035, 0.015, math.radians(223.1)),
            (12.0, 0.045, 0.025, math.radians(268.4)),
            (16.0, 0.055, 0.045, math.radians(207.4)),
            (18.0, 0.020, 0.069, math.radians(266.1)),
        ]
        self.assertFalse(
            survives_progress_check(samples, radius=0.10, angle=None, timeout=15.0),
            'The old translation-only checker must reproduce the field failure.',
        )
        self.assertTrue(
            survives_progress_check(samples, radius=0.10, angle=0.20, timeout=15.0),
            'A valid turn must refresh navigation progress.',
        )

    def test_humble_plugin_and_navigation_configuration_are_wired(self):
        self.assertTrue(PLUGIN_HEADER.is_file(), 'Angle-aware progress checker header is missing.')
        self.assertTrue(PLUGIN_SOURCE.is_file(), 'Angle-aware progress checker source is missing.')
        self.assertTrue(PLUGIN_XML.is_file(), 'Progress checker plugin description is missing.')

        source = PLUGIN_SOURCE.read_text(encoding='utf-8')
        plugin_xml = PLUGIN_XML.read_text(encoding='utf-8')
        plugin_tree = ET.parse(PLUGIN_XML)
        cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
        package_xml = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')
        params = NAV_PARAMS.read_text(encoding='utf-8')

        self.assertIn('required_movement_angle', source)
        self.assertIn('isRobotMovedEnough', source)
        self.assertIn('robot_navigation2::PoseProgressChecker', plugin_xml)
        plugin_class = plugin_tree.getroot().find('class')
        self.assertIsNotNone(plugin_class)
        self.assertEqual(plugin_tree.getroot().attrib['path'], 'pose_progress_checker')
        self.assertEqual(plugin_class.attrib['base_class_type'], 'nav2_core::ProgressChecker')
        self.assertIn('pluginlib_export_plugin_description_file', cmake)
        self.assertIn('install(TARGETS pose_progress_checker', cmake)
        self.assertIn('install(DIRECTORY include/', cmake)
        self.assertIn('ament_add_pytest_test(', cmake)
        self.assertIn('<depend>nav2_core</depend>', package_xml)
        self.assertIn('<depend>pluginlib</depend>', package_xml)
        self.assertIn('plugin: "robot_navigation2::PoseProgressChecker"', params)
        self.assertRegex(params, r'required_movement_angle:\s*0\.20\b')
        self.assertRegex(params, r'rotate_to_goal_heading:\s*false\b')

    def test_hardware_bridge_does_not_amplify_turn_rate(self):
        source = CMDVEL_BRIDGE.read_text(encoding='utf-8')
        self.assertIn("declare_parameter('linear_scale_factor', 1.5)", source)
        self.assertIn("declare_parameter('angular_scale_factor', 1.0)", source)
        self.assertIn("declare_parameter('min_in_place_turn_command', 0.15)", source)
        self.assertIn('shape_velocity_command(', source)
        self.assertIn('linear_scale=self.linear_scale', source)
        self.assertIn('angular_scale=self.angular_scale', source)
        self.assertIn('min_in_place_turn_command=self.min_in_place_turn', source)
        self.assertNotIn('msg.angular.z * self.scale', source)


if __name__ == '__main__':
    unittest.main()
