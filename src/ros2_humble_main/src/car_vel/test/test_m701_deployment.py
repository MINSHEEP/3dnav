#!/usr/bin/env python3
"""Deployment contracts for the persistent M701 serial bridge."""

from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH = (PACKAGE_ROOT / 'launch' / 'launch_manager.launch.py').read_text(
    encoding='utf-8')
SETUP = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
PACKAGE_XML = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')
BRIDGE = (PACKAGE_ROOT / 'car_vel' / 'm701_air_quality_bridge.py')
DEFAULT_M701_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'


class M701DeploymentTests(unittest.TestCase):

    def test_bridge_starts_by_default_as_a_page_independent_backend(self):
        argument_start = LAUNCH.index("'m701_enabled'")
        argument_block = LAUNCH[argument_start:argument_start + 350]
        self.assertIn("default_value='true'", argument_block)
        self.assertIn("'m701_port'", LAUNCH)
        self.assertIn(f"DEFAULT_M701_PORT = '{DEFAULT_M701_PORT}'", LAUNCH)
        self.assertIn('default_value=DEFAULT_M701_PORT', LAUNCH)
        marker = "executable='m701_air_quality_bridge'"
        self.assertIn(marker, LAUNCH)
        marker_index = LAUNCH.index(marker)
        block_start = LAUNCH.rfind('        Node(', 0, marker_index)
        block_end = LAUNCH.find('        Node(', marker_index + len(marker))
        block = LAUNCH[block_start:block_end]
        self.assertIn("condition=IfCondition", block)
        self.assertIn("'poll_interval_sec'", block)
        self.assertIn("respawn=True", block)

    def test_python_entry_point_and_serial_dependency_are_installed(self):
        self.assertIn(
            "'m701_air_quality_bridge = car_vel.m701_air_quality_bridge:main'",
            SETUP,
        )
        self.assertIn('<exec_depend>python3-serial</exec_depend>', PACKAGE_XML)
        self.assertIn("'M701_RS485.md'", SETUP)

    def test_vendor_driver_device_name_is_documented(self):
        guide = (PACKAGE_ROOT / 'M701_RS485.md').read_text(encoding='utf-8')
        self.assertIn('/dev/ttyCH341USB0', guide)
        self.assertIn('linux-headers-$(uname -r)', guide)
        self.assertIn('sudo make install', guide)
        self.assertIn('Secure Boot', guide)

    def test_bridge_publishes_read_only_json_status(self):
        source = BRIDGE.read_text(encoding='utf-8')
        self.assertIn(f"DEFAULT_M701_PORT = '{DEFAULT_M701_PORT}'", source)
        self.assertIn("self.declare_parameter('port', DEFAULT_M701_PORT)", source)
        self.assertIn("'/air_quality/m701'", source)
        self.assertIn('String', source)
        self.assertIn("'valid': False", source)
        self.assertIn("'warming_up'", source)
        self.assertNotIn('create_subscription(', source)


if __name__ == '__main__':
    unittest.main()
