#!/usr/bin/env python3
"""Static integration contracts for fail-safe mapping teleoperation."""

from pathlib import Path
import unittest


CAR = Path(__file__).resolve().parents[1]


class MappingTeleopContractTest(unittest.TestCase):

    def test_mapping_page_uses_dedicated_teleop_topic(self):
        page = (CAR / 'car_vel' / 'web_frontend' / 'mapping.html').read_text(
            encoding='utf-8')
        self.assertIn("cmdVelTopic: '/cmd_vel_teleop'", page)
        self.assertNotIn('A</kbd>/<kbd>D</kbd> 横移', page)

    def test_xingji_recovery_receives_explicit_session_and_key_release(self):
        page = (CAR / 'car_vel' / 'web_frontend' / 'mapping.html').read_text(
            encoding='utf-8')
        common = (CAR / 'car_vel' / 'web_frontend' / 'robot_ui_common.js').read_text(
            encoding='utf-8')
        bridge = (CAR / 'car_vel' / 'robot_body_bridge.py').read_text(
            encoding='utf-8')
        self.assertIn("'/xingji/mapping/teleop_active'", page)
        self.assertIn("'/xingji/mapping/teleop_neutral'", page)
        self.assertIn('onNeutral: publishMappingTeleopNeutral', page)
        self.assertIn('!Object.values(teleop.keys).some(Boolean)', page)
        self.assertIn('this.onNeutral()', common)
        self.assertIn("'/xingji/mapping/teleop_active'", bridge)
        self.assertIn("'/xingji/mapping/teleop_neutral'", bridge)

    def test_arbiter_accepts_teleop_without_direct_hardware_bypass(self):
        arbiter = (CAR / 'car_vel' / 'velocity_arbiter.py').read_text(
            encoding='utf-8')
        self.assertIn("Twist, '/cmd_vel_teleop'", arbiter)
        self.assertIn("if source == 'teleop':", arbiter)
        self.assertIn('output = self._latest_teleop', arbiter)

        page = (CAR / 'car_vel' / 'web_frontend' / 'mapping.html').read_text(
            encoding='utf-8')
        self.assertNotIn("cmdVelTopic: '/cmd_vel'", page)

    def test_mapping_launch_keeps_fail_safe_arbiter(self):
        launch = (CAR / 'launch' / 'mapping.launch.py').read_text(
            encoding='utf-8')
        self.assertIn("executable='velocity_arbiter'", launch)
        self.assertIn("'command_timeout_sec': 0.3", launch)


if __name__ == '__main__':
    unittest.main()
