#!/usr/bin/env python3
"""Contract for navigation/inspection page-entry robot configuration."""

from pathlib import Path
import unittest


CAR_VEL_ROOT = Path(__file__).resolve().parents[1]
WEB_ROOT = CAR_VEL_ROOT / 'car_vel' / 'web_frontend'
BRIDGE = CAR_VEL_ROOT / 'car_vel' / 'dog_remote_bridge.py'


class NavigationPageSafetyModeTests(unittest.TestCase):

    def test_backend_sets_explicit_idempotent_targets_after_standing(self):
        source = BRIDGE.read_text(encoding='utf-8')
        callback = source.split(
            'def _ensure_standing_callback(', 1)[1].split(
                'def _ensure_standing(', 1)[0]

        self.assertIn('self._configure_navigation_safety_mode()', callback)
        self.assertIn('self._send_command(CMD_OBSTACLE_STOP, 1)', callback)
        self.assertIn(
            'self._send_command(CMD_SET_MOTION_MODEL, self.obstacle_model)',
            callback)
        self.assertIn('self.obstacle_stop_enabled = True', callback)
        self.assertNotIn('self.current_model = self.obstacle_model', callback)
        self.assertIn('response.success = self._configure_navigation_safety_mode()', callback)
        self.assertIn('self._navigation_gait_confirmed(after=sent_at)', callback)
        self.assertNotIn('self._toggle_model()', callback)
        self.assertNotIn('self._toggle_obstacle_stop()', callback)

    def test_navigation_page_initializes_and_gates_map_controls(self):
        page = (WEB_ROOT / 'navigation.html').read_text(encoding='utf-8')

        self.assertIn("'/dog_remote/ensure_standing'", page)
        self.assertIn('function ensureNavigationSafety(', page)
        self.assertIn('ensureNavigationSafety(activeSession);', page)
        self.assertIn('!navigationSafetyReady || !nav2Ready', page)
        self.assertIn('已切换越障模式并保持碰撞停止开启', page)

    def test_inspection_page_uses_the_same_entry_contract(self):
        page = (WEB_ROOT / 'inspection.html').read_text(encoding='utf-8')

        self.assertIn("'/dog_remote/ensure_standing'", page)
        self.assertIn('ensureInspectionStanding(activeSession);', page)
        self.assertIn('!standingConfirmed', page)
        self.assertIn('已切换越障模式并保持碰撞停止开启', page)


if __name__ == '__main__':
    unittest.main()
