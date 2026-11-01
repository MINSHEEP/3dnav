#!/usr/bin/env python3
"""Static contracts for inspection readiness, timeout and respawn safety."""

from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent
LAUNCH_SOURCE = (
    PACKAGE_ROOT / 'launch' / 'launch_manager.launch.py'
).read_text(encoding='utf-8')
MANAGER_SOURCE = (
    WORKSPACE_SRC / 'nav_service' / 'lzp_service' / 'lzp_service'
    / 'inspection_task_manager.py'
).read_text(encoding='utf-8')
ROUTE_MANAGER_SOURCE = (
    WORKSPACE_SRC / 'nav_service' / 'lzp_service' / 'lzp_service'
    / 'route_navigation_manager.py'
).read_text(encoding='utf-8')
NAVIGATION_SOURCE = (
    PACKAGE_ROOT / 'car_vel' / 'web_frontend' / 'navigation.html'
).read_text(encoding='utf-8')
COMMON_SOURCE = (
    PACKAGE_ROOT / 'car_vel' / 'web_frontend' / 'robot_ui_common.js'
).read_text(encoding='utf-8')


class InspectionReliabilityContracts(unittest.TestCase):

    def test_route_and_inspection_nodes_are_respawned(self):
        for executable in ('route_navigation_manager', 'inspection_task_manager'):
            marker = f"executable='{executable}'"
            start = LAUNCH_SOURCE.index(marker)
            node_block = LAUNCH_SOURCE[start:start + 500]
            self.assertIn('respawn=True', node_block)
            self.assertIn('respawn_delay=2.0', node_block)

    def test_rosbridge_uses_finite_parallel_service_calls(self):
        self.assertIn("'default_call_service_timeout': 75.0", LAUNCH_SOURCE)
        self.assertIn("'call_services_in_new_thread': True", LAUNCH_SOURCE)
        self.assertIn("'send_action_goals_in_new_thread': True", LAUNCH_SOURCE)

    def test_backend_enforces_readiness_and_route_deadline(self):
        self.assertIn("Bool, '/nav2_ready'", MANAGER_SOURCE)
        self.assertIn("String, '/relocalization/status'", MANAGER_SOURCE)
        self.assertIn("if not self._nav2_ready:", MANAGER_SOURCE)
        self.assertIn("if not self._localization_ready:", MANAGER_SOURCE)
        self.assertIn("'route_service_timeout_sec', 10.0", MANAGER_SOURCE)
        self.assertIn("'/navigate_to_pose/_action/cancel_goal'", MANAGER_SOURCE)
        self.assertIn('CancelGoal.Request()', MANAGER_SOURCE)
        self.assertIn("'storage_warning_free_gb', 5.0", MANAGER_SOURCE)
        self.assertIn('shutil.disk_usage(self._evidence_root)', MANAGER_SOURCE)

    def test_browser_service_calls_have_a_default_deadline(self):
        self.assertIn('timeoutMs = 15000', COMMON_SOURCE)
        self.assertIn('ROS2 服务 ${name} 调用超时', COMMON_SOURCE)

    def test_paused_routes_cannot_be_skipped_implicitly(self):
        self.assertIn(
            'Resume the route before skipping the current point',
            ROUTE_MANAGER_SOURCE)
        self.assertIn(
            '!active || routeExecutionStatus.paused || routeServiceBusy',
            NAVIGATION_SOURCE)

    def test_transient_preflight_wait_is_bounded_and_deployed(self):
        self.assertIn("'localization_stale'", ROUTE_MANAGER_SOURCE)
        self.assertIn("'terrain_not_ready'", ROUTE_MANAGER_SOURCE)
        self.assertIn("self._begin_preflight_wait(pose, 'route', preflight)",
                      ROUTE_MANAGER_SOURCE)
        self.assertIn("'/request_nomotion_update'", ROUTE_MANAGER_SOURCE)
        self.assertIn("self._state = 'paused_preflight'", ROUTE_MANAGER_SOURCE)
        self.assertIn("'goal_preflight_wait_timeout_sec'", LAUNCH_SOURCE)
        self.assertIn("'goal_preflight_nomotion_max_requests'", LAUNCH_SOURCE)


if __name__ == '__main__':
    unittest.main()
