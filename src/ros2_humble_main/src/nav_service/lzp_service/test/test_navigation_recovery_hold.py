"""Exercise real recovery branches without ROS or hardware commands."""
import ast
import copy
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[1] / 'lzp_service/route_navigation_manager.py'


def manager():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    methods = [item for item in ast.walk(tree) if isinstance(item, ast.FunctionDef)
               and item.name in ('_resume_after_auto_recovery', '_fail_auto_recovery',
                                 '_lio_health_callback', '_finish_task')]
    namespace = {'copy': copy, 'json': json, 'time': time}
    exec('from __future__ import annotations\nclass Manager:\n' +
         '\n'.join('    ' + ast.unparse(item).replace('\n', '\n    ')
                   for item in methods), namespace)
    node = namespace['Manager']()
    node._lock = threading.RLock()
    node._mode = 'single'
    node._state = 'waiting_lio_recovery'
    node._lio_recovering = True
    node._lio_recovery_hard_fault = False
    node._auto_recovery_stage = 'monitoring'
    node._robot_body_status = {'status': {'emergency_latched': False}}
    node._active_goal_handle = None
    node._sending = False
    node._retry_current_after_recovery = True
    node._recovery_pose = SimpleNamespace(goal='original')
    node._send_zero_velocity = Mock()
    node._publish_goal_status = Mock()
    node._publish_route_log = Mock()
    node._publish_all_status = Mock()
    node._dispatch_current_route_point = Mock()
    node._send_pose = Mock()
    node._clear_preflight_wait = Mock()
    node._clear_emergency_stop_publisher = Mock()
    return node


class NavigationRecoveryHoldTests(unittest.TestCase):
    def assert_manual_hold(self, node):
        self.assertEqual(node._auto_recovery_stage, 'manual_reset_required')
        self.assertEqual(node._state, 'recovery_failed')
        node._send_zero_velocity.assert_called()
        node._send_pose.assert_not_called()
        node._dispatch_current_route_point.assert_not_called()
        node._clear_emergency_stop_publisher.publish.assert_not_called()

    def test_complete_recovery_cannot_reset_a_real_lio_fault(self):
        node = manager()
        node._lio_recovery_hard_fault = True
        node._resume_after_auto_recovery()
        self.assert_manual_hold(node)

    def test_complete_recovery_cannot_reset_an_operator_emergency(self):
        node = manager()
        node._robot_body_status['status']['emergency_latched'] = True
        node._resume_after_auto_recovery()
        self.assert_manual_hold(node)

    def test_fast_healthy_transition_cannot_restart_a_hard_fault_goal(self):
        node = manager()
        node._lio_recovery_hard_fault = True
        node._lio_health_callback(SimpleNamespace(data='{"state":"healthy"}'))
        self.assert_manual_hold(node)

    def test_fast_healthy_transition_preserves_operator_emergency(self):
        node = manager()
        node._robot_body_status['status']['emergency_latched'] = True
        node._lio_health_callback(SimpleNamespace(data='{"state":"healthy"}'))
        self.assert_manual_hold(node)

    def test_soft_pause_can_retry_without_hardware_reset(self):
        node = manager()
        node._lio_health_callback(SimpleNamespace(data='{"state":"healthy"}'))
        self.assertFalse(node._lio_recovering)
        self.assertEqual(node._auto_recovery_stage, 'idle')
        node._send_pose.assert_called_once()
        node._clear_emergency_stop_publisher.publish.assert_not_called()

    def test_failed_recovery_stays_held_after_native_becomes_healthy(self):
        node = manager()
        node._fail_auto_recovery('localization could not be verified')
        node._lio_health_callback(SimpleNamespace(data='{"state":"healthy"}'))
        self.assertEqual(node._auto_recovery_stage, 'failed')
        node._send_zero_velocity.assert_called_once()
        node._send_pose.assert_not_called()
        node._publish_all_status.assert_called_once()

    def test_manual_hold_survives_another_soft_recovery_cycle(self):
        node = manager()
        node._lio_recovery_hard_fault = True
        node._resume_after_auto_recovery()
        node._capture_recovery_rough_pose = Mock()
        node._clear_preflight_wait = Mock()
        node._lio_health_callback(SimpleNamespace(data='{"state":"recovering"}'))
        node._lio_health_callback(SimpleNamespace(data='{"state":"healthy"}'))
        self.assert_manual_hold(node)

    def test_confirmed_task_stop_clears_only_task_hold_not_hardware_latch(self):
        node = manager()
        node._lio_recovery_hard_fault = True
        node._resume_after_auto_recovery()
        node._finish_task('stopped', 'operator stopped the task')
        self.assertEqual(node._auto_recovery_stage, 'idle')
        self.assertEqual(node._mode, 'idle')
        node._clear_emergency_stop_publisher.publish.assert_not_called()

    def test_failed_task_stop_allows_health_revalidation_without_old_goal(self):
        node = manager()
        node._fail_auto_recovery('timeout')
        node._finish_task('stopped', 'operator stopped the task')
        node._lio_health_callback(SimpleNamespace(data='{"state":"healthy"}'))
        self.assertFalse(node._lio_recovering)
        self.assertEqual(node._auto_recovery_stage, 'idle')
        node._send_pose.assert_not_called()
        node._clear_emergency_stop_publisher.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
