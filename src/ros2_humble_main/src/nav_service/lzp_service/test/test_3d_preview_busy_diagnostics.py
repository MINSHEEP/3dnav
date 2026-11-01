"""Exercise busy rejection at the real route-preview callback boundary."""
from types import SimpleNamespace as NS
import unittest
import json
from unittest.mock import Mock

from test_3d_route_localization_refresh import harness
from test_3d_goal_preview import _load_manager_methods


class PreviewBusyDiagnosticsTests(unittest.TestCase):
    def test_each_owner_is_identified_without_planning_or_motion(self):
        cases = (
            ('_mode', 'single', '单点导航'),
            ('_mode', 'route', '多点路线'),
            ('_campus_mission_active', True, '园区任务'),
            ('_inspection_task_active', True, '巡检任务'),
            ('_lio_recovering', True, '里程计恢复'),
            ('_single_refresh_map_name', 'garden', '单点定位刷新'),
            ('_active_goal_handle', object(), 'Nav2 目标尚未结束'),
            ('_sending', True, 'Nav2 目标发送尚未完成'),
        )
        for attribute, value, expected in cases:
            with self.subTest(attribute=attribute, value=value):
                node, request, _ = harness()
                setattr(node, attribute, value)
                response = node._preview_3d_route_callback(request, NS())
                self.assertFalse(response.valid)
                self.assertEqual(response.code, 'navigation_busy')
                self.assertIn(expected, response.message)
                node._preview_action_client.send_goal_async.assert_not_called()
                node._action_client.send_goal_async.assert_not_called()
                node._nomotion_update_client.call_async.assert_not_called()

    def test_all_blockers_visible_and_no_flag_is_cleared(self):
        node, request, _ = harness()
        node._inspection_task_active = True
        node._inspection_task_id = 'task-123'
        node._lio_recovering = True
        node._auto_recovery_stage = 'failed'
        response = node._preview_3d_route_callback(request, NS())
        for text in ('巡检任务', 'task-123', '里程计恢复', 'failed'):
            self.assertIn(text, response.message)
        self.assertTrue(node._inspection_task_active)
        self.assertTrue(node._lio_recovering)

    def test_confirmed_owner_release_allows_preview_but_not_motion(self):
        node, request, _ = harness()
        node._campus_mission_active = True
        self.assertFalse(node._preview_3d_route_callback(request, NS()).valid)
        node._campus_mission_active = False  # owner's terminal status
        response = node._preview_3d_route_callback(request, NS())
        self.assertTrue(response.valid, response.message)
        node._action_client.send_goal_async.assert_not_called()

    def test_inspection_permission_does_not_bypass_other_owners(self):
        node, request, _ = harness()
        node._inspection_task_active = True
        self.assertEqual(node._navigation_busy_details(allow_inspection=True), [])
        node._lio_recovering = True
        self.assertIn('里程计恢复', '；'.join(
            node._navigation_busy_details(allow_inspection=True)))

    def test_readonly_status_exposes_same_blockers(self):
        implementation = _load_manager_methods(
            '_publish_navigation_mode', '_navigation_mode_locked')
        node = implementation()
        node._mode = 'idle'
        node._campus_mission_active = False
        node._inspection_task_active = False
        node._lio_recovering = True
        node._auto_recovery_stage = 'failed'
        node._navigation_mode_store = NS(snapshot=lambda: {'mode': 'terrain_3d'})
        node._navigation_mode_publisher = Mock()
        node._publish_navigation_mode.__func__.__globals__.update(
            navigation_mode_label=lambda mode: mode, String=NS)
        node._publish_navigation_mode()
        payload = json.loads(node._navigation_mode_publisher.publish.call_args[0][0].data)
        self.assertFalse(payload['locked'])  # no active task ownership
        self.assertIn('failed', payload['preview_blockers'][0])


if __name__ == '__main__':
    unittest.main()
