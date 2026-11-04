"""Goal decisions remain inspectable when a browser's local log is lost."""
import ast
import json
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_single_goal_localization_refresh as fixtures


class NavigationGoalDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.node = fixtures.manager()
        self.logger = Mock()
        self.node.get_logger = lambda: self.logger

    def records(self):
        calls = self.logger.info.call_args_list + self.logger.warning.call_args_list
        return [json.loads(call.args[0].split('NAV_GOAL ', 1)[1])
                for call in calls if 'NAV_GOAL ' in call.args[0]]

    def test_soft_stop_topics_revoke_pending_preview_and_goal_authority(self):
        tree = ast.parse(fixtures.SOURCE.read_text(encoding='utf-8'))
        wiring = {}
        for call in ast.walk(tree):
            if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                    and call.func.attr == 'create_subscription' and len(call.args) >= 3
                    and isinstance(call.args[1], ast.Constant)
                    and isinstance(call.args[2], ast.Attribute)):
                wiring[call.args[1].value] = call.args[2].attr
        for topic in ('/motion_stop', '/emergency_stop'):
            with self.subTest(topic=topic):
                self.assertIn(topic, wiring)
                node = fixtures.manager()
                node._single_goal_callback(fixtures.pose(x=3.0))
                node._preview_token = object()
                generation = getattr(node, '_preview_request_generation', 0)
                getattr(node, wiring[topic])(object())
                self.assertIsNone(node._preview_token)
                self.assertIsNone(node._pending_single_pose)
                self.assertGreater(node._preview_request_generation, generation)
                self.assertEqual(node._action_client.sent, [])

    def test_stale_goal_records_received_and_refresh_without_dispatch(self):
        self.node._latest_amcl_pose = fixtures.amcl(10_000_000_000)
        self.node._latest_amcl_stamp_ns = 10_000_000_000
        self.node._latest_amcl_received_at = time.monotonic() - 4.0
        self.node._single_goal_callback(fixtures.pose(x=3.0))
        records = self.records()
        self.assertTrue(any(item['event'] == 'received' and item['goal']['x'] == 3.0
                            for item in records))
        refresh = next(item for item in records
                       if item['event'] == 'goal_refreshing_localization')
        self.assertEqual(refresh['map_name'], 'ceshi3_remap1')
        self.assertGreaterEqual(refresh['amcl_age_sec'], 3.0)
        self.assertIn('继续执行本次目标', refresh['reason'])
        self.assertEqual(self.node._action_client.sent, [])

    def test_hardware_emergency_rejects_goal_without_reset_or_queue(self):
        self.node._robot_body_status = {'status': {'emergency_latched': True}}
        self.node._latest_amcl_pose = fixtures.amcl(20_000_000_000)
        self.node._latest_amcl_stamp_ns = 20_000_000_000
        self.node._latest_amcl_received_at = time.monotonic()
        self.node._single_goal_callback(fixtures.pose())
        self.assertEqual(self.node._goal_status_publisher.values[-1],
                         'goal_rejected_emergency_latched')
        self.assertIsNone(self.node._pending_single_pose)
        self.assertEqual(self.node._action_client.sent, [])

    def test_ui_block_reason_is_logged_but_cannot_send_a_goal(self):
        payload = json.dumps({'event': 'page_notice', 'map_name': 'campus',
                              'reason': '初始定位尚未通过确认'})
        self.node._goal_ui_event_callback(fixtures.String(payload))
        record = self.records()[-1]
        self.assertEqual(record['event'], 'ui_page_notice')
        self.assertIn('初始定位', record['reason'])
        self.assertEqual(self.node._action_client.sent, [])
        self.node._goal_ui_event_callback(fixtures.String(payload))
        self.assertEqual(len(self.records()), 1)

    def test_invalid_ui_event_has_no_motion_or_state_side_effect(self):
        for text in ('not json', '[]', '{"event":"start_navigation"}',
                     json.dumps({'event': 'page_notice', 'reason': 'x' * 3000})):
            self.node._goal_ui_event_callback(fixtures.String(text))
        self.assertEqual(self.node._mode, 'idle')
        self.assertEqual(self.node._action_client.sent, [])
        self.assertEqual(self.records(), [])


if __name__ == '__main__':
    unittest.main()
