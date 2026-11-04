"""Real 3D callback/refresh integration, deterministic time and fake ROS I/O."""
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import test_3d_goal_preview as fixtures


class Clock:
    def __init__(self):
        self.now = 100.0
        self.on_sleep = lambda: None

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        self.on_sleep()


class Future:
    def __init__(self, value=None, error=None):
        self.value, self.error = value, error

    def done(self):
        return True

    def result(self):
        if self.error:
            raise self.error
        return self.value


def sample(stamp_ns=20_000_000_000, x=1.2):
    return NS(header=NS(frame_id='map', stamp=NS(sec=stamp_ns // 10**9,
        nanosec=stamp_ns % 10**9)), pose=NS(pose=fixtures._pose(x, 1.2).pose))


def harness():
    node = fixtures._guard_harness()
    names = ('_goal_preflight', '_preview_3d_goal_callback', '_amcl_pose_callback',
             '_start_previewed_3d_goal_callback', '_refresh_3d_preview_pose',
             '_recheck_3d_start', '_validate_3d_route_segments',
             '_cancel_navigation_callback')
    implementation = fixtures._load_manager_methods(*names)
    clock = Clock()
    for name in names:
        function = getattr(implementation, name)
        function.__globals__.update(time=clock, EmptyService=NS(Request=NS),
            uses_terrain=lambda mode: mode == 'terrain_3d',
            validate_navigation_goal=lambda *_a, **_k: fixtures.Result(True, 'ok', 'clear'))
        setattr(node, name, MethodType(function, node))
    node._preview_3d_guard.__func__.__globals__['time'] = clock
    node._lock = fixtures.threading.RLock()
    # These tests isolate the existing refresh protocol; authority gating is
    # exercised separately by test_global_localization_preflight.
    node._global_localization_check = lambda: None
    node._preview_busy = False
    node._preview_token = None
    node._map_generation = 1
    node._preview_request_generation = 0
    node._robot_body_received_at = clock.now
    node._latest_amcl_pose = sample(10_000_000_000)
    node._latest_amcl_stamp_ns = 10_000_000_000
    node._latest_amcl_received_at = clock.now - 5.133
    node._goal_preflight_pose_max_age = 3.0
    node._goal_preflight_wait_timeout = 0.4
    node._nomotion_update_interval = 0.1
    node._nomotion_update_max_requests = 3
    node._goal_preflight_tolerance = 0.1
    node._goal_preflight_start_tolerance = 0.35
    node._traversability_map_name = node._current_map_name
    node._terrain_costmap_ready_load_id = node._current_map_load_id
    node._traversability_grid = fixtures._Grid([[0] * 5 for _ in range(4)])
    node._preview_plan_timeout = 1.0
    node._preview_ttl = 20.0
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=20_000_000_000, to_msg=lambda: 'stamp'))
    node._log_goal_event = Mock()
    node._publish_goal_status = Mock()
    node._send_zero_velocity = Mock()
    node._publish_all_status = Mock()
    node._pose_yaw = lambda pose: 0.0
    node._preview_3d_snapshot = lambda: {'map_name': node._current_map_name,
        'map_load_id': node._current_map_load_id,
        'start_x': node._latest_amcl_pose.pose.pose.position.x,
        'start_y': node._latest_amcl_pose.pose.pose.position.y, 'start_yaw': 0.,
        'body_generation': node._robot_body_status['generation']}
    node._preview_3d_identity_matches = lambda snapshot: (
        snapshot['map_load_id'] == node._current_map_load_id
        and snapshot['body_generation'] == node._robot_body_status['generation'])
    node._preview_3d_snapshot_matches = lambda snapshot: True
    node._await_preview_future = lambda *_args: True
    planned = fixtures._path((1.2, 1.2), (3.2, 1.2))
    handle = NS(accepted=True, get_result_async=lambda: Future(NS(status=4, result=NS(path=planned))))
    node._preview_action_client = Mock()
    node._preview_action_client.server_is_ready.return_value = True
    node._preview_action_client.send_goal_async.return_value = Future(handle)
    node._action_client = Mock()
    node._action_client.server_is_ready.return_value = True
    node._single_goal_callback = Mock()
    node._nomotion_update_client = Mock()
    node._nomotion_update_client.service_is_ready.return_value = True
    node._nomotion_update_client.call_async.side_effect = lambda _req: (
        node._amcl_pose_callback(sample()), Future())[1]
    request = NS(goal=fixtures._pose(3.2, 1.2), map_name=node._current_map_name,
                 map_load_id=node._current_map_load_id)
    return node, request, clock


class ThreeDLocalizationRefreshTests(unittest.TestCase):
    def test_stale_preview_refreshes_then_only_plans(self):
        node, request, _clock = harness()
        result = node._preview_3d_goal_callback(request, NS())
        self.assertTrue(result.valid, result.message)
        node._nomotion_update_client.call_async.assert_called_once()
        node._preview_action_client.send_goal_async.assert_called_once()
        node._single_goal_callback.assert_not_called()
        node._action_client.send_goal_async.assert_not_called()
        self.assertFalse(node._preview_busy)

    def test_fresh_preview_does_not_force_an_unneeded_update(self):
        node, request, clock = harness()
        node._latest_amcl_received_at = clock.now
        node._latest_amcl_stamp_ns = 20_000_000_000
        result = node._preview_3d_goal_callback(request, NS())
        self.assertTrue(result.valid)
        node._nomotion_update_client.call_async.assert_not_called()

    def test_duplicate_stamp_cannot_finish_refresh(self):
        node, request, _clock = harness()
        node._nomotion_update_client.call_async.side_effect = lambda _req: (
            node._amcl_pose_callback(sample(10_000_000_000)), Future())[1]
        result = node._preview_3d_goal_callback(request, NS())
        self.assertFalse(result.valid)
        self.assertEqual(result.code, 'localization_refresh_timeout')
        node._preview_action_client.send_goal_async.assert_not_called()
        self.assertLessEqual(node._nomotion_update_client.call_async.call_count, 3)

    def test_delayed_new_but_old_source_stamp_cannot_finish_refresh(self):
        node, request, _clock = harness()
        node._nomotion_update_client.call_async.side_effect = lambda _req: (
            node._amcl_pose_callback(sample(11_000_000_000)), Future())[1]
        result = node._preview_3d_goal_callback(request, NS())
        self.assertFalse(result.valid)
        self.assertEqual(result.code, 'localization_refresh_timeout')
        node._preview_action_client.send_goal_async.assert_not_called()

    def test_service_failure_does_not_plan_or_queue_a_goal(self):
        node, request, _clock = harness()
        node._nomotion_update_client.call_async.side_effect = RuntimeError('service unavailable')
        result = node._preview_3d_goal_callback(request, NS())
        self.assertFalse(result.valid)
        self.assertEqual(result.code, 'localization_refresh_failed')
        self.assertIsNone(node._preview_token)
        node._single_goal_callback.assert_not_called()

    def test_map_or_body_change_aborts_waiting_preview(self):
        for change in ('map', 'body'):
            with self.subTest(change=change):
                node, request, clock = harness()
                node._nomotion_update_client.call_async.side_effect = lambda _req: Future()
                clock.on_sleep = (lambda: setattr(node, '_current_map_load_id', 'another-load')) if change == 'map' else (
                    lambda: node._robot_body_status['status'].update(emergency_latched=True))
                result = node._preview_3d_goal_callback(request, NS())
                self.assertFalse(result.valid)
                node._preview_action_client.send_goal_async.assert_not_called()
                node._single_goal_callback.assert_not_called()

    def test_refresh_does_not_hold_the_global_lock_while_waiting(self):
        node, request, clock = harness()
        node._nomotion_update_client.call_async.side_effect = lambda _req: Future()
        lock_states = []
        clock.on_sleep = lambda: (lock_states.append(node._lock._is_owned()),
                                  node._amcl_pose_callback(sample()))
        self.assertTrue(node._preview_3d_goal_callback(request, NS()).valid)
        self.assertTrue(lock_states)
        self.assertFalse(any(lock_states))

    def test_explicit_start_refresh_rechecks_same_preview_and_consumes_token(self):
        node, request, clock = harness()
        node._latest_amcl_received_at = clock.now
        node._latest_amcl_stamp_ns = 20_000_000_000
        first = node._preview_3d_goal_callback(request, NS())
        node._latest_amcl_received_at -= 5.133
        node._nomotion_update_client.call_async.side_effect = lambda _req: (
            node._amcl_pose_callback(sample(20_000_000_001)), Future())[1]
        node._single_goal_callback.side_effect = lambda _:setattr(node,'_mode','single')
        result = node._start_previewed_3d_goal_callback(NS(preview_id=first.preview_id), NS())
        self.assertTrue(result.success,result.message)
        self.assertEqual(result.code, 'accepted')
        self.assertIsNone(node._preview_token)
        node._single_goal_callback.assert_called_once()
        node._action_client.send_goal_async.assert_not_called()
        replay = node._start_previewed_3d_goal_callback(NS(preview_id=first.preview_id), NS())
        self.assertFalse(replay.success)
        self.assertEqual(replay.code, 'preview_missing')

    def test_cancel_during_refresh_prevents_late_preview_authorization(self):
        node, request, clock = harness()
        node._nomotion_update_client.call_async.side_effect = lambda _req: Future()
        clock.on_sleep = lambda: node._cancel_navigation_callback(NS())
        result = node._preview_3d_goal_callback(request, NS())
        self.assertFalse(result.valid)
        self.assertEqual(result.code, 'preview_canceled')
        self.assertIsNone(node._preview_token)
        node._preview_action_client.send_goal_async.assert_not_called()

    def test_planning_that_outlasts_pose_age_refreshes_again_before_returning(self):
        node, request, clock = harness()
        node._latest_amcl_received_at = clock.now
        node._latest_amcl_stamp_ns = 19_999_999_999
        planned = node._preview_action_client.send_goal_async.return_value
        def slow_plan(_goal):
            clock.now += 3.2
            node._robot_body_received_at = clock.now
            return planned
        node._preview_action_client.send_goal_async.side_effect = slow_plan
        result = node._preview_3d_goal_callback(request, NS())
        self.assertTrue(result.valid, result.message)
        node._nomotion_update_client.call_async.assert_called_once()
        node._single_goal_callback.assert_not_called()

    def test_missing_initial_localization_does_not_trigger_a_global_reset(self):
        node, request, _clock = harness()
        node._latest_amcl_pose = None
        node._latest_amcl_stamp_ns = 0
        result = node._preview_3d_goal_callback(request, NS())
        self.assertEqual(result.code, 'localization_missing')
        node._nomotion_update_client.call_async.assert_not_called()
        node._single_goal_callback.assert_not_called()

    def test_cancel_or_generation_change_after_refresh_cannot_authorize_plan(self):
        for change in ('cancel', 'body'):
            with self.subTest(change=change):
                node, request, _clock = harness()
                real_refresh = node._refresh_3d_preview_pose
                def late_change(*args):
                    result = real_refresh(*args)
                    if change == 'cancel':
                        node._cancel_navigation_callback(NS())
                    else:
                        node._robot_body_status['generation'] += 1
                    return result
                node._refresh_3d_preview_pose = late_change
                result = node._preview_3d_goal_callback(request, NS())
                self.assertFalse(result.valid)
                node._preview_action_client.send_goal_async.assert_not_called()

    def test_cancel_during_explicit_start_refresh_prevents_late_navigation(self):
        node, request, clock = harness()
        preview = node._preview_3d_goal_callback(request, NS())
        node._latest_amcl_received_at = clock.now - 5.133
        def update(_):
            node._amcl_pose_callback(sample(20_000_000_001))
            node._cancel_navigation_callback(NS())
            return Future()
        node._nomotion_update_client.call_async.side_effect = update
        blocked = node._start_previewed_3d_goal_callback(NS(preview_id=preview.preview_id), NS())
        self.assertFalse(blocked.success)
        node._single_goal_callback.assert_not_called()
        self.assertIsNone(node._preview_token)


if __name__ == '__main__':
    unittest.main()
