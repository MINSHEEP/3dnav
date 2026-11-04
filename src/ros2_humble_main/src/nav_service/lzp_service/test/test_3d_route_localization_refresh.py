"""Shared navigation/patrol route refresh with real methods, fake ROS/time."""
from types import MethodType, SimpleNamespace as NS
import copy
import hashlib
import ast
from pathlib import Path
import unittest
from unittest.mock import Mock

from test_3d_localization_refresh import harness as single_harness, Future, sample
import test_3d_goal_preview as fixtures


def harness():
    node, request, clock = single_harness()
    names = ('_preview_3d_route_callback', '_plan_3d_route_segments',
             '_validate_3d_route_segments', '_route_preview_points', '_route_digest',
             '_preview_3d_snapshot', '_preview_3d_snapshot_matches', '_pose_yaw',
             '_verify_3d_route_start_ticket',
             '_reserve_3d_inspection_route_callback', '_prepare_3d_route_start',
             '_reserve_3d_inspection_route_checked', '_prepare_3d_route_start_checked')
    implementation = fixtures._load_manager_methods(*names)
    for name in names:
        function = getattr(implementation, name)
        function.__globals__.update(time=clock, hashlib=hashlib)
        setattr(node, name, function if name in ('_route_preview_points', '_route_digest', '_pose_yaw')
                else MethodType(function, node))
    clock.time_ns = lambda: 30_000_000_000
    node._terrain_costmap_ready_stamp_ns = 500
    node._navigation_mode_store = NS(snapshot=lambda: {'version': 1})
    node._inflation_sync = NS(snapshot=lambda: {'version': 1})
    points = [{'id': 1, 'x': 3.2, 'y': 1.2}]
    route = {'id': 8, 'name': 'patrol', 'map_name': 'garden', 'points': points}
    node._database = NS(get_route=lambda _: copy.deepcopy(route))
    node._terrain_capability = NS(planar_execution_blockers=lambda *a, **kw: [])
    node._route_pose = lambda point: fixtures._pose(point['x'], point['y'])
    node._inspection_route_authorization = None
    node._inspection_task_active = False
    node._inspection_task_paused = False
    node._inspection_task_id = ''
    node._owner_token = ''
    node.get_logger = lambda: Mock()
    request = NS(route_id=8, start_point_id=0, point_limit=0, reverse=False,
        loop_count=1, continuous=False, map_name='garden', map_load_id='load-1',
        preview_id='', owner_token='', navigation_mode='terrain_3d',
        authorization_generation=0, request_deadline_unix_ns=40_000_000_000,
        bidirectional=False)
    return node, request, clock


class RouteRefreshTests(unittest.TestCase):
    def advance_stationary(self, node, clock, seconds=4):
        clock.now += seconds
        node._robot_body_received_at = clock.now
        node._nomotion_update_client.call_async.side_effect = lambda _: (
            node._amcl_pose_callback(sample(21_000_000_000)), Future())[1]
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=21_000_000_000, to_msg=lambda: 'stamp'))

    def assert_no_motion(self, node):
        node._action_client.send_goal_async.assert_not_called()
        node._single_goal_callback.assert_not_called()

    def test_both_pages_shared_preview_refreshes_before_planning(self):
        node, request, _ = harness()
        response = node._preview_3d_route_callback(request, NS())
        self.assertTrue(response.valid, response.message)
        node._nomotion_update_client.call_async.assert_called_once()
        self.assert_no_motion(node)

    def test_duplicate_old_stamp_times_out_and_preserves_route(self):
        node, request, _ = harness()
        node._nomotion_update_client.call_async.side_effect = lambda _: Future()
        before = node._database.get_route(8)
        response = node._preview_3d_route_callback(request, NS())
        self.assertEqual(response.code, 'localization_refresh_timeout')
        self.assertFalse(response.valid)
        self.assertEqual(node._database.get_route(8), before)
        self.assertIsNone(node._preview_token)
        self.assert_no_motion(node)

    def test_manual_start_refresh_revalidates_single_use_confirmation(self):
        node, request, clock = harness()
        result = node._preview_3d_route_callback(request, NS())
        self.assertTrue(result.valid, result.message)
        request.preview_id = result.preview_id
        clock.now += 4
        node._robot_body_received_at = clock.now
        stamp = 20_000_000_000
        node._latest_amcl_stamp_ns = stamp
        node._nomotion_update_client.call_async.side_effect = lambda _: (
            node._amcl_pose_callback(sample(21_000_000_000)), Future())[1]
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=21_000_000_000, to_msg=lambda: 'stamp'))
        result, ticket = node._prepare_3d_route_start(request)
        self.assertTrue(result.allowed, result.message)
        self.assertTrue(node._verify_3d_route_start_ticket(ticket, node._database.get_route(8)))
        node._preview_request_generation += 1
        self.assertFalse(node._verify_3d_route_start_ticket(ticket, node._database.get_route(8)),
                         'cancel after revalidation must still prevent dispatch')
        self.assertIsNone(node._preview_token)
        self.assert_no_motion(node)
        replay, ticket = node._prepare_3d_route_start(request)
        self.assertFalse(replay.allowed)
        self.assertIsNone(ticket)

    def test_manual_refresh_rejects_changed_evidence_and_expired_confirmation(self):
        for change in ('position', 'yaw', 'map', 'terrain', 'body', 'route',
                       'cancel', 'replacement', 'expiry', 'timeout', 'segments'):
            with self.subTest(change=change):
                node, request, clock = harness()
                preview = node._preview_3d_route_callback(request, NS())
                self.assertTrue(preview.valid, preview.message)
                request.preview_id = preview.preview_id
                self.advance_stationary(node, clock)
                def update(_):
                    pose = sample(21_000_000_000)
                    if change == 'position':
                        pose.pose.pose.position.x += 0.3
                    elif change == 'yaw':
                        pose.pose.pose.orientation.z = 0.5
                        pose.pose.pose.orientation.w = 0.8660254
                    node._amcl_pose_callback(pose)
                    if change == 'map':
                        node._current_map_load_id = 'another-load'
                    elif change == 'terrain':
                        node._terrain_costmap_ready_stamp_ns += 1
                    elif change == 'body':
                        node._robot_body_status['generation'] += 1
                    elif change == 'route':
                        route = node._database.get_route(8)
                        route['points'][0]['x'] += 1
                        node._database.get_route = lambda _: route
                    elif change == 'cancel':
                        node._preview_request_generation += 1
                    elif change == 'replacement':
                        node._preview_token = {'id': 'newer-preview'}
                    elif change == 'expiry':
                        clock.now += 30
                    elif change == 'segments':
                        node._validate_3d_route_segments = lambda *a: (
                            NS(allowed=False, code='blocked', message='blocked'), None)
                    return Future()
                node._nomotion_update_client.call_async.side_effect = (
                    (lambda _: Future()) if change == 'timeout' else update)
                result, ticket = node._prepare_3d_route_start(request)
                self.assertFalse(result.allowed, change)
                self.assertIsNone(ticket)
                self.assert_no_motion(node)

    def test_2d_and_gangben_do_not_gain_manual_refresh_requirements(self):
        for mode, body in (('map_2d', 'xingji'), ('terrain_3d', 'gangben_zsl1')):
            node, request, _ = harness()
            request.navigation_mode = mode
            node._robot_body_status['config']['selected'] = body
            result, ticket = node._prepare_3d_route_start(request)
            self.assertTrue(result.allowed)
            self.assertIsNone(ticket)
            node._nomotion_update_client.call_async.assert_not_called()

    def test_inspection_start_reservation_refreshes_without_motion(self):
        node, request, clock = harness()
        preview = node._preview_3d_route_callback(request, NS())
        self.assertTrue(preview.valid)
        request.preview_id = preview.preview_id
        request.owner_token = 'inspection-1'
        self.advance_stationary(node, clock)
        result = node._reserve_3d_inspection_route_callback(request, NS())
        self.assertTrue(result.success, result.message)
        self.assertEqual(node._inspection_route_authorization['owner_token'], 'inspection-1')
        self.assert_no_motion(node)

    def test_inspection_reservation_timeout_grants_no_lease(self):
        node, request, clock = harness()
        preview = node._preview_3d_route_callback(request, NS())
        request.preview_id, request.owner_token = preview.preview_id, 'inspection-1'
        self.advance_stationary(node, clock)
        node._nomotion_update_client.call_async.side_effect = lambda _: Future()
        result = node._reserve_3d_inspection_route_callback(request, NS())
        self.assertFalse(result.success)
        self.assertIsNone(node._inspection_route_authorization)
        self.assertIsNone(node._preview_token)
        self.assertEqual(len(node._database.get_route(8)['points']), 1)
        self.assert_no_motion(node)

    def test_inspection_reservation_rejects_corrected_pose_outside_old_preview(self):
        node, request, clock = harness()
        preview = node._preview_3d_route_callback(request, NS())
        request.preview_id, request.owner_token = preview.preview_id, 'inspection-1'
        self.advance_stationary(node, clock)
        def update(_):
            corrected = sample(21_000_000_000)
            corrected.pose.pose.position.x += 0.3
            node._amcl_pose_callback(corrected)
            return Future()
        node._nomotion_update_client.call_async.side_effect = update
        result = node._reserve_3d_inspection_route_callback(request, NS())
        self.assertFalse(result.success)
        self.assertIsNone(node._inspection_route_authorization)
        self.assert_no_motion(node)

    def patrol(self):
        node, request, clock = harness()
        preview = node._preview_3d_route_callback(request, NS())
        request.preview_id, request.owner_token = preview.preview_id, 'inspection-1'
        self.assertTrue(node._reserve_3d_inspection_route_callback(request, NS()).success)
        node._inspection_task_active = True
        node._inspection_task_id = request.owner_token
        return node, request, clock

    def test_inspection_camera_warmup_refreshes_and_replans_valid_lease(self):
        node, request, clock = self.patrol()
        self.advance_stationary(node, clock)
        result, ticket = node._prepare_3d_route_start(request)
        self.assertTrue(result.allowed, result.message)
        self.assertEqual(ticket['owner_token'], request.owner_token)
        self.assertEqual(node._preview_action_client.send_goal_async.call_count, 2)
        self.assert_no_motion(node)

    def test_patrol_pause_cancel_or_new_generation_cannot_reuse_refresh(self):
        for change in ('pause', 'cancel', 'generation'):
            with self.subTest(change=change):
                node, request, clock = self.patrol()
                self.advance_stationary(node, clock)
                def update(_):
                    node._amcl_pose_callback(sample(21_000_000_000))
                    if change == 'pause':
                        node._inspection_task_paused = True
                    elif change == 'cancel':
                        node._inspection_route_authorization = None
                    else:
                        node._inspection_route_authorization['generation'] += 1
                    return Future()
                node._nomotion_update_client.call_async.side_effect = update
                result, ticket = node._prepare_3d_route_start(request)
                self.assertFalse(result.allowed)
                self.assertIsNone(ticket)
                self.assert_no_motion(node)

    def test_inspection_long_replanning_refreshes_after_camera_warmup(self):
        node, request, clock = self.patrol()
        self.advance_stationary(node, clock)
        real_plan = node._plan_3d_route_segments
        def plan(*args):
            result = real_plan(*args)
            clock.now += 4
            node._robot_body_received_at = clock.now
            node.get_clock = lambda: NS(now=lambda: NS(
                nanoseconds=22_000_000_000, to_msg=lambda: 'stamp'))
            node._nomotion_update_client.call_async.side_effect = lambda _: (
                node._amcl_pose_callback(sample(22_000_000_000)), Future())[1]
            return result
        node._plan_3d_route_segments = plan
        result, ticket = node._prepare_3d_route_start(request)
        self.assertTrue(result.allowed, result.message)
        self.assertEqual(ticket['owner_token'], 'inspection-1')
        self.assertEqual(node._nomotion_update_client.call_async.call_count, 3)
        self.assert_no_motion(node)

    def test_inspection_reservation_wait_covers_bounded_refresh(self):
        service = Path(fixtures.SOURCE_PATH).with_name('inspection_task_manager.py')
        tree = ast.parse(service.read_text(encoding='utf-8'))
        waits = [call for call in ast.walk(tree) if isinstance(call, ast.Call)
                 and isinstance(call.func, ast.Attribute) and call.func.attr == 'wait'
                 and call.args and isinstance(call.args[0], ast.Constant)
                 and call.args[0].value == 12.0]
        self.assertEqual(len(waits), 1)

    def test_pause_resume_during_long_patrol_plan_cannot_promote_old_request(self):
        node, request, _ = self.patrol()
        real_plan = node._plan_3d_route_segments
        def plan(*args):
            result = real_plan(*args)
            # A pause followed by resume leaves task active, but must fence
            # the already in-flight Start request from the previous generation.
            node._inspection_route_authorization['generation'] += 2
            return result
        node._plan_3d_route_segments = plan
        result, ticket = node._prepare_3d_route_start(request)
        self.assertTrue(result.allowed)
        self.assertFalse(node._verify_3d_route_start_ticket(
            ticket, node._database.get_route(8)))
        self.assert_no_motion(node)

    def test_long_route_planning_refreshes_at_end(self):
        node, request, clock = harness()
        real_plan = node._plan_3d_route_segments
        def plan(*args):
            result = real_plan(*args)
            self.advance_stationary(node, clock)
            return result
        node._plan_3d_route_segments = plan
        result = node._preview_3d_route_callback(request, NS())
        self.assertTrue(result.valid, result.message)
        self.assertEqual(node._nomotion_update_client.call_async.call_count, 2)
        self.assert_no_motion(node)

    def test_no_initial_pose_does_not_guess_or_reset_localization(self):
        node, request, _ = harness()
        node._latest_amcl_pose = None
        result = node._preview_3d_route_callback(request, NS())
        self.assertEqual(result.code, 'localization_missing')
        self.assertFalse(result.valid)
        node._nomotion_update_client.call_async.assert_not_called()
        self.assert_no_motion(node)

    def test_map_route_or_cancel_change_during_refresh_invalidates_preview(self):
        for change in ('map', 'route', 'cancel'):
            with self.subTest(change=change):
                node, request, _ = harness()
                def update(_):
                    node._amcl_pose_callback(sample())
                    if change == 'map':
                        node._current_map_load_id = 'new'
                    elif change == 'route':
                        previous = node._database.get_route
                        def changed(route_id):
                            route = previous(route_id)
                            route['name'] = 'changed'
                            return route
                        node._database.get_route = changed
                    else:
                        node._preview_request_generation += 1
                    return Future()
                node._nomotion_update_client.call_async.side_effect = update
                result = node._preview_3d_route_callback(request, NS())
                self.assertFalse(result.valid)
                self.assertIsNone(node._preview_token)
                self.assert_no_motion(node)


if __name__ == '__main__':
    unittest.main()
