#!/usr/bin/env python3
"""Web single-goal refresh revalidates the explicit goal before dispatch.

Exercise the production ROS callback methods with fake ROS boundary objects so
these safety regressions can also run on a development machine without ROS.
"""

from __future__ import annotations

import ast
import copy
import math
from pathlib import Path
import textwrap
import threading
import time
from types import SimpleNamespace


SOURCE = (
    Path(__file__).resolve().parents[1] / 'lzp_service' /
    'route_navigation_manager.py')


class FakeFuture:
    def __init__(self, result=None, error=None, done=True):
        self._result = result
        self._error = error
        self._done = done

    def done(self):
        return self._done

    def result(self):
        if self._error:
            raise self._error
        return self._result


class FakeActionClient:
    def __init__(self):
        self.ready = True
        self.sent = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent.append(goal)
        return SimpleNamespace(add_done_callback=lambda callback: None)


class FakeGoalHandle:
    def __init__(self):
        self.cancel_requested = False
        self.accepted = True

    def cancel_goal_async(self):
        self.cancel_requested = True
        return SimpleNamespace(add_done_callback=lambda callback: None)

    def get_result_async(self):
        return SimpleNamespace(add_done_callback=lambda callback: None)


class FailingCancelGoalHandle(FakeGoalHandle):
    def cancel_goal_async(self):
        raise RuntimeError('Nav2 cancellation unavailable')


class GoalStatus:
    STATUS_UNKNOWN = 0
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


class FakePublisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(getattr(message, 'data', message))


class FakeNomotionClient:
    def __init__(self):
        self.ready = True
        self.requests = 0
        self.error = None

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests += 1
        return FakeFuture(error=self.error)


class EmptyService:
    class Request:
        pass


class NavigateToPose:
    class Goal:
        pass


class String:
    def __init__(self, data=''):
        self.data = data


class Twist:
    pass


class GoalValidationResult:
    def __init__(self, allowed, code, message):
        self.allowed = allowed
        self.code = code
        self.message = message


def load_manager_methods():
    source = SOURCE.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = next(node for node in tree.body if (
        isinstance(node, ast.ClassDef) and
        node.name == 'RouteNavigationManager'))
    method_names = (
        '_single_goal_callback', '_timer_callback', '_amcl_pose_callback',
        '_cancel_navigation_callback', '_current_map_callback',
        '_goal_result_callback', '_handle_goal_terminal',
        '_goal_response_callback',
        '_cancel_active_goal', '_cancel_response_callback',
        '_record_managed_goal_event',
        '_goal_preflight', '_goal_rejection_status', '_publish_goal_status',
        '_log_goal_event', '_goal_ui_event_callback',
        '_request_stop', '_finish_task', '_clear_preflight_wait', '_send_pose',
        '_send_zero_velocity', '_dispatch_web_single_pose',
        '_begin_single_localization_refresh',
        '_drive_single_localization_refresh', '_drive_stationary_localization_update',
        '_clear_single_localization_refresh',
        '_fail_single_localization_refresh',
    )
    fragments = []
    for name in method_names:
        method = next((node for node in manager.body if (
            isinstance(node, ast.FunctionDef) and node.name == name)), None)
        if method is not None:
            fragment = ast.get_source_segment(source, method)
            if any(isinstance(item, ast.Name) and item.id == 'staticmethod'
                   for item in method.decorator_list):
                fragment = '@staticmethod\n' + fragment
            fragments.append(textwrap.indent(fragment, '    '))
    namespace = {
        'copy': copy,
        'json': __import__('json'),
        'math': math,
        'time': time,
        'String': String,
        'Twist': Twist,
        'EmptyService': EmptyService,
        'NavigateToPose': NavigateToPose,
        'GoalStatus': GoalStatus,
        'MAP_2D': 'map_2d',
        'uses_terrain': lambda mode: mode != 'map_2d',
        'GoalValidationResult': GoalValidationResult,
    }
    exec('from __future__ import annotations\nclass Manager:\n' +
         '\n'.join(fragments), namespace)
    return namespace['Manager']


def pose(stamp_ns=20_000_000_000, x=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id='map',
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000)),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


def amcl(stamp_ns, x=0.0):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000)),
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))


def manager():
    node = load_manager_methods()()
    node._lock = threading.RLock()
    node._current_map_name = 'ceshi3_remap1'
    node._current_map_load_id = 'load-1'
    node._navigation_mode = 'map_2d'
    node._active_navigation_mode = ''
    node._mode = 'idle'
    node._state = 'stopped'
    node._status_message = ''
    node._campus_mission_active = False
    node._lio_recovering = False
    node._owner_token = ''
    node._sending = False
    node._active_goal_handle = None
    node._active_pose = None
    node._pending_single_pose = None
    node._cancel_reason = None
    node._cancel_in_progress = False
    node._latest_amcl_pose = None
    node._latest_amcl_received_at = None
    node._latest_amcl_stamp_ns = 0
    node._goal_preflight_pose_max_age = 3.0
    node._goal_preflight_wait_timeout = 1.0
    node._nomotion_update_interval = 0.01
    node._nomotion_update_max_requests = 2
    node._traversability_grid = None
    node._traversability_map_name = ''
    node._terrain_costmap_ready_map_name = ''
    node._terrain_costmap_ready_load_id = ''
    node._terrain_costmap_ready_stamp_ns = 0
    node._pending_traversability_stamp_ns = 0
    node._pending_traversability_grid = None
    node._recovery_pose = None
    node._retry_current_after_recovery = False
    node._last_goal_status = ''
    node._goal_token = 0
    node._goal_context = None
    node._single_refresh_map_name = ''
    node._single_refresh_baseline_stamp_ns = 0
    node._single_refresh_started_at = None
    node._single_refresh_deadline = None
    node._single_refresh_next_nomotion_at = None
    node._single_refresh_nomotion_requests = 0
    node._single_refresh_future = None
    node._nav2_ready = True
    node._action_client = FakeActionClient()
    node._nomotion_update_client = FakeNomotionClient()
    node._goal_status_publisher = FakePublisher()
    node._zero_velocity_publisher = FakePublisher()
    node._navigation_speed_ready = lambda: True
    node._drive_navigation_speed_sync = lambda: None
    node._speed_applied = set()
    node._inflation_ready_for_navigation = lambda: True
    # These tests isolate the legacy AMCL refresh protocol. The authority
    # adapter has its own integration tests; do not omit its preflight seam.
    node._global_localization_check = lambda: None
    node._drive_inflation_radius_sync = lambda: None
    node._drive_auto_recovery = lambda: None
    node._publish_all_status = lambda: None
    node._publish_route_log = lambda text: None
    node._effective_navigation_mode = lambda: (
        node._active_navigation_mode or node._navigation_mode)
    node.get_logger = lambda: SimpleNamespace(
        warning=lambda text: None,
        error=lambda text: None)
    return node


def test_stale_click_dispatches_retained_goal_only_after_new_sample():
    node = manager()
    old = amcl(10_000_000_000)
    node._latest_amcl_pose = old
    node._latest_amcl_stamp_ns = 10_000_000_000
    node._latest_amcl_received_at = time.monotonic() - 4.0

    node._single_goal_callback(pose(x=3.0))

    assert node._goal_status_publisher.values[-1] == (
        'goal_refreshing_localization')
    assert node._action_client.sent == []
    assert node._pending_single_pose is None
    assert node._nomotion_update_client.requests >= 1

    node._amcl_pose_callback(amcl(10_000_000_001))
    node._drive_single_localization_refresh()

    assert len(node._action_client.sent) == 1
    assert node._action_client.sent[0].pose.pose.position.x == 3.0
    assert node._single_refresh_pose is None
    assert node._mode == 'single'
    node._drive_single_localization_refresh()
    assert len(node._action_client.sent) == 1


def test_duplicate_amcl_stamp_cannot_finish_refresh():
    node = manager()
    node._latest_amcl_pose = amcl(10_000_000_000)
    node._latest_amcl_stamp_ns = 10_000_000_000
    node._latest_amcl_received_at = time.monotonic() - 4.0
    node._single_goal_callback(pose(x=3.0))

    node._amcl_pose_callback(amcl(10_000_000_000, x=99.0))
    node._drive_single_localization_refresh()

    assert node._goal_status_publisher.values[-1] == (
        'goal_refreshing_localization')
    assert node._action_client.sent == []


def test_refresh_timeout_and_service_failure_clear_stale_goal():
    node = manager()
    node._single_goal_callback(pose(x=3.0))
    node._single_refresh_deadline = time.monotonic() - 0.01
    node._drive_single_localization_refresh()

    assert node._goal_status_publisher.values[-1] == (
        'goal_localization_refresh_failed')
    assert node._pending_single_pose is None
    assert node._action_client.sent == []

    failed_service = manager()
    failed_service._nomotion_update_client.error = RuntimeError('AMCL offline')
    failed_service._single_goal_callback(pose(x=4.0))
    failed_service._drive_single_localization_refresh()

    assert failed_service._goal_status_publisher.values[-1] == (
        'goal_localization_refresh_failed')
    assert failed_service._pending_single_pose is None
    assert failed_service._action_client.sent == []


def test_cancel_during_idle_refresh_disarms_it():
    node = manager()
    node._single_goal_callback(pose(x=3.0))

    node._cancel_navigation_callback(SimpleNamespace())
    node._amcl_pose_callback(amcl(11_000_000_000))
    node._drive_single_localization_refresh()

    assert node._goal_status_publisher.values[-1] == 'goal_canceled'
    assert node._action_client.sent == []


def test_map_replacement_or_failed_unload_disarms_pending_and_refresh():
    for update in (
        SimpleNamespace(success=True, map_name='other_map'),
        SimpleNamespace(success=False, map_name=''),
        SimpleNamespace(
            success=True,
            map_name='ceshi3_remap1',
            message='Successfully loaded and confirmed map: ceshi3_remap1'),
        SimpleNamespace(
            success=True,
            map_name='ceshi3_remap1',
            message='Successfully loaded map: ceshi3_remap1'),
    ):
        node = manager()
        node._single_goal_callback(pose(x=3.0))
        node._pending_single_pose = pose(x=4.0)
        stale_grid = object()
        node._traversability_grid = stale_grid
        node._traversability_map_name = 'ceshi3_remap1'
        node._pending_traversability_grid = stale_grid
        node._pending_traversability_stamp_ns = 123
        node._terrain_costmap_ready_map_name = 'ceshi3_remap1'
        node._terrain_costmap_ready_stamp_ns = 123

        node._current_map_callback(update)
        node._amcl_pose_callback(amcl(11_000_000_000))
        node._drive_single_localization_refresh()

        assert node._pending_single_pose is None
        assert node._traversability_grid is None
        assert node._pending_traversability_grid is None
        assert node._terrain_costmap_ready_stamp_ns == 0
        assert node._action_client.sent == []
        assert node._goal_status_publisher.values[-1] == (
            'goal_localization_refresh_failed')


def test_waiting_nav2_rechecks_freshness_before_dispatch():
    node = manager()
    node._latest_amcl_pose = amcl(10_000_000_000)
    node._latest_amcl_stamp_ns = 10_000_000_000
    node._latest_amcl_received_at = time.monotonic()
    node._action_client.ready = False
    node._single_goal_callback(pose(x=3.0))
    assert node._pending_single_pose is not None
    node._latest_amcl_received_at = time.monotonic() - 4.0
    node._action_client.ready = True

    node._timer_callback()

    assert node._action_client.sent == []
    assert node._pending_single_pose is None
    assert node._goal_status_publisher.values[-1] == (
        'goal_refreshing_localization')


def test_repeated_clicks_do_not_restart_refresh_or_start_goal():
    node = manager()
    node._latest_amcl_pose = amcl(10_000_000_000)
    node._latest_amcl_stamp_ns = 10_000_000_000
    node._latest_amcl_received_at = time.monotonic() - 4.0

    node._single_goal_callback(pose(x=3.0))
    deadline = node._single_refresh_deadline
    requests = node._nomotion_update_client.requests
    node._single_goal_callback(pose(x=4.0))

    assert node._single_refresh_deadline == deadline
    assert node._nomotion_update_client.requests == requests
    assert node._action_client.sent == []

    node._amcl_pose_callback(amcl(10_000_000_001))
    # A second click cannot replace the explicit goal already being refreshed.
    node._single_goal_callback(pose(x=5.0))
    assert len(node._action_client.sent) == 1
    assert node._action_client.sent[0].pose.pose.position.x == 3.0


def test_stale_replacement_waits_for_old_async_cancel_then_revalidates():
    node = manager()
    old_handle = FakeGoalHandle()
    node._mode = 'single'
    node._state = 'navigating'
    node._active_goal_handle = old_handle
    node._goal_context = 'single'
    node._active_pose = pose(x=1.0)
    node._latest_amcl_pose = amcl(10_000_000_000)
    node._latest_amcl_stamp_ns = 10_000_000_000
    node._latest_amcl_received_at = time.monotonic() - 4.0

    node._single_goal_callback(pose(x=3.0))
    assert old_handle.cancel_requested
    assert node._mode == 'single'
    assert node._pending_single_pose is None
    assert node._goal_status_publisher.values[-1] == (
        'goal_refreshing_localization')

    node._amcl_pose_callback(amcl(10_000_000_001))
    node._drive_single_localization_refresh()
    assert node._goal_status_publisher.values[-1] == (
        'goal_refreshing_localization')
    assert node._action_client.sent == []

    node._goal_result_callback(FakeFuture(
        result=SimpleNamespace(status=GoalStatus.STATUS_CANCELED)),
        old_handle)
    assert node._mode == 'single'
    assert len(node._action_client.sent) == 1
    assert node._action_client.sent[0].pose.pose.position.x == 3.0
    assert 'goal_canceled' not in node._goal_status_publisher.values


def test_unconfirmed_old_goal_cancel_times_out_without_claiming_stopped():
    node = manager()
    old_handle = FakeGoalHandle()
    node._mode = 'single'
    node._state = 'navigating'
    node._active_goal_handle = old_handle
    node._goal_context = 'single'
    node._active_pose = pose(x=1.0)
    node._latest_amcl_pose = amcl(10_000_000_000)
    node._latest_amcl_stamp_ns = 10_000_000_000
    node._latest_amcl_received_at = time.monotonic() - 4.0

    node._single_goal_callback(pose(x=3.0))
    node._single_refresh_deadline = time.monotonic() - 0.01
    node._drive_single_localization_refresh()

    assert node._goal_status_publisher.values[-1] == (
        'goal_localization_refresh_failed')
    assert node._mode == 'single'
    assert old_handle.cancel_requested
    assert node._action_client.sent == []
    assert node._pending_single_pose is None
    assert '旧导航取消未确认' in node._status_message

    # A later AMCL update does not make the still-active old goal safe to
    # replace.  Its eventual canceled result must not auto-send a new goal.
    node._amcl_pose_callback(amcl(10_000_000_001))
    node._single_goal_callback(pose(x=5.0))
    assert node._pending_single_pose is None
    assert node._action_client.sent == []
    assert node._goal_status_publisher.values[-1] == (
        'goal_rejected_navigation_stopping')
    node._goal_result_callback(FakeFuture(
        result=SimpleNamespace(status=GoalStatus.STATUS_CANCELED)),
        old_handle)
    assert node._mode == 'idle'
    assert node._action_client.sent == []


def test_map_switch_cancels_active_old_goal_and_forgets_pending_target():
    node = manager()
    old_handle = FakeGoalHandle()
    node._mode = 'single'
    node._state = 'navigating'
    node._active_goal_handle = old_handle
    node._goal_context = 'single'
    node._active_pose = pose(x=1.0)
    node._pending_single_pose = pose(x=3.0)

    node._current_map_callback(SimpleNamespace(
        success=True, map_name='other_map'))

    assert node._pending_single_pose is None
    assert old_handle.cancel_requested
    assert node._mode == 'single'
    assert node._action_client.sent == []
    node._goal_result_callback(FakeFuture(
        result=SimpleNamespace(status=GoalStatus.STATUS_CANCELED)),
        old_handle)
    assert node._mode == 'idle'
    assert node._action_client.sent == []


def test_cancel_request_exception_reports_failure_without_claiming_idle():
    node = manager()
    old_handle = FailingCancelGoalHandle()
    node._mode = 'single'
    node._state = 'navigating'
    node._active_goal_handle = old_handle
    node._goal_context = 'single'
    node._active_pose = pose(x=1.0)
    node._latest_amcl_pose = amcl(10_000_000_000)
    node._latest_amcl_stamp_ns = 10_000_000_000
    node._latest_amcl_received_at = time.monotonic() - 4.0

    node._single_goal_callback(pose(x=3.0))

    assert node._goal_status_publisher.values[-1] == (
        'goal_localization_refresh_failed')
    assert node._mode == 'single'
    assert node._active_goal_handle is old_handle
    assert node._pending_single_pose is None
    assert node._action_client.sent == []
    assert '取消' in node._status_message


def test_late_old_goal_acceptance_during_refresh_does_not_claim_navigating():
    node = manager()
    node._mode = 'single'
    node._state = 'sending'
    node._sending = True
    node._goal_token = 7
    node._goal_context = 'single'
    node._active_pose = pose(x=1.0)
    node._latest_amcl_pose = amcl(10_000_000_000)
    node._latest_amcl_stamp_ns = 10_000_000_000
    node._latest_amcl_received_at = time.monotonic() - 4.0

    node._single_goal_callback(pose(x=3.0))
    old_handle = FakeGoalHandle()
    node._goal_response_callback(FakeFuture(result=old_handle), 7)

    assert old_handle.cancel_requested
    assert node._state == 'stopping'
    assert node._goal_status_publisher.values[-1] == (
        'goal_refreshing_localization')
    assert 'navigating' not in node._goal_status_publisher.values
    assert node._pending_single_pose is None
    assert node._action_client.sent == []


if __name__ == '__main__':
    for name, test in sorted(globals().copy().items()):
        if name.startswith('test_') and callable(test):
            test()
            print(f'PASS {name}')


def test_stationary_updates_are_only_for_idle_2d_localization():
    for mode, execution, expected in [
        ('map_2d', 'idle', 1), ('terrain_3d', 'idle', 0),
        ('map_2d', 'route', 0), ('map_2d', 'single', 0),
    ]:
        node = manager()
        node._navigation_mode, node._mode = mode, execution
        node._latest_amcl_pose = amcl(20_000_000_000)
        node._drive_stationary_localization_update()
        node._drive_stationary_localization_update()
        assert node._nomotion_update_client.requests == expected


def test_stationary_update_does_not_compete_with_explicit_refresh():
    node = manager()
    node._latest_amcl_pose = amcl(20_000_000_000)
    node._single_refresh_map_name = node._current_map_name
    node._drive_stationary_localization_update()
    assert node._nomotion_update_client.requests == 0
