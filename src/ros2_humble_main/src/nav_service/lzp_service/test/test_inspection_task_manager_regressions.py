"""Regression coverage for field failures in the inspection manager."""

import ast
from concurrent.futures import Future
from datetime import datetime
import inspect
import json
import math
from pathlib import Path
import threading
import textwrap
import time
from types import SimpleNamespace
from typing import Any, Dict, Iterable, List


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / 'lzp_service'
    / 'inspection_task_manager.py'
)


class _String:
    def __init__(self, data=''):
        self.data = data


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _RclpyCompatibleLogger:
    """Reproduce rclpy's fixed-severity-per-call-site contract."""

    def __init__(self):
        self._severities = {}
        self.records = []

    def _record(self, level, message):
        caller = inspect.currentframe().f_back.f_back
        call_site = (caller.f_code.co_filename, caller.f_lineno)
        previous = self._severities.setdefault(call_site, level)
        if previous != level:
            raise ValueError('Logger severity cannot be changed between calls.')
        self.records.append((level, message))

    def info(self, message):
        self._record('info', message)

    def warning(self, message):
        self._record('warning', message)

    def error(self, message):
        self._record('error', message)


def _load_real_log_method():
    source = MODULE_PATH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'InspectionTaskManager'
    )
    method = next(
        node for node in manager.body
        if isinstance(node, ast.FunctionDef) and node.name == '_log'
    )
    method_source = ast.get_source_segment(source, method)
    namespace = {
        'String': _String,
        '_json': lambda value: json.dumps(value, ensure_ascii=False),
        'datetime': datetime,
    }
    exec(
        'class LogHarness:\n' + textwrap.indent(method_source, '    '),
        namespace,
    )
    return namespace['LogHarness']


def _load_confirmed_detection_filter():
    source = MODULE_PATH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == '_confirmed_detections'
    )
    namespace = {
        'Any': Any,
        'Dict': Dict,
        'Iterable': Iterable,
        'List': List,
        'math': math,
        'MIN_CONFIRMED_DETECTION_CONFIDENCE': 0.50,
    }
    exec(ast.get_source_segment(source, function), namespace)
    return namespace['_confirmed_detections']


def _load_manager_methods(*method_names):
    """Load selected production methods without importing ROS2 on Windows."""
    source = MODULE_PATH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'InspectionTaskManager'
    )
    methods = []
    for method_name in method_names:
        methods.append(next(
            node for node in manager.body
            if isinstance(node, ast.FunctionDef) and node.name == method_name
        ))
    namespace = {
        'time': time,
        'threading': threading,
        'utc_now': lambda: 'now',
        'RouteExecutionStatus': object,
        'CancelGoal': type('CancelGoal', (), {'Request': type('Request', (), {})}),
        'Revoke3DInspectionRoute': type(
            'Revoke3DInspectionRoute', (),
            {'Request': type('Request', (), {})}),
        'ControlRoute': type(
            'ControlRoute', (), {'Request': type('Request', (), {})}),
        'StartRoute': type(
            'StartRoute', (), {'Request': type('Request', (), {})}),
        'TERRAIN_3D': 'terrain_3d',
        'normalize_navigation_mode': lambda value: (
            str(value) if str(value) in ('map_2d', 'terrain_3d')
            else (_ for _ in ()).throw(ValueError('invalid navigation mode'))),
    }
    body = 'class ManagerHarness:\n' + ''.join(
        textwrap.indent(ast.get_source_segment(source, method), '    ') + '\n'
        for method in methods
    )
    exec(body, namespace)
    return namespace['ManagerHarness']


def test_patrol_return_and_next_round_use_the_same_frozen_3d_navigation_service():
    harness = _load_manager_methods(
        '_begin_patrol_loop', '_begin_reposition', '_start_route',
        '_dispatch_pending_route_request')()
    harness._route = {'points': [{'id': 7}, {'id': 11}]}
    harness._state = dict(task_id='inspection-3d', route_id=9,
        navigation_mode='terrain_3d', current_loop=1, bidirectional=False)
    # A later settings message cannot replace the task's frozen 3D scope.
    harness._selected_navigation_mode = 'map_2d'
    harness._task = {'request': {'previewed_3d': True}}
    harness._3d_authorization_generation = 3
    harness._pending_route_start = harness._pending_route_control = None
    harness._route_request_future = None
    harness._is_active = lambda: True
    harness._persist = lambda: None
    requests = []

    def start(request):
        requests.append(request)
        return Future()

    harness._route_start_client = SimpleNamespace(
        service_is_ready=lambda: True, call_async=start)
    harness._begin_patrol_loop()
    harness._route_request_future = None  # Previous service request completed.
    harness._begin_reposition()
    harness._route_request_future = None
    harness._state['current_loop'] = 2
    harness._begin_patrol_loop(start_point_id=11)

    assert len(requests) == 3
    assert all(request.navigation_mode == 'terrain_3d' for request in requests)
    assert all(request.owner_token == 'inspection-3d' for request in requests)
    assert all(request.authorization_generation == 3 for request in requests)
    assert [(request.hold_at_each_point, request.start_point_id,
             request.point_limit) for request in requests] == [
                 (True, 0, 0), (False, 7, 1), (True, 11, 0)]


class _DatabaseStub:
    def __init__(self, active_task=None):
        self.active_task = active_task
        self.point_results = []
        self.events = []
        self.updates = []

    def get_active_task(self):
        return self.active_task

    def add_point_result(self, task_id, result):
        self.point_results.append((task_id, result))

    def add_event(self, *args, **kwargs):
        self.events.append((args, kwargs))

    def update_task(self, *args, **kwargs):
        self.updates.append((args, kwargs))


class _RoutePointEvent:
    last_point_result = 'skipped'
    last_point_id = 7
    last_point_index = 0
    last_point_name = '检查点1'
    last_point_detail = 'operator skip'


def test_log_level_changes_use_distinct_rclpy_call_sites():
    """An info event followed by an alert must not terminate the ROS node."""
    harness = _load_real_log_method()()
    harness._log_publisher = _Publisher()
    logger = _RclpyCompatibleLogger()
    harness.get_logger = lambda: logger

    harness._log('巡检任务已创建', 'info')
    harness._log('路线途中检测到异常', 'warning')
    harness._log('相机连接失败', 'error')

    assert [level for level, _message in logger.records] == [
        'info', 'warning', 'error'
    ]
    assert len(harness._log_publisher.messages) == 3


def test_inspection_backend_rejects_sub_fifty_percent_detections():
    confirmed = _load_confirmed_detection_filter()
    detections = [
        {'category': 'fire', 'confidence': 0.49},
        {'category': 'smoke', 'confidence': 0.50},
        {'category': 'face', 'confidence': 0.99, 'alert': False},
    ]
    assert confirmed(detections) == [detections[1]]


def test_inspection_backend_accepts_only_valid_over_threshold_temperature():
    confirmed = _load_confirmed_detection_filter()
    detections = [
        {'category': 'high_temperature', 'measurement_type': 'temperature',
         'max_temperature_c': 59.9, 'threshold_c': 60.0},
        {'category': 'high_temperature', 'measurement_type': 'temperature',
         'max_temperature_c': 60.0, 'threshold_c': 60.0},
        {'category': 'high_temperature', 'measurement_type': 'temperature',
         'max_temperature_c': float('nan'), 'threshold_c': 60.0},
        {'category': 'high_temperature', 'measurement_type': 'temperature',
         'max_temperature_c': 80.0, 'threshold_c': 60.0, 'alert': False},
    ]
    assert confirmed(detections) == [detections[1]]


def test_capture_completion_does_not_override_paused_state():
    harness = _load_manager_methods('_drive_inspection_future')()
    future = Future()
    future.set_result({
        'point': {'id': 7, 'sequence': 0, 'name': '检查点1'},
        'failures': [], 'skipped': False, 'cancelled': False,
    })
    harness._inspection_future = future
    harness._state = {
        'task_id': 'task', 'state': 'paused', 'paused': True,
        'completed_points': 0, 'completed_detection_points': 0,
        'skipped_points': 0, 'point_statuses': {},
    }
    harness._cancel_event = threading.Event()
    harness._skip_event = threading.Event()
    route_calls = []
    harness._call_route_control = lambda *args: route_calls.append(args)
    harness._persist = lambda: None
    harness._publish_alerts = lambda **_kwargs: None

    harness._drive_inspection_future()

    assert harness._state['state'] == 'paused'
    assert harness._inspection_future is future
    assert route_calls == []


def test_skipped_inspection_is_counted_only_by_route_event():
    harness = _load_manager_methods(
        '_drive_inspection_future', '_handle_route_point_event')()
    future = Future()
    future.set_result({
        'point': {'id': 7, 'sequence': 0, 'name': '检查点1'},
        'failures': [], 'skipped': True, 'cancelled': False,
    })
    harness._inspection_future = future
    harness._state = {
        'task_id': 'task', 'state': 'inspecting', 'paused': False,
        'current_loop': 1, 'completed_points': 0,
        'completed_detection_points': 0, 'failed_points': 0,
        'skipped_points': 0, 'point_statuses': {},
    }
    harness._database = _DatabaseStub()
    harness._cancel_event = threading.Event()
    harness._skip_event = threading.Event()
    harness._call_route_control = lambda *_args: None
    harness._persist = lambda: None
    harness._publish_alerts = lambda **_kwargs: None
    harness._log = lambda *_args: None

    harness._drive_inspection_future()
    harness._handle_route_point_event(_RoutePointEvent())

    assert harness._state['completed_points'] == 1
    assert harness._state['skipped_points'] == 1


def test_skip_is_rejected_while_task_is_paused():
    harness = _load_manager_methods('_skip_current_point')()
    harness._state = {
        'state': 'paused', 'paused': True, 'resume_state': 'inspecting'}
    harness._skip_event = threading.Event()
    route_calls = []
    harness._call_route_control = lambda *args: route_calls.append(args)

    success, message = harness._skip_current_point()

    assert not success
    assert '继续' in message
    assert not harness._skip_event.is_set()
    assert route_calls == []


def test_late_route_completion_is_deferred_while_paused():
    harness = _load_manager_methods('_finish_patrol_loop')()
    harness._state = {
        'task_id': 'task', 'state': 'paused', 'paused': True,
        'current_loop': 1, 'loop_count': 2, 'bidirectional': True,
    }
    harness._database = _DatabaseStub()
    harness._persist = lambda: None
    harness._complete_task = lambda: harness._state.update(state='completed')
    harness._enter_interval_wait = lambda: harness._state.update(
        state='interval_wait')
    harness._begin_reposition = lambda: harness._state.update(
        state='returning_to_start')

    harness._finish_patrol_loop()

    assert harness._state['state'] == 'paused'
    assert harness._state['deferred_route_completion'] == 'patrol'


def test_restart_preserves_the_state_a_paused_task_should_resume():
    active_task = {
        'task_id': 'task', 'route_id': 3,
        'template': {'checkpoints': []},
        'state': {
            'task_id': 'task', 'state': 'paused', 'paused': True,
            'active': True, 'resume_state': 'interval_wait',
            'interval_remaining_seconds': 120,
        },
    }
    harness = _load_manager_methods('_recover_active_task')()
    harness._database = _DatabaseStub(active_task)
    harness._routes = type('Routes', (), {
        'get_route': lambda _self, _route_id: {
            'id': 3, 'name': 'route', 'map_name': 'map', 'points': [{'id': 7}]}
    })()
    harness._camera = type('Camera', (), {'set_context': lambda *_args: None})()
    harness._route_to_dict = lambda route: dict(route)
    harness._reset_runtime_events = lambda: None
    harness._pause_event = threading.Event()
    harness._state = {}

    harness._recover_active_task()

    assert harness._state['state'] == 'paused'
    assert harness._state['resume_state'] == 'interval_wait'


def test_interval_countdown_is_persisted_as_it_changes():
    harness = _load_manager_methods('_drive_interval_wait')()
    harness._state = {
        'state': 'interval_wait',
        'interval_deadline_monotonic': time.monotonic() + 15.0,
        'interval_remaining_seconds': 30,
    }
    persisted = []
    harness._persist = lambda: persisted.append(
        harness._state['interval_remaining_seconds'])
    harness._begin_patrol_loop = lambda: None

    harness._drive_interval_wait()

    assert len(persisted) == 1
    assert 13 <= persisted[0] <= 15


def test_respawned_manager_adopts_and_pauses_its_owned_route():
    harness = _load_manager_methods('_adopt_recovered_route')()
    harness._state = {
        'task_id': 'task', 'state': 'paused', 'paused': True,
        'resume_state': 'inspecting', 'recovery_pending': True,
    }
    harness._database = _DatabaseStub()
    route_calls = []
    harness._call_route_control = lambda *args: route_calls.append(args)
    message = type('RouteStatus', (), {
        'owner_token': 'task', 'active': True, 'state': 'awaiting_release',
        'point_event_sequence': 9,
    })()

    harness._adopt_recovered_route(message)

    assert harness._route_session_active
    assert harness._route_session_purpose == 'patrol'
    assert harness._state['recovery_pending'] is False
    assert route_calls == [('pause', 'recover_pause')]


def test_route_loss_turns_an_active_task_into_recoverable_pause():
    harness = _load_manager_methods('_pause_for_route_loss')()
    harness._state = {
        'task_id': 'task', 'state': 'navigating',
        'active': True, 'paused': False,
    }
    harness._route_session_active = True
    harness._route_session_purpose = 'patrol'
    harness._pause_event = threading.Event()
    harness._database = _DatabaseStub()
    camera_modes = []
    cancel_requests = []
    harness._nav_cancel_client = type('CancelClient', (), {
        'service_is_ready': lambda _self: True,
        'call_async': lambda _self, request: cancel_requests.append(request),
    })()
    harness._submit_camera_mode = lambda *args: camera_modes.append(args)
    harness._log = lambda *_args: None
    harness._is_active = lambda: True
    harness._uses_previewed_3d_route = lambda: False

    harness._pause_for_route_loss('route node restarted')

    assert harness._state['state'] == 'paused'
    assert harness._state['resume_state'] == 'navigating'
    assert harness._state['recovery_pending'] is True
    assert harness._pause_event.is_set()
    assert camera_modes == [('paused', 'pause')]
    assert len(cancel_requests) == 1


def test_preflight_timeout_pauses_task_without_losing_route_session():
    harness = _load_manager_methods('_pause_for_preflight_timeout')()
    harness._state = {
        'task_id': 'task', 'state': 'navigating',
        'active': True, 'paused': False,
    }
    harness._route_session_active = True
    harness._pause_event = threading.Event()
    harness._database = _DatabaseStub()
    camera_modes = []
    logs = []
    harness._submit_camera_mode = lambda *args: camera_modes.append(args)
    harness._log = lambda *args: logs.append(args)
    harness._is_active = lambda: True
    harness._uses_previewed_3d_route = lambda: False

    harness._pause_for_preflight_timeout('机器人实时位置未能刷新')

    assert harness._state['state'] == 'paused'
    assert harness._state['paused'] is True
    assert harness._state['resume_state'] == 'navigating'
    assert harness._route_session_active is True
    assert harness._pause_event.is_set()
    assert camera_modes == [('paused', 'pause')]
    assert harness._database.events[0][0][1] == 'navigation_preflight_paused'
    assert logs and '当前巡检点已保留' in logs[0][0]


def test_route_service_future_has_a_finite_deadline():
    harness = _load_manager_methods('_drive_route_request_future')()
    harness._route_request_future = Future()
    harness._route_request_purpose = 'control:pause:pause'
    harness._route_request_started_monotonic = time.monotonic() - 11.0
    harness._route_service_timeout_sec = 10.0
    results = []
    harness._apply_route_request_result = lambda *args: results.append(args)
    harness._dispatch_pending_route_request = lambda: None
    harness._persist = lambda: None

    harness._drive_route_request_future()

    assert harness._route_request_future is None
    assert results and results[0][0] == 'control:pause:pause'
    assert results[0][1] is False
    assert '未响应' in results[0][2]


def test_route_node_restart_is_detected_even_after_owner_token_is_lost():
    harness = _load_manager_methods('_route_status_callback')()
    harness._lock = threading.RLock()
    harness._state = {'task_id': 'task', 'active': True, 'state': 'navigating'}
    harness._route_session_active = True
    harness._is_active = lambda: True
    pauses = []
    harness._pause_for_route_loss = lambda detail: pauses.append(detail)
    harness._persist = lambda: None
    harness._publish = lambda **_kwargs: None
    message = type('RouteStatus', (), {
        'active': False, 'state': 'idle', 'owner_token': '',
        'message': 'route manager restarted',
    })()

    harness._route_status_callback(message)

    assert pauses == ['route manager restarted']


def test_3d_pause_ack_detaches_old_start_future_before_resume_or_new_task():
    harness = _load_manager_methods(
        '_drive_pending_3d_revoke', '_drive_route_request_future')()

    class LateFuture:
        def __init__(self):
            self.cancel_requested = False

        def cancel(self):
            self.cancel_requested = True

        def done(self):
            return True

        def result(self):
            return SimpleNamespace(success=False, message='late old failure')

    old_future = LateFuture()
    reply = Future()
    reply.set_result(SimpleNamespace(
        success=True, motion_stopped=True, route_session_active=False,
        authorization_generation=1, message='paused'))
    harness._revoke_3d_route_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda _request: reply)
    urgent_calls = []
    harness._route_control_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: urgent_calls.append(request))
    harness._state = {
        'task_id': 'old', 'state': 'pausing', 'active': True,
        'paused': True, 'message': ''}
    harness._pending_3d_revoke = {
        'action': 'pause', 'terminal': '', 'reason': ''}
    harness._last_3d_revoke_attempt = 0.0
    harness._3d_authorization_generation = 0
    harness._route_request_future = old_future
    harness._route_request_purpose = 'start:patrol'
    harness._route_request_started_monotonic = time.monotonic()
    harness._pending_route_start = None
    harness._pending_route_control = None
    harness._route_session_active = False
    harness._route_session_purpose = ''
    harness._database = _DatabaseStub()
    harness._log = lambda *_args: None
    applied = []
    harness._apply_route_request_result = lambda *args: applied.append(args)
    harness._dispatch_pending_route_request = lambda: None
    harness._route_service_timeout_sec = 10.0

    harness._drive_pending_3d_revoke()
    assert old_future.cancel_requested, harness._state
    assert harness._route_request_future is None
    assert harness._state['state'] == 'paused'
    assert harness._3d_authorization_generation == 1
    assert urgent_calls[0].command == 'pause'
    harness._state['state'] = 'resuming'
    harness._drive_route_request_future()
    assert applied == []
    assert harness._state['state'] == 'resuming'
    harness._state = {
        'task_id': 'new', 'state': 'preparing', 'active': True}
    harness._drive_route_request_future()
    assert applied == []
    assert harness._state['task_id'] == 'new'


def test_3d_interval_pause_preserves_remaining_wait():
    harness = _load_manager_methods('_pause_task')()
    harness._uses_previewed_3d_route = lambda: True
    harness._state = {
        'task_id': 'task', 'state': 'interval_wait',
        'interval_deadline_monotonic': time.monotonic() + 45.0,
    }
    harness._pause_event = threading.Event()
    harness._submit_camera_mode = lambda *_args: None
    harness._schedule_3d_revoke = lambda *_args: None

    success, _message = harness._pause_task()

    assert success
    assert harness._state['state'] == 'pausing'
    assert 44.0 < harness._state['paused_interval_remaining_seconds'] < 46.0


def test_3d_cancel_uses_independent_urgent_stop_if_revoke_unavailable():
    harness = _load_manager_methods('_drive_pending_3d_revoke')()
    harness._state = {
        'task_id': 'old', 'state': 'cancelling',
        'active': True, 'paused': True, 'message': ''}
    harness._pending_3d_revoke = {
        'action': 'cancel', 'terminal': 'cancelled', 'reason': ''}
    harness._last_3d_revoke_attempt = 0.0
    harness._3d_authorization_generation = 0
    urgent = []
    harness._route_control_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: urgent.append(request))
    harness._revoke_3d_route_client = SimpleNamespace(
        service_is_ready=lambda: False)

    harness._drive_pending_3d_revoke()

    assert len(urgent) == 1
    assert urgent[0].command == 'stop'
    assert urgent[0].owner_token == 'old'
    assert harness._state['state'] == 'cancelling'
    assert '尚未确认停止' in harness._state['message']


if __name__ == '__main__':
    test_log_level_changes_use_distinct_rclpy_call_sites()
    test_inspection_backend_rejects_sub_fifty_percent_detections()
    test_inspection_backend_accepts_only_valid_over_threshold_temperature()
    test_capture_completion_does_not_override_paused_state()
    test_skipped_inspection_is_counted_only_by_route_event()
    test_skip_is_rejected_while_task_is_paused()
    test_late_route_completion_is_deferred_while_paused()
    test_restart_preserves_the_state_a_paused_task_should_resume()
    test_interval_countdown_is_persisted_as_it_changes()
    test_respawned_manager_adopts_and_pauses_its_owned_route()
    test_route_loss_turns_an_active_task_into_recoverable_pause()
    test_preflight_timeout_pauses_task_without_losing_route_session()
    test_route_service_future_has_a_finite_deadline()
    test_route_node_restart_is_detected_even_after_owner_token_is_lost()
    test_3d_pause_ack_detaches_old_start_future_before_resume_or_new_task()
    test_3d_interval_pause_preserves_remaining_wait()
    test_3d_cancel_uses_independent_urgent_stop_if_revoke_unavailable()
