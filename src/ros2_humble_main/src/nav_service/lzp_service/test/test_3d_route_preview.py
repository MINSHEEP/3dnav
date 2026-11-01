#!/usr/bin/env python3
"""Offline contracts for Xingji-only whole-route preview and patrol lease."""

import ast
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import threading
import time
import textwrap
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'lzp_service' / 'route_navigation_manager.py').read_text(
    encoding='utf-8')
TREE = ast.parse(SOURCE)
CLASS = next(node for node in TREE.body if (
    isinstance(node, ast.ClassDef) and node.name == 'RouteNavigationManager'))
INTERFACE = ROOT.parent / 'lzp_interface'


@dataclass
class Result:
    allowed: bool
    code: str
    message: str


def _method(name):
    node = next(item for item in CLASS.body if (
        isinstance(item, ast.FunctionDef) and item.name == name))
    return ast.get_source_segment(SOURCE, node)


def _harness(*names):
    names = list(names)
    if '_prepare_3d_route_start' in names:
        names.append('_prepare_3d_route_start_checked')
    namespace = {
        'GoalValidationResult': Result,
        'TERRAIN_3D': 'terrain_3d',
        'time': time,
        'json': json,
        'hashlib': hashlib,
        'normalize_navigation_mode': lambda value: value,
    }
    body = 'from __future__ import annotations\nclass Harness:\n'
    for name in names:
        body += textwrap.indent(_method(name), '    ') + '\n'
    exec(body, namespace)
    return namespace['Harness']


def _request(mode='terrain_3d'):
    return SimpleNamespace(
        route_id=8, start_point_id=0, point_limit=0,
        reverse=False, loop_count=1, continuous=False,
        navigation_mode=mode, owner_token='', preview_id='',
        authorization_generation=0, request_deadline_unix_ns=0)


def _route():
    return {
        'id': 8, 'map_name': 'garden', 'name': 'patrol',
        'points': [{'id': 11, 'x': 1.0, 'y': 0.0},
                   {'id': 12, 'x': 2.0, 'y': 0.0},
                   {'id': 13, 'x': 3.0, 'y': 0.0}],
    }


def test_route_order_includes_wrap_for_finite_repetition():
    cls = _harness('_route_preview_points')
    select = cls._route_preview_points
    route = _route()
    assert [p['id'] for p in select(route)] == [11, 12, 13]
    assert [p['id'] for p in select(
        route, start_point_id=12, loop_count=2)] == [12, 13, 11, 12, 13]
    assert [p['id'] for p in select(
        route, reverse=True, point_limit=2)] == [13, 12]
    try:
        select(route, continuous=True)
    except ValueError as error:
        assert '持续循环' in str(error)
    else:
        raise AssertionError('3D continuous route was accepted')


def test_xingji_requires_preview_but_2d_and_gangben_do_not():
    cls = _harness('_prepare_3d_route_start')
    manager = cls()
    manager._lock = threading.RLock()
    manager._navigation_mode = 'terrain_3d'
    manager._mode = 'idle'
    manager._preview_busy = False
    manager._robot_body_status = {
        'config': {'selected': 'xingji'}}
    manager._current_map_name = 'garden'
    manager._database = SimpleNamespace(get_route=lambda _id: _route())
    manager._terrain_capability = SimpleNamespace(
        planar_execution_blockers=lambda _points, **_kwargs: [])
    manager._route_digest = lambda _route: 'route-sha'
    manager._route_preview_points = lambda route, **_kwargs: route['points']
    manager._inspection_route_authorization = None
    manager._preview_token = None
    result, ticket = manager._prepare_3d_route_start(_request())
    assert not result.allowed and result.code == 'preview_missing'
    assert ticket is None
    result, ticket = manager._prepare_3d_route_start(_request('map_2d'))
    assert result.allowed and ticket is None
    manager._robot_body_status['config']['selected'] = 'gangben_zsl1'
    result, ticket = manager._prepare_3d_route_start(_request())
    assert result.allowed and ticket is None


def test_delayed_old_start_request_cannot_enter_after_pause_resume():
    cls = _harness('_prepare_3d_route_start')
    manager = cls()
    manager._lock = threading.RLock()
    manager._navigation_mode = 'terrain_3d'
    manager._mode = 'idle'
    manager._preview_busy = False
    manager._robot_body_status = {'config': {'selected': 'xingji'}}
    manager._current_map_name = 'garden'
    manager._database = SimpleNamespace(get_route=lambda _id: _route())
    manager._terrain_capability = SimpleNamespace(
        planar_execution_blockers=lambda _points, **_kwargs: [])
    manager._route_digest = lambda _route: 'route-sha'
    manager._route_preview_points = lambda route, **_kwargs: route['points']
    manager._inspection_route_authorization = {
        'owner_token': 'inspection-1', 'route_id': 8,
        'route_digest': 'route-sha', 'generation': 2,
        'state': 'active', 'snapshot': {}}
    manager._inspection_task_active = True
    manager._inspection_task_id = 'inspection-1'
    manager._inspection_task_paused = False
    request = _request()
    request.owner_token = 'inspection-1'
    request.authorization_generation = 0  # Sent before pause/resume.
    request.request_deadline_unix_ns = time.time_ns() + 1_000_000_000

    result, ticket = manager._prepare_3d_route_start(request)

    assert not result.allowed
    assert result.code == 'inspection_authorization_expired'
    assert ticket is None


def test_ticket_rejects_route_map_mode_body_inflation_and_deadline_changes():
    cls = _harness('_verify_3d_route_start_ticket')
    manager = cls()
    manager._route_digest = lambda _route: 'route-sha'
    manager._preview_3d_snapshot_matches = lambda _snapshot: True
    manager._preview_3d_identity_matches = lambda _snapshot: True
    route = _route()
    ticket = {
        'route_digest': 'route-sha', 'snapshot': {'map_load_id': 'L1'},
        'owner_token': '', 'expires_at': time.monotonic() + 10.0,
        'request_generation': 0,
    }
    assert manager._verify_3d_route_start_ticket(ticket, route)
    manager._route_digest = lambda _route: 'changed-route'
    assert not manager._verify_3d_route_start_ticket(ticket, route)
    manager._route_digest = lambda _route: 'route-sha'
    # The existing snapshot contract includes map load_id, mode version,
    # body generation, terrain stamp, and inflation version.
    manager._preview_3d_snapshot_matches = lambda _snapshot: False
    assert not manager._verify_3d_route_start_ticket(ticket, route)
    manager._preview_3d_snapshot_matches = lambda _snapshot: True
    ticket['expires_at'] = time.monotonic() - 1.0
    assert manager._verify_3d_route_start_ticket(ticket, route)
    manager._preview_request_generation = 1
    assert not manager._verify_3d_route_start_ticket(ticket, route)
    manager._preview_request_generation = 0

    ticket.update({
        'owner_token': 'inspection-1',
        'deadline_ns': time.time_ns() + 1_000_000_000,
        'auth_generation': 0,
    })
    manager._inspection_route_authorization = {
        'owner_token': 'inspection-1',
        'route_digest': 'route-sha',
        'snapshot': {'map_load_id': 'L1'},
        'state': 'active',
        'generation': 0,
    }
    manager._inspection_task_active = True
    manager._inspection_task_id = 'inspection-1'
    manager._inspection_task_paused = False
    assert manager._verify_3d_route_start_ticket(ticket, route)
    manager._inspection_task_paused = True
    assert not manager._verify_3d_route_start_ticket(ticket, route)
    manager._inspection_task_paused = False
    manager._inspection_task_active = False
    assert not manager._verify_3d_route_start_ticket(ticket, route)
    manager._inspection_task_active = True
    ticket['deadline_ns'] = time.time_ns() - 1
    assert not manager._verify_3d_route_start_ticket(ticket, route)


def test_pause_resume_generation_fences_in_flight_start_and_cancel_revokes():
    cls = _harness(
        '_verify_3d_route_start_ticket',
        '_revoke_3d_inspection_route_callback')
    manager = cls()
    manager._lock = threading.RLock()
    manager._mode = 'idle'  # StartRoute is still computing its preview.
    manager._owner_token = ''
    manager._effective_navigation_mode = lambda: 'terrain_3d'
    manager._route_digest = lambda _route: 'route-sha'
    manager._preview_3d_snapshot_matches = lambda _snapshot: True
    manager._preview_3d_identity_matches = lambda _snapshot: True
    manager._inspection_route_authorization = {
        'owner_token': 'inspection-1',
        'route_digest': 'route-sha',
        'snapshot': {}, 'generation': 0,
        'pause_latched': False, 'state': 'active'}
    manager._inspection_task_active = True
    manager._inspection_task_id = 'inspection-1'
    manager._inspection_task_paused = False
    ticket = {
        'route_digest': 'route-sha', 'snapshot': {},
        'owner_token': 'inspection-1',
        'deadline_ns': time.time_ns() + 1_000_000_000,
        'auth_generation': 0}
    route = _route()
    assert manager._verify_3d_route_start_ticket(ticket, route)
    request = SimpleNamespace(owner_token='inspection-1', action='pause')
    response = manager._revoke_3d_inspection_route_callback(
        request, SimpleNamespace())
    assert response.success and response.motion_stopped
    assert not manager._verify_3d_route_start_ticket(ticket, route)
    request.action = 'resume'
    assert manager._revoke_3d_inspection_route_callback(
        request, SimpleNamespace()).success
    assert not manager._verify_3d_route_start_ticket(ticket, route)
    ticket['auth_generation'] = 2  # Only fresh planning may now commit.
    assert manager._verify_3d_route_start_ticket(ticket, route)
    request.action = 'cancel'
    assert manager._revoke_3d_inspection_route_callback(
        request, SimpleNamespace()).success
    assert manager._inspection_route_authorization is None
    assert not manager._verify_3d_route_start_ticket(ticket, route)


def test_inspection_cancel_goal_terminal_never_advances_route():
    cls = _harness('_handle_goal_terminal')
    for succeeded in (False, True):
        manager = cls()
        manager._goal_context = 'route'
        manager._cancel_reason = 'inspection_cancel'
        manager._active_pose = None
        manager._mode = 'route'
        manager._current_point = lambda: {'name': '当前点'}
        manager._publish_route_log = lambda _message: None
        manager._publish_all_status = lambda: None
        manager._finish_task = lambda state, _detail: setattr(
            manager, 'finished_as', state)
        manager._advance_route = lambda: setattr(manager, 'advanced', True)
        manager.finished_as = None
        manager.advanced = False

        manager._handle_goal_terminal(succeeded, 'Nav2 terminal')

        assert manager.finished_as == 'stopped'
        assert not manager.advanced


def test_revoke_retries_rejected_nav2_cancel_until_pause_confirmed():
    cls = _harness('_revoke_3d_inspection_route_callback')
    manager = cls()
    manager._lock = threading.RLock()
    manager._mode = 'route'
    manager._owner_token = 'inspection-1'
    manager._effective_navigation_mode = lambda: 'terrain_3d'
    manager._inspection_route_authorization = {
        'owner_token': 'inspection-1', 'generation': 0}
    manager._inspection_task_paused = False
    manager._state = 'pausing'
    manager._active_goal_handle = object()
    manager._cancel_in_progress = False
    manager._pause_route = lambda: (
        (True, 'already paused') if manager._state == 'paused_navigation'
        else (_ for _ in ()).throw(AssertionError(
            'must retry cancellation, not restart pause state')))
    manager._send_zero_velocity = lambda: None
    attempts = []
    manager._cancel_active_goal = lambda: attempts.append('cancel') or False
    request = SimpleNamespace(owner_token='inspection-1', action='pause')

    first = manager._revoke_3d_inspection_route_callback(
        request, SimpleNamespace())
    second = manager._revoke_3d_inspection_route_callback(
        request, SimpleNamespace())
    assert first.success and not first.motion_stopped
    assert second.success and not second.motion_stopped
    assert attempts == ['cancel', 'cancel']
    manager._state = 'paused_navigation'
    confirmed = manager._revoke_3d_inspection_route_callback(
        request, SimpleNamespace())
    assert confirmed.motion_stopped


def test_late_3d_control_cannot_resume_release_or_skip_after_revoke():
    cls = _harness('_control_route_callback')
    manager = cls()
    manager._lock = threading.RLock()
    manager.VALID_CONTROLS = {
        'pause', 'resume', 'release', 'skip', 'stop'}
    manager._mode = 'route'
    manager._owner_token = 'inspection-1'
    manager._robot_body_status = {'config': {'selected': 'xingji'}}
    manager._effective_navigation_mode = lambda: 'terrain_3d'
    manager._inspection_route_authorization = {
        'owner_token': 'inspection-1', 'pause_latched': True}
    manager._inspection_task_active = True
    manager._inspection_task_id = 'inspection-1'
    manager._inspection_task_paused = True
    manager._publish_all_status = lambda: None
    dispatched = []
    manager._resume_route = lambda: (
        dispatched.append('resume'), (True, 'ok'))[1]
    manager._release_route_point = lambda: (
        dispatched.append('release'), (True, 'ok'))[1]
    manager._skip_route_point = lambda: (
        dispatched.append('skip'), (True, 'ok'))[1]
    manager._pause_route = lambda: (True, 'already paused')
    manager._request_stop = lambda _reason: (True, 'stopping')

    for action in ('resume', 'release', 'skip'):
        response = manager._control_route_callback(
            SimpleNamespace(
                command=action, owner_token='inspection-1',
                authorization_generation=0),
            SimpleNamespace())
        assert not response.success
    assert dispatched == []
    manager._inspection_route_authorization = None
    manager._inspection_task_paused = False
    for action in ('resume', 'release', 'skip'):
        response = manager._control_route_callback(
            SimpleNamespace(
                command=action, owner_token='inspection-1',
                authorization_generation=0),
            SimpleNamespace())
        assert not response.success
    assert dispatched == []
    manager._inspection_route_authorization = {
        'owner_token': 'inspection-1',
        'pause_latched': False, 'generation': 2}
    for action in ('resume', 'release', 'skip'):
        response = manager._control_route_callback(
            SimpleNamespace(
                command=action, owner_token='inspection-1',
                authorization_generation=0),
            SimpleNamespace())
        assert not response.success
    assert dispatched == []
    fresh = manager._control_route_callback(
        SimpleNamespace(
            command='resume', owner_token='inspection-1',
            authorization_generation=2), SimpleNamespace())
    assert fresh.success and dispatched == ['resume']


def test_inspection_cancel_revokes_active_authorization():
    cls = _harness('_inspection_status_callback')
    manager = cls()
    manager._lock = threading.RLock()
    manager._inspection_route_authorization = {
        'owner_token': 'inspection-1', 'state': 'active'}
    manager._inspection_task_active = True
    manager._inspection_task_id = 'inspection-1'
    manager._inspection_task_paused = False
    manager._publish_navigation_mode = lambda: None
    manager._publish_inflation_radius = lambda: None
    manager._inspection_status_callback(SimpleNamespace(data=json.dumps({
        'active': False, 'task_id': 'inspection-1', 'paused': False})))
    assert manager._inspection_route_authorization is None


def test_start_ticket_checked_before_any_route_state_or_nav2_send():
    source = _method('_start_route_callback')
    assert source.index('_verify_3d_route_start_ticket') < source.index(
        "self._mode = 'route'")
    assert source.index('_verify_3d_route_start_ticket') < source.index(
        '_dispatch_current_route_point()')
    srv = (INTERFACE / 'srv' / 'StartRoute.srv').read_text(
        encoding='utf-8')
    assert 'string preview_id' in srv
    assert 'int64 request_deadline_unix_ns' in srv


def test_preview_planning_never_calls_motion_action():
    planning = _method('_plan_3d_route_segments')
    preview = _method('_preview_3d_route_callback')
    assert 'ComputePathToPose.Goal()' in planning
    assert 'NavigateToPose.Goal()' not in planning
    assert '_send_pose(' not in planning + preview
    assert '_single_goal_callback(' not in planning + preview


if __name__ == '__main__':
    test_route_order_includes_wrap_for_finite_repetition()
    test_xingji_requires_preview_but_2d_and_gangben_do_not()
    test_delayed_old_start_request_cannot_enter_after_pause_resume()
    test_ticket_rejects_route_map_mode_body_inflation_and_deadline_changes()
    test_pause_resume_generation_fences_in_flight_start_and_cancel_revokes()
    test_inspection_cancel_goal_terminal_never_advances_route()
    test_revoke_retries_rejected_nav2_cancel_until_pause_confirmed()
    test_late_3d_control_cannot_resume_release_or_skip_after_revoke()
    test_inspection_cancel_revokes_active_authorization()
    test_start_ticket_checked_before_any_route_state_or_nav2_send()
    test_preview_planning_never_calls_motion_action()
