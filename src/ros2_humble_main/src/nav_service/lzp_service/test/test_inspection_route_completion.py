"""Actual route finish -> inspection terminal -> late release/skip lifecycle."""
from types import SimpleNamespace as NS
import threading
from unittest.mock import Mock

import pytest

import test_3d_goal_preview as route_fixture
import test_inspection_task_manager_regressions as inspection_fixture


def route():
    cls = route_fixture._load_manager_methods(
        '_finish_task', '_record_managed_goal_event', '_status_message_object')
    cls._status_message_object.__globals__['RouteExecutionStatus'] = lambda: NS(current_point=NS(id=0))
    node = cls()
    node._mode, node._state = 'route', 'awaiting_release'
    node._owner_token = 'task'
    node._route = dict(id=3,name='1',map_name='map')
    node._goal_event_sequence = 10
    node._last_goal_owner_token = 'previous'
    node._last_goal_result = 'succeeded'
    node._last_goal_detail = ''
    node._point_event_sequence = 19
    node._last_point_id, node._last_point_index = 21, 18
    node._last_point_name, node._last_point_result = '目标点21', 'completed'
    node._last_point_detail = ''
    node._current_map_name = 'map'
    node._current_point = lambda: None
    node._dwell_remaining = lambda: 0.
    node._effective_navigation_mode = lambda: 'terrain_3d'
    node._clear_preflight_wait = Mock()
    node._send_zero_velocity = Mock()
    statuses = []
    node._publish_all_status = lambda: statuses.append(node._status_message_object())
    return node, statuses


def inspection():
    cls = inspection_fixture._load_manager_methods(
        '_route_status_callback', '_finish_patrol_loop', '_complete_task',
        '_apply_route_request_result', '_skip_current_point', '_start_route')
    node = cls()
    node._lock = threading.RLock()
    node._state = dict(task_id='task',state='releasing_point',active=True,paused=False,
        current_loop=1,loop_count=1,bidirectional=False,completed_points=19)
    node._route_session_active = True
    node._route_session_purpose = 'patrol'
    node._last_route_event_sequence = 19
    node._route_completion_event_floor = 10
    node._route_status = None
    node._is_active = lambda: node._state['active']
    node._route_request_future = None
    node._pause_event = threading.Event()
    node._skip_event = threading.Event()
    node._uses_previewed_3d_route = lambda: False
    node._pending_route_control = None
    node._database = NS(add_event=Mock(),update_task=Mock())
    for name in ('_persist','_publish','_log','_submit_camera_mode',
                 '_handle_route_point_event','_handle_point_hold','_pause_for_route_loss',
                 '_call_route_control','_dispatch_pending_route_request'):
        setattr(node,name,Mock())
    return node


def terminal():
    node,statuses = route()
    node._finish_task('completed','Route completed')
    return statuses[-1]


def test_real_finish_does_not_lose_inspection_completion_owner():
    message = terminal()
    node = inspection()
    node._route_status_callback(message)
    assert node._state['state'] == 'completed'
    assert node._state['active'] is False
    assert node._state['progress_percent'] == 100.
    assert node._route_session_active is False
    node._database.update_task.assert_called_once()
    # A late reply to the last release cannot resurrect the closed task.
    node._apply_route_request_result('control:release:release',True,'Point released')
    assert node._state['state'] == 'completed'
    node._route_status_callback(message)
    node._database.update_task.assert_called_once()


def test_late_control_reply_cannot_overwrite_a_consumed_terminal_state():
    node = inspection()
    node._state.update(state='completed',active=False)
    node._route_session_active = False
    node._apply_route_request_result('control:release:release',True,'Point released')
    assert node._state['state'] == 'completed'


def test_missing_route_session_does_not_accept_a_phantom_skip():
    node = inspection()
    node._state['state'] = 'navigating'
    node._route_session_active = False
    ok, detail = node._skip_current_point()
    assert not ok
    assert '路线' in detail
    node._call_route_control.assert_not_called()


def test_foreign_or_replayed_completion_does_not_finish_current_route():
    for field,value in (('last_goal_owner_token','other-task'),('goal_event_sequence',10)):
        node = inspection()
        message = terminal()
        setattr(message,field,value)
        node._route_status_callback(message)
        assert node._state['active'] and node._route_session_active
        node._database.update_task.assert_not_called()


def test_start_remembers_terminal_event_floor_before_async_reply():
    node = inspection()
    node._route_status = NS(goal_event_sequence=25)
    node._start_route('patrol',False,True,0,0)
    assert node._route_completion_event_floor == 25


def test_terminal_status_before_start_reply_is_consumed_when_reply_arrives():
    node = inspection()
    node._route_session_active = False
    node._state['state'] = 'starting'
    node._route_status_callback(terminal())
    assert node._state['active']
    node._apply_route_request_result('start:patrol',True,'Route started')
    assert node._state['state'] == 'completed' and not node._state['active']


def test_safety_notice_is_queued_only_on_the_first_successful_patrol_start():
    node = inspection()
    node._state.update(state='starting', start_safety_notice_pending=True)
    node._queue_patrol_safety_notice = Mock()
    node._apply_route_request_result('start:patrol', True, 'Route started')
    node._apply_route_request_result('start:patrol', True, 'Next loop started')
    node._queue_patrol_safety_notice.assert_called_once_with()
    assert node._state['state'] == 'navigating'


def test_safety_notice_and_checkpoint_work_are_queued_independently():
    node = inspection_fixture._load_manager_methods('_handle_point_hold', '_queue_patrol_safety_notice')()
    node._state = dict(task_id='task', paused=False, current_loop=1)
    node._processed_hold_key = ''
    node._checkpoints = {7: {'route_point_id': 7}}
    node._inspection_future = None
    node._patrol_speech_worker = Mock()
    node._send_patrol_safety_notice = Mock()
    node._workers = Mock()
    node._run_checkpoint = Mock()
    node._skip_event = threading.Event()
    node._log = Mock()
    message = NS(current_point=NS(id=7, sequence=2, name='检查点', x=1, y=2), point_event_sequence=8)
    node._handle_point_hold(message)
    node._handle_point_hold(message)
    node._patrol_speech_worker.submit.assert_called_once_with(node._send_patrol_safety_notice, 'task')
    node._workers.submit.assert_called_once()
    assert node._state['state'] == 'inspecting'


def test_paused_completion_is_deferred_until_explicit_resume():
    node = inspection()
    node._state.update(state='paused',paused=True)
    node._route_status_callback(terminal())
    assert node._state['state'] == 'paused'
    assert node._state['deferred_route_completion'] == 'patrol'
    assert not node._route_session_active
    node._database.update_task.assert_not_called()
    node._apply_route_request_result('control:release:release',True,'released')
    assert node._state['state'] == 'paused'


def test_final_reposition_completion_enters_interval_not_task_completion():
    node = inspection()
    node._route_session_purpose = 'reposition'
    node._enter_interval_wait = Mock()
    node._route_status_callback(terminal())
    node._enter_interval_wait.assert_called_once()
    node._database.update_task.assert_not_called()
    assert not node._route_session_active


@pytest.mark.parametrize('command', ['release','skip'])
def test_late_reply_cannot_replace_next_loop_interval(command):
    node = inspection()
    node._state.update(state='interval_wait')
    node._route_session_active = False
    node._apply_route_request_result(f'control:{command}:{command}',True,'ok')
    assert node._state['state'] == 'interval_wait'


def test_skip_during_active_route_still_forwards_to_route_manager():
    node = inspection()
    node._state['state'] = 'navigating'
    assert node._skip_current_point()[0]
    node._call_route_control.assert_called_once_with('skip','skip')


def test_active_inspection_skip_still_cancels_checkpoint_work():
    node = inspection()
    node._state['state'] = 'inspecting'
    assert node._skip_current_point()[0]
    assert node._skip_event.is_set()
    node._call_route_control.assert_not_called()


def test_new_route_process_can_reset_completion_sequence():
    node = inspection()
    node._route_status = NS(goal_event_sequence=0)
    node._start_route('patrol',False,True,0,0)
    assert node._route_completion_event_floor == 0
    message = terminal()
    message.goal_event_sequence = 1
    node._route_status_callback(message)
    assert node._state['state'] == 'completed'


@pytest.mark.parametrize('state',['pausing','cancelling'])
def test_revoke_barrier_keeps_control_of_racing_completion(state):
    node = inspection()
    node._state['state'] = state
    node._route_status_callback(terminal())
    assert node._state['state'] == state
    node._database.update_task.assert_not_called()


def test_task_already_completed_rejects_skip_without_backend_request():
    node = inspection()
    node._route_status_callback(terminal())
    assert not node._skip_current_point()[0]
    node._call_route_control.assert_not_called()


def test_late_resume_reply_cannot_replace_completion_message():
    node = inspection()
    node._route_status_callback(terminal())
    node._apply_route_request_result('control:resume:resume',True,'resumed')
    assert node._state['state'] == 'completed'
    assert node._state['message'] == '巡检任务已完成'
