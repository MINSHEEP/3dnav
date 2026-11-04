import ast
import copy
import json
import math
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

SOURCE = Path(__file__).resolve().parents[1]/'lzp_service/route_navigation_manager.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RouteNavigationManager')
names = {'_global_localization_required', '_global_localization_check', '_global_localization_callback',
         '_handle_goal_terminal','_timer_callback','_retain_goal_for_global_recovery'}
methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
namespace = dict(TERRAIN_3D='terrain_3d', time=NS(monotonic=lambda:100.), json=json, copy=copy, math=math,
    GoalValidationResult=lambda allowed, code, message:NS(allowed=allowed, code=code, message=message))
exec(compile(ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='Manager', bases=[],
    keywords=[], body=methods, decorator_list=[])], type_ignores=[])), str(SOURCE), 'exec'), namespace)


def manager(mode='terrain_3d', body='xingji'):
    h = namespace['Manager']()
    h._effective_navigation_mode = lambda:mode
    h._robot_body_status = {'config':{'selected':body}, 'generation':1}
    h._current_map_name, h._current_map_load_id = 'map', 'load'
    h._global_localization_status = None
    h._global_localization_at = 0.
    h._lock = nullcontext()
    h._mode, h._state = 'route', 'navigating'
    h._publish_route_log = Mock()
    h._request_stop = Mock()
    h._send_zero_velocity = Mock()
    return h


def test_matching_anchor_required_before_every_xingji_3d_goal():
    h = manager()
    assert h._global_localization_check().allowed is False
    h._global_localization_status = dict(ready=True, scope='xingji_3d',
        map_name='map', map_load_id='load', body_generation=1)
    h._global_localization_at = 100.
    assert h._global_localization_check() is None
    h._global_localization_status['map_load_id'] = 'previous-load'
    assert h._global_localization_check().allowed is False
    h._global_localization_status['map_load_id'] = 'load'
    h._global_localization_at = 99.
    assert h._global_localization_check().allowed is False


def test_active_goal_is_canceled_and_a_later_ready_does_not_restart_it():
    h = manager()
    h._global_localization_callback(NS(data=json.dumps(dict(ready=False, reason='quality_failed'))))
    h._request_stop.assert_called_once_with('global_localization_untrusted')
    h._global_localization_callback(NS(data=json.dumps(dict(ready=True, scope='xingji_3d',
        map_name='map', map_load_id='load', body_generation=1))))
    assert h._request_stop.call_count == 1
    assert h._preview_token is None


def test_route_and_inspection_execution_keep_goal_during_bounded_match_recovery():
    for inspection in (False, True):
        h=manager()
        h._inspection_task_active=inspection
        status=dict(ready=False,recovering=True,reason='matching_recovery',
            scope='xingji_3d',map_name='map',map_load_id='load',body_generation=1,
            sensor_fresh=True,accepted_stamp=97.,recovery_remaining_sec=1.9)
        h._global_localization_callback(NS(data=json.dumps(status)))
        h._request_stop.assert_not_called()
        h._send_zero_velocity.assert_called_once()
        assert h._global_localization_check().code=='global_localization_recovering'
        # Preflight is still blocked. A new target cannot start during a hold.
        assert not h._global_localization_check().allowed
        h._global_localization_callback(NS(data=json.dumps(dict(status,
            ready=True,recovering=False,reason='accepted'))))
        h._request_stop.assert_not_called()
        assert h._state=='navigating'


def test_sensor_gap_keeps_task_without_treating_stale_tf_as_motion_permission():
    h=manager()
    status=dict(ready=False,recovering=True,reason='sensor_recovery',scope='xingji_3d',
        map_name='map',map_load_id='load',body_generation=1,sensor_fresh=False,
        sensor_recovering=True,recovery_remaining_sec=2.6,
        lio_timing=dict(state='tf_stale',source_age_sec=.4,receipt_age_sec=.4))
    h._global_localization_callback(NS(data=json.dumps(status)))
    h._request_stop.assert_not_called()
    h._send_zero_velocity.assert_called_once()
    assert h._global_localization_check().code=='global_localization_recovering'
    status['lio_timing']['source_age_sec']=2.
    h._global_localization_callback(NS(data=json.dumps(status)))
    h._request_stop.assert_called_once_with('global_localization_untrusted')


def test_short_status_heartbeat_gap_blocks_motion_but_does_not_cancel_task():
    h=manager()
    h._global_localization_status=dict(ready=True,scope='xingji_3d',map_name='map',
        map_load_id='load',body_generation=1,sensor_fresh=True)
    h._global_localization_at=99.6
    assert h._global_localization_check().code=='global_localization_recovering'
    h._publish_all_status=Mock()
    h._timer_callback()
    h._request_stop.assert_not_called()
    h._send_zero_velocity.assert_called_once()
    h._global_localization_at=98.4
    assert h._global_localization_check().code=='global_localization_untrusted'


def test_recovery_cannot_hide_changed_identity_expiry_or_missing_sensor_evidence():
    for change in (dict(map_load_id='old'),dict(body_generation=2),
                   dict(recovery_remaining_sec=0),dict(recovery_remaining_sec=10),
                   dict(sensor_fresh=False),dict(recovery_remaining_sec=float('nan'))):
        h=manager()
        status=dict(dict(ready=False,recovering=True,reason='matching_recovery',
            scope='xingji_3d',map_name='map',map_load_id='load',body_generation=1,
            sensor_fresh=True,accepted_stamp=97.,recovery_remaining_sec=1.9),**change)
        h._global_localization_callback(NS(data=json.dumps(status)))
        h._request_stop.assert_called_once_with('global_localization_untrusted')


def test_timer_does_not_cancel_or_dispatch_while_holding_a_live_goal():
    h=manager()
    h._publish_all_status=Mock()
    h._global_localization_callback(NS(data=json.dumps(dict(
        ready=False,recovering=True,reason='matching_recovery',scope='xingji_3d',
        map_name='map',map_load_id='load',body_generation=1,
        sensor_fresh=True,recovery_remaining_sec=1.5))))
    h._timer_callback()
    h._request_stop.assert_not_called()
    h._publish_all_status.assert_called_once()
    assert h._state=='navigating'


def test_controller_abort_during_hold_retains_same_goal_and_manual_cancel_wins():
    for context in ('route','single'):
        h=manager()
        h._mode=context
        h._goal_context=context
        h._cancel_reason=None
        h._active_pose=NS(goal='original')
        h._global_match_hold=True
        h._global_localization_status=dict(ready=False,recovering=True,
            reason='matching_recovery',scope='xingji_3d',map_name='map',
            map_load_id='load',body_generation=1,sensor_fresh=True,
            recovery_remaining_sec=1.5)
        h._global_localization_at=100.
        h._begin_preflight_wait=Mock()
        h._publish_all_status=Mock()
        h._handle_goal_terminal(False,'controller progress timeout')
        h._request_stop.assert_not_called()
        if context=='route':
            assert h._begin_preflight_wait.call_args.args[0].goal=='original'
        else:
            assert h._pending_single_pose.goal=='original'
            assert h._state=='waiting_nav2'

    h=manager()
    h._global_match_hold=True
    h._goal_context='route';h._cancel_reason='user_stop';h._active_pose=NS(goal='original')
    h._finish_task=Mock();h._record_managed_goal_event=Mock()
    h._single_refresh_map_name='';h._publish_all_status=Mock();h._current_point=lambda:dict(name='one')
    h._retain_goal_for_global_recovery=Mock()
    h._handle_goal_terminal(False,'manual cancel')
    h._retain_goal_for_global_recovery.assert_not_called()
    h._finish_task.assert_called_once()


def test_original_scopes_do_not_gain_3d_quality_gates():
    for mode, body in [('map_2d','xingji'), ('map_2d','gangben_zsl1'), ('terrain_3d','gangben_zsl1')]:
        h = manager(mode, body)
        assert h._global_localization_check() is None
        h._global_localization_status = dict(ready=True, scope='legacy', navigation_mode=mode)
        h._global_localization_at = 100.
        assert h._global_localization_check() is None


def test_2d_waits_for_tf_restoration_after_leaving_3d():
    h = manager('map_2d')
    h._global_localization_status = dict(ready=False, scope='legacy', navigation_mode='map_2d')
    h._global_localization_at = 100.
    assert h._global_localization_check().code == 'localization_tf_handoff_pending'


def test_actual_terminal_handler_does_not_retry_canceled_route_or_single_goal():
    for context in ('single', 'route'):
        h = manager()
        h._goal_context, h._cancel_reason = context, 'global_localization_untrusted'
        h._active_pose = None
        h._single_refresh_map_name = ''
        h._current_point = lambda:dict(name='one')
        h._publish_goal_status = Mock()
        h._record_managed_goal_event = Mock()
        h._finish_task, h._advance_route, h._publish_all_status = Mock(), Mock(), Mock()
        h._handle_goal_terminal(False, 'localization stopped')
        assert h._finish_task.call_count == 1
        h._advance_route.assert_not_called()
