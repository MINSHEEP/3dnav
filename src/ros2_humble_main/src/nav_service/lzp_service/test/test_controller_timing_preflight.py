"""Only new tasks wait for CPU-profile transactions; no active-goal gate."""
import ast
from pathlib import Path
from types import SimpleNamespace
from contextlib import nullcontext
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[1] / 'lzp_service/route_navigation_manager.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
manager = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RouteNavigationManager')
method = next(n for n in manager.body if isinstance(n, ast.FunctionDef) and n.name == '_controller_timing_preflight')
namespace = dict(TERRAIN_3D='terrain_3d', time=SimpleNamespace(monotonic=lambda:100.0),
    GoalValidationResult=lambda allowed,code,message: SimpleNamespace(allowed=allowed,code=code,message=message))
exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), namespace)
start_method = next(n for n in manager.body if isinstance(n, ast.FunctionDef) and n.name == '_start_route_callback')
namespace['normalize_navigation_mode'] = lambda value:value
exec(compile(ast.Module(body=[start_method], type_ignores=[]), str(SOURCE), 'exec'), namespace)


def check(*, execution='idle', mode='terrain_3d', body='xingji', status=None, age=0.0):
    state = SimpleNamespace(_mode=execution, _effective_navigation_mode=lambda:mode,
        _robot_body_status={'config':{'selected':body}}, _controller_timing_status=status,
        _controller_timing_received_at=100.0-age)
    return namespace['_controller_timing_preflight'](state)


def ready(**changes):
    return dict(ready=True, profile='xingji_3d', navigation_mode='terrain_3d',
                body_id='xingji', actual_verified_age_sec=0.5, **changes)


def test_active_route_and_single_never_gain_new_configuration_stop_gate():
    for execution in ('route','single'):
        assert check(execution=execution,status={'ready':False,'state':'awaiting_late_ack'}) is None


def test_confirmed_scope_allows_new_task():
    assert check(status=ready()) is None


def test_old_ready_3d_profile_cannot_authorize_new_2d_task():
    result = check(mode='map_2d',status=ready())
    assert not result.allowed
    assert result.code == 'controller_profile_pending'


def test_exact_baseline_restoration_allows_2d_and_other_body():
    status = ready()
    status.update(profile='xingji_2d',navigation_mode='map_2d')
    assert check(mode='map_2d',status=status) is None
    status.update(profile='baseline',body_id='gangben_zsl1',navigation_mode='terrain_3d')
    assert check(body='gangben_zsl1',status=status) is None


def test_new_task_cannot_use_stale_confirmed_state_or_old_actual_readback():
    assert check(status=ready(),age=3.1).allowed is False
    status=ready()
    status['actual_verified_age_sec']=3.1
    assert check(status=status).allowed is False


def test_backward_compatible_absence_does_not_create_a_new_lock():
    assert check(status=None) is None


def test_unresolved_write_rejection_is_explicit_and_finite():
    status=ready()
    status.update(ready=False,state='awaiting_late_ack',error='远端写入尚未确认')
    result=check(status=status)
    assert not result.allowed
    assert '远端写入尚未确认' in result.message
    assert '尚未启动' in result.message


def test_actual_start_route_checks_every_scope_before_claiming_route_ownership():
    for mode, body, owner in (('map_2d','xingji',''),('terrain_3d','gangben_zsl1',''),
                              ('terrain_3d','xingji','inspection:test')):
        state = SimpleNamespace(_mode='idle', _navigation_mode=mode,
            _effective_navigation_mode=lambda:mode, _lio_recovering=False,
            _robot_body_status={'config':{'selected':body}},
            _controller_timing_status=ready(),_controller_timing_received_at=100.0,
            _prepare_3d_route_start=lambda request:(SimpleNamespace(allowed=True),None),
            _lock=nullcontext(),_database=Mock())
        state._controller_timing_status['ready'] = False
        state._controller_timing_preflight = lambda **kwargs: namespace['_controller_timing_preflight'](state,**kwargs)
        response = SimpleNamespace()
        request = SimpleNamespace(navigation_mode=mode,route_id=1,owner_token=owner)
        result=namespace['_start_route_callback'](state,request,response)
        assert result.success is False
        assert state._mode == 'idle'
        assert '导航尚未启动' in result.message
        state._database.get_route.assert_not_called()


def test_actual_start_route_passes_explicit_requested_mode_to_idle_check():
    state = SimpleNamespace(_mode='idle',_navigation_mode='map_2d',_lio_recovering=False,
        _prepare_3d_route_start=lambda request:(SimpleNamespace(allowed=True),None),
        _lock=nullcontext(),_controller_timing_preflight=Mock(return_value=None),_database=Mock())
    state._database.get_route.return_value=None
    request=SimpleNamespace(navigation_mode='map_2d',route_id=1)
    response=namespace['_start_route_callback'](state,request,SimpleNamespace())
    state._controller_timing_preflight.assert_called_once_with(navigation_mode='map_2d')
    assert response.message == 'Route not found'
