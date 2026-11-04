import ast
import copy
import json
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

SOURCE = Path(__file__).resolve().parents[1]/'lzp_service/route_navigation_manager.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RouteNavigationManager')
names = {'_global_localization_required', '_global_localization_check', '_global_localization_callback',
         '_handle_goal_terminal'}
methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
namespace = dict(TERRAIN_3D='terrain_3d', time=NS(monotonic=lambda:100.), json=json, copy=copy,
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
