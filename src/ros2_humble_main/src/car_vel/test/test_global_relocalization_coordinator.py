import ast
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

SOURCE = Path(__file__).resolve().parents[1]/'car_vel/relocalization_coordinator.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
names = {'_global_identity','_authority_callback','_start_pending_request',
         '_process_action_result','_fallback_or_fail', '_pose_dictionary', '_yaw'}
methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
clock = NS(value=100.)
namespace = dict(copy=copy,json=json,math=math,time=NS(monotonic=lambda:clock.value),
    GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5), Relocalize=NS(Goal=NS,Result=NS(
        CODE_OK=0,CODE_TIMEOUT=33,CODE_INTERNAL_ERROR=34,CODE_VALIDATION_FAILED=32)))
exec(compile(ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='Coordinator',bases=[],
    keywords=[], body=methods, decorator_list=[])],type_ignores=[])), str(SOURCE), 'exec'), namespace)
namespace['RelocalizationCoordinator'] = namespace['Coordinator']


def harness(strict=True):
    clock.value = 100.
    h = namespace['Coordinator']()
    h.current_map_name, h.current_map_load_id = 'map', 'load'
    h._global_body, h._global_body_generation, h._global_mode = 'xingji', 1, 'terrain_3d'
    h._global_body_at = h._global_mode_at = 100.
    h._strict_identity = h._global_identity() if strict else None
    h._await_authority = None
    h._global_status, h._global_status_at = {}, 100.
    h.active_request_id, h.generation = 'initial', 2
    h.active_request_is_automatic = False
    h.pending_request = dict(generation=2, map_name='map', rough_pose=NS())
    h.send_generation = h.active_goal_handle = h.server_wait_deadline = None
    h.search_xy, h.search_yaw, h.relocalize_timeout, h.action_result_grace = 3., math.pi/2, 30., 1.5
    h.action_server_wait = 5.
    h.action_client = Mock()
    h.action_client.server_is_ready.return_value = True
    h._publish_status = Mock()
    h._active_rough_pose = Mock(return_value=NS())
    h._begin_amcl_confirmation = Mock()
    h._clear_action_deadline = Mock()
    h.MAP_FAILURE_CODES = {11,12,13,14,15,16}
    return h


def test_new_initial_request_waits_for_tracker_ack_not_merely_action_discovery():
    h = harness()
    h._start_pending_request(2)
    h.action_client.send_goal_async.assert_not_called()
    assert h.server_wait_deadline == 105.
    h._global_status = dict(initial_request_id='old',matching=False,scope='xingji_3d',map_load_id='load')
    h._start_pending_request(2)
    h.action_client.send_goal_async.assert_not_called()
    h._global_status['initial_request_id'] = 'initial'
    h._start_pending_request(2)
    goal = h.action_client.send_goal_async.call_args.args[0]
    assert goal.robust_validation is True and goal.map_load_id == 'load'


def test_wait_deadline_cannot_fall_back_to_coarse_pose_in_xingji_3d():
    h = harness()
    h._start_pending_request(2)
    clock.value = 106.
    h._start_pending_request(2)
    h._begin_amcl_confirmation.assert_not_called()
    assert h._publish_status.call_args.kwargs['terminal'] is True


def test_original_legacy_coarse_fallback_is_unchanged():
    h = harness(strict=False)
    h._fallback_or_fail(32, 'bad fit')
    h._begin_amcl_confirmation.assert_called_once()
    assert h._publish_status.call_args.args[0] == 'fallback'


def test_good_3d_result_waits_for_same_seed_and_authority_confirmation():
    h = harness()
    result = NS(success=True,code=0,candidate_verified=True, fitness=.1,overlap=.8,
        correction_xy=.2,correction_yaw=.1,refined_pose=NS(),transform_model='se3_map_odom_v1')
    h._process_action_result(result,4)
    h._begin_amcl_confirmation.assert_not_called()
    assert h._await_authority and h._publish_status.call_args.args[1] == 'validation_passed'
    h._authority_callback(NS(data=json.dumps(dict(scope='xingji_3d',ready=True,
        seed_request_id='old',map_load_id='load'))))
    assert h._await_authority
    h._authority_callback(NS(data=json.dumps(dict(scope='xingji_3d',ready=True,
        seed_request_id='initial',map_load_id='load'))))
    assert h._await_authority is None
    assert h._publish_status.call_args.kwargs['success'] is True


def test_unverified_result_or_changed_body_cannot_initialize_global_authority():
    for verified, generation in [(False,1),(True,2)]:
        h = harness()
        h._global_body_generation = generation
        result = NS(success=True,code=0,candidate_verified=verified, fitness=.1,overlap=.8,
            correction_xy=.2,correction_yaw=.1,refined_pose=NS())
        h._process_action_result(result,4)
        h._begin_amcl_confirmation.assert_not_called()
        assert h._await_authority is None
        assert h._publish_status.call_args.args[0] == 'failed'
