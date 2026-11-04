"""Exercise the production ROS adapter with transport/time replaced, not its policy."""
import ast
import json
import math
from collections import deque
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

from car_vel.global_pose_authority import (
    GlobalPoseAuthority, Pose2, compose, lio_step_valid, usable_lio_health, angle)

SOURCE = Path(__file__).resolve().parents[1]/'car_vel/global_pose_authority_node.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
clock = NS(wall=100., ros=100.)
namespace = dict(json=json, time=NS(monotonic=lambda: clock.wall), compose=compose,
    Pose2=Pose2, lio_step_valid=lio_step_valid, String=NS, Parameter=NS, ParameterValue=NS,
    GetParameters=NS(Request=NS), SetParametersAtomically=NS(Request=NS),
    ParameterType=NS(PARAMETER_BOOL=1), stamp_seconds=lambda s:s.sec+s.nanosec*1e-9)
namespace['math'] = math
namespace['angle'] = angle
namespace['usable_lio_health'] = usable_lio_health
namespace['EmptyService'] = NS(Request=NS)
pose_function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'pose2')
exec(compile(ast.Module(body=[pose_function], type_ignores=[]), str(SOURCE), 'exec'), namespace)
namespace['Time'] = lambda **kw:NS(to_msg=lambda:NS(sec=int(kw['nanoseconds']//10**9),
    nanosec=int(kw['nanoseconds'] % 10**9)))
namespace['TransformStamped'] = lambda:NS(header=NS(), transform=NS(
    translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
namespace['GoalStatus'] = NS(STATUS_SUCCEEDED=4)
methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name != '__init__']
adapter = ast.ClassDef(name='Adapter', bases=[], keywords=[], body=methods, decorator_list=[])
exec(compile(ast.fix_missing_locations(ast.Module(body=[adapter], type_ignores=[])), str(SOURCE), 'exec'), namespace)
Adapter = namespace['Adapter']


def harness():
    clock.wall = clock.ros = 100.
    h = Adapter()
    h.policy = GlobalPoseAuthority()
    h.scope_known = True
    h.scope = True
    h.body, h.body_generation, h.mode = 'xingji', 1, 'terrain_3d'
    h.map_name, h.load_id, h.lio_epoch = 'map', 'load', 0
    h.mode_at = h.body_at = h.lio_at = clock.wall
    h.lio_ok = True
    h.active = False
    h.param_future = h.param_operation = None
    h.amcl_gid = '01'
    h.actual_tf = h.baseline = None
    h.last_read = h.last_owned_tf = h.param_started = h.handoff_at = 0.
    h.record_bad = False
    h.record = None
    h._save = Mock(side_effect=lambda pending:setattr(h, 'record', dict(
        gid=h.amcl_gid, baseline=h.baseline, pending=pending)))
    h.error = ''
    h.tf_fault = False
    h.own_tf_gid = None
    h.own_tf_messages = deque(maxlen=64)
    h._ensure_own_tf_gid = Mock(return_value=True)
    h.external_tf_at = 0.
    h.read, h.write = Mock(), Mock()
    h.read.service_is_ready.return_value = h.write.service_is_ready.return_value = True
    h.get_publishers_info_by_topic = lambda topic:[NS(node_name='amcl',
        node_namespace='/', endpoint_gid=[1])]
    h.policy.set_identity((h.body, h.body_generation, h.map_name, h.load_id, h.lio_epoch, h.mode))
    h.seed_request = h.initial_request_id = ''
    h.legacy_required, h.legacy_wait = False, None
    h.tracking = h.tracking_handle = None
    h.registration_busy = False
    h.raw_pose = h.last_raw_stamp = h.last_pose_stamp = h.last_odom = None
    h.last_odom_wall = 0.
    h.was_ready = h.motion_latched = False
    h.last_status = None
    h.pose_pub, h.status_pub, h.broadcaster = Mock(), Mock(), Mock()
    h.initial_pose_pub, h.nomotion = Mock(), Mock()
    h.get_logger = Mock()
    h.get_clock = lambda:NS(now=lambda:NS(nanoseconds=int(clock.ros*1e9)))
    h._odom = lambda stamp=None:(Pose2(0,0,0), NS(sec=int(clock.ros), nanosec=0))
    h._message = lambda pose, stamp:NS(mapped=pose, header=NS(frame_id='map',stamp=stamp),
        pose=NS(covariance=[0.]*36, pose=NS(position=NS(x=pose.x,y=pose.y,z=0.),
        orientation=NS(x=0.,y=0.,z=math.sin(pose.yaw/2),w=math.cos(pose.yaw/2)))))
    h._match = Mock()
    return h


def complete_read(h, value):
    h.param_future = Future()
    h.param_operation = 'read'
    h.param_future.set_result(NS(values=[NS(type=1, bool_value=value)]))


def test_disable_requires_successful_set_and_independent_readback():
    h = harness()
    complete_read(h, True)
    h._parameters(clock.wall)
    assert h.baseline is True
    assert h.record['pending'] is True
    assert h.param_operation == 'set'
    assert h.actual_tf is True
    pending = Future()
    h.param_future = pending
    pending.set_result(NS(result=NS(successful=True)))
    h._parameters(clock.wall)
    assert h.actual_tf is None
    assert h.param_operation == 'read'
    complete_read(h, False)
    h._parameters(clock.wall)
    assert h.actual_tf is False
    assert h.record['pending'] is False


def test_late_set_kept_and_restored_after_scope_change_no_second_writer():
    h = harness()
    h.baseline, h.actual_tf = True, True
    h.last_read = clock.wall
    h._parameters(clock.wall)
    pending = Future(); h.param_future = pending
    h.scope = False
    clock.wall += 4
    h._parameters(clock.wall)
    assert h.param_future is pending and not pending.cancelled()
    assert h.actual_tf is None
    pending.set_result(NS(result=NS(successful=True)))
    h._parameters(clock.wall)
    complete_read(h, False)
    h._parameters(clock.wall)
    assert h.param_operation == 'set'
    request = h.write.call_async.call_args.args[0]
    assert request.parameters[0].value.bool_value is True


def test_actual_baseline_false_is_not_overwritten_in_other_body():
    h = harness()
    h.scope = False
    complete_read(h, False)
    h._parameters(clock.wall)
    assert h.baseline is False
    h.write.call_async.assert_not_called()


def test_periodic_readback_does_not_pause_original_2d_or_gangben_navigation():
    h = harness()
    h.scope = False
    h.baseline = h.actual_tf = True
    h.record = dict(gid='01', baseline=True, pending=False)
    h.param_operation, h.param_future = 'read', Future()
    assert h._legacy_ready()
    h.param_operation = 'set'
    assert not h._legacy_ready()


def test_restart_with_pending_same_amcl_instance_does_not_guess_ownership():
    h = harness()
    h.amcl_gid = ''
    h.record = dict(gid='01', baseline=True, pending=True)
    h._parameters(clock.wall)
    assert h.record_bad and h.actual_tf is None
    h.write.call_async.assert_not_called()


def test_legacy_passthrough_preserves_exact_object_and_timestamp():
    h = harness()
    h.scope = False
    h.baseline = h.actual_tf = True
    message = NS(header=NS(stamp=NS(sec=99, nanosec=123)))
    h._raw(message)
    assert h.pose_pub.publish.call_args.args[0] is message
    h.scope = True
    h.pose_pub.reset_mock()
    h._raw(message)
    h.pose_pub.publish.assert_not_called()


def test_match_loss_latches_until_explicit_new_initialization():
    h = harness()
    h._parameters = Mock()
    h.actual_tf = False
    h.was_ready = True
    h.active = True
    assert h.policy.accept(Pose2(0,0,0), Pose2(0,0,0), stamp=97., now=97.1,
        rmse=.1, overlap=.8, ambiguous=False, initial=True)
    h._tick()
    assert h.motion_latched
    assert json.loads(h.status_pub.publish.call_args.args[0].data)['ready'] is False
    # A later good local match must not resume the old executing goal.
    h.policy.accepted_stamp = 100.
    h._tick()
    assert json.loads(h.status_pub.publish.call_args.args[0].data)['ready'] is False
    h.broadcaster.sendTransform.assert_not_called()


def test_stale_action_result_from_previous_map_is_discarded():
    h = harness()
    future = Future()
    future.set_result(NS(status=4, result=NS(success=True, candidate_verified=True)))
    h._match_result(future, ('old-map',))
    assert h.policy.transform is None


def test_accepted_transform_and_pose_published_raw_amcl_jump_ignored():
    h = harness()
    h._parameters = Mock()
    h.actual_tf = False
    assert h.policy.accept(Pose2(1,2,.1), Pose2(0,0,0), stamp=99.9, now=100.,
        rmse=.1, overlap=.8, ambiguous=False, initial=True)
    h._tick()
    assert json.loads(h.status_pub.publish.call_args.args[0].data)['ready']
    t = h.broadcaster.sendTransform.call_args.args[0]
    assert t.header.frame_id == 'map' and t.child_frame_id == 'odom'
    assert (t.transform.translation.x, t.transform.translation.y) == (1,2)
    actual = h.pose_pub.publish.call_args.args[0].mapped
    assert (actual.x, actual.y) == (1,2) and math.isclose(actual.yaw, .1)
    count = h.pose_pub.publish.call_count
    h._raw(NS(header=NS(stamp=NS(sec=100,nanosec=1)), pose='bad jump'))
    assert h.pose_pub.publish.call_count == count
    h._tick()
    assert h.pose_pub.publish.call_count == count


def test_missing_discovery_does_not_cancel_a_remote_set():
    h = harness()
    pending = Future()
    h.param_future = pending
    h.param_operation = 'set'
    h.get_publishers_info_by_topic = lambda _:[]
    h._parameters(clock.wall)
    assert h.param_future is pending and not pending.cancelled()
    assert h.actual_tf is None


def test_unconnected_body_cannot_enter_3d_tf_scope():
    h = harness()
    h._parameters = Mock()
    h._body(NS(data=json.dumps(dict(config=dict(selected='xingji', profiles=dict(
        xingji=dict(navigation_verified=True))), connected=False, active_body='xingji', generation=1))))
    assert h.body == ''
    h._tick()
    assert not h.scope
    h.broadcaster.sendTransform.assert_not_called()


def test_original_2d_passthrough_does_not_require_body_connection():
    h = harness()
    h._parameters = Mock()
    h.body, h.mode = '', 'map_2d'
    h.scope = False
    h.actual_tf = h.baseline = True
    h._tick()
    assert h.scope_known and not h.scope
    message = NS(header=NS(stamp=NS(sec=99, nanosec=123)))
    h._raw(message)
    assert h.pose_pub.publish.call_args.args[0] is message


def test_switching_back_to_2d_waits_for_three_new_agreeing_amcl_samples():
    h = harness()
    h._parameters = Mock()
    h.actual_tf, h.baseline = False, True
    assert h.policy.accept(Pose2(1,2,.1), Pose2(0,0,0), stamp=99.9, now=100.,
        rmse=.1, overlap=.8, ambiguous=False, initial=True)
    h.mode = 'map_2d'
    h._tick()
    assert h.legacy_required and h.legacy_wait
    h.initial_pose_pub.publish.assert_called_once()
    # Old or mismatched raw messages never authorize restoration.
    bad = h._message(Pose2(5,5,1),NS(sec=100,nanosec=10_000_000))
    h._raw(bad)
    assert h.legacy_required
    for i in range(1,4):
        message = h._message(Pose2(1,2,.1), NS(sec=100,nanosec=(i+1)*10_000_000))
        h._raw(message)
    assert not h.legacy_required
    assert not h._legacy_ready()  # Parameter still needs successful SET + GET.


def test_back_to_legacy_cannot_enable_amcl_tf_before_position_agrees():
    h = harness()
    h.scope = False
    h.baseline, h.actual_tf = True, False
    h.last_read = clock.wall
    h.legacy_required = True
    h._parameters(clock.wall)
    h.write.call_async.assert_not_called()


def test_new_initial_request_revokes_old_anchor_and_waits_for_worker_exit():
    h = harness()
    h.policy.transform = Pose2(1,2,.1)
    handle = Mock()
    h.tracking_handle = handle
    h._seed(NS(data=json.dumps(dict(stage='stop_robot', state='starting', terminal=False,
        map_name='map', map_load_id='load', body_generation=1, request_id='new-initial'))))
    assert h.policy.transform is None
    assert h.initial_request_id == 'new-initial'
    handle.cancel_goal_async.assert_called_once()
    assert h.tracking_handle is handle  # No pretend cancellation of remote PCL work.


def test_own_tf_endpoint_is_cached_external_tf_latches_a_fault():
    h = harness()
    del h._ensure_own_tf_gid
    h.get_name, h.get_namespace = lambda:'global_pose_authority', lambda:'/'
    h.get_publishers_info_by_topic = Mock(return_value=[NS(node_name='global_pose_authority',
        node_namespace='/',endpoint_gid=[2])])
    h.actual_tf, h.last_owned_tf = False, 99.
    assert h._ensure_own_tf_gid()
    t = namespace['TransformStamped']()
    t.header.frame_id, t.child_frame_id = 'map', 'odom'
    t.header.stamp = NS(sec=100,nanosec=0)
    message = NS(transforms=[t])
    h.own_tf_messages.append(h._tf_fingerprint(t))
    h._tf_observation(message)
    assert not h.tf_fault and h.own_tf_gid == bytes([2])
    # Temporary discovery silence cannot make our own TF look like an intruder.
    h.get_publishers_info_by_topic.return_value = []
    h._tf_observation(message)
    assert not h.tf_fault
    t.header.stamp = NS(sec=100,nanosec=1)
    h._tf_observation(message)
    assert h.tf_fault and h.policy.transform is None


def test_only_verified_current_initial_request_can_seed_the_authority():
    h = harness()
    h.initial_request_id = 'initial'
    data = dict(stage='validation_passed', state='refined', terminal=False,
        source_pose='refined', candidate_verified=True, request_id='wrong',
        map_name='map',map_load_id='load',body_generation=1,
        pose_stamp=dict(sec=99,nanosec=900_000_000),pose=dict(x=1.,y=2.,yaw=.1),
        metrics=dict(fitness_m=.1,overlap=.8))
    h._seed(NS(data=json.dumps(data)))
    assert h.policy.transform is None
    data['request_id'] = 'initial'
    h._seed(NS(data=json.dumps(data)))
    assert h.seed_request == 'initial' and h.policy.transform is not None


def test_actual_action_result_path_applies_small_correction_rejects_large_one():
    h = harness()
    assert h.policy.accept(Pose2(1,2,.1),Pose2(0,0,0),stamp=99.9,now=100.,
        rmse=.1,overlap=.8,ambiguous=False,initial=True)
    identity = h.policy.identity
    clock.ros = 100.2
    future = Future()
    future.set_result(NS(status=4,result=NS(success=True,candidate_verified=True,
        refined_pose=h._message(Pose2(1.03,2.,.1),NS(sec=100,nanosec=100_000_000)),
        fitness=.1,overlap=.8)))
    h._match_result(future,identity)
    assert math.isclose(h.policy.transform.x,1.03)
    before = h.policy.transform
    clock.ros = 100.4
    future = Future()
    future.set_result(NS(status=4,result=NS(success=True,candidate_verified=True,
        refined_pose=h._message(Pose2(2.7,2.,1.0),NS(sec=100,nanosec=300_000_000)),
        fitness=.1,overlap=.8)))
    h._match_result(future,identity)
    assert h.policy.transform == before and h.policy.reason == 'innovation_exceeded'


def test_already_quarantined_nonpausing_sample_does_not_erase_good_lio_anchor():
    h = harness()
    h.policy.transform = Pose2(1,2,.1)
    h._lio(NS(data=json.dumps(dict(state='warning', code='odometry_sample_quarantined',
        sample_quarantined=True, temporary_stop=False, hard_stop=False))))
    assert h.lio_ok and h.policy.transform is not None
    h._lio(NS(data=json.dumps(dict(state='resetting'))))
    assert not h.lio_ok and h.policy.transform is None


def test_repeated_tf_stamp_cannot_keep_pose_fresh_when_ros_clock_is_frozen():
    h = harness()
    h._parameters = Mock()
    h.actual_tf = False
    h.active = True
    assert h.policy.accept(Pose2(1,2,.1),Pose2(0,0,0),stamp=99.9,now=100.,
        rmse=.1,overlap=.8,ambiguous=False,initial=True)
    h._tick()
    assert json.loads(h.status_pub.publish.call_args.args[0].data)['ready']
    clock.wall += .4
    h._tick()
    assert h.motion_latched
    assert not json.loads(h.status_pub.publish.call_args.args[0].data)['ready']
