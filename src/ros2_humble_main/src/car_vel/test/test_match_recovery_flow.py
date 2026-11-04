"""Production localization -> route watchdog -> final velocity recovery flow."""
import importlib.util
import json
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

from car_vel.global_pose_authority import Pose2
from test_global_pose_authority_node import harness, clock
import test_velocity_arbiter_callbacks as velocity_fixture

Message=velocity_fixture.Message

path=Path(__file__).resolve().parents[2]/'nav_service/lzp_service/test/test_global_localization_preflight.py'
spec=importlib.util.spec_from_file_location('recovery_route_fixture',path)
route_fixture=importlib.util.module_from_spec(spec)
spec.loader.exec_module(route_fixture)


def setup(accepted_stamp=97.05):
    h=harness()
    h._sync_tf_parameters=Mock()
    h.actual_tf=False
    h.was_ready=h.active=True
    assert h.policy.accept(Pose2(1,2,.1),Pose2(0,0,0),stamp=accepted_stamp,now=accepted_stamp+.05,
        rmse=.19,overlap=.94,ambiguous=False,initial=True)
    h._odom=lambda stamp=None:(Pose2(0,0,0),NS(sec=int(clock.ros),
        nanosec=round((clock.ros-int(clock.ros))*1e9)))
    velocity=velocity_fixture.VelocityArbiterCallbacksTest(methodName='runTest')
    velocity.setUp()
    velocity.node._navigation_active_callback(Message(True))
    route=route_fixture.manager()
    def tick(at, command=True, identity=True):
        clock.wall=clock.ros=velocity.now=at
        h.lio_at=at
        if identity:h.mode_at=h.body_at=at
        h._tick()
        status=h.status_pub.publish.call_args.args[0]
        route._global_localization_callback(status)
        velocity.body('xingji');velocity.terrain(True)
        velocity.node._lio_pause_callback(Message(False))
        velocity.node._navigation_pause_callback(Message(False))
        velocity.node._global_localization_callback(status)
        if command:velocity.command()
        velocity.node._tick()
        return json.loads(status.data)
    return h,velocity,route,tick


def match(h,at,mapped=None):
    stamp=NS(sec=int(at),nanosec=round((at-int(at))*1e9))
    pose=h._message(mapped or Pose2(1.01,2,.1),stamp)
    future=Future()
    future.set_result(NS(status=4,result=NS(success=True,candidate_verified=True,
        refined_pose=pose,fitness=.219,overlap=.94,transform_model='se3_map_odom_v1')))
    h._match_result(future,h.policy.identity)


def test_gap_holds_motion_keeps_goal_and_resumes_after_one_match_and_new_command():
    h,v,route,tick=setup()
    assert tick(100.)['ready']
    count=h.broadcaster.sendTransform.call_count
    status=tick(100.2)
    assert status['recovering'] and not status['ready']
    assert not h.motion_latched
    assert h.broadcaster.sendTransform.call_count>count
    assert v.last_velocity()==(0.,0.)
    route._request_stop.assert_not_called()
    tick(100.4);match(h,100.3)
    assert tick(100.5,command=False)['ready']
    assert v.last_velocity()==(0.,0.), 'pre-recovery command must stay discarded'
    assert tick(100.6)['ready']
    assert v.last_velocity()==(.2,.1)
    route._request_stop.assert_not_called()
    assert route._state=='navigating'


def test_unresolved_gap_stops_by_deadline_and_late_matches_never_restart_task():
    h,v,route,tick=setup()
    tick(100.)
    for i in range(1,22):tick(100.+i*.1)
    assert h.motion_latched
    assert v.last_velocity()==(0.,0.)
    route._request_stop.assert_called_with('global_localization_untrusted')
    # Actual cancellation changes the manager to idle; no automatic restart.
    route._mode,route._state='idle','stopped'
    match(h,102.05)
    assert not tick(102.2)['ready']
    assert route._state=='stopped' and v.last_velocity()==(0.,0.)


def test_tf_delay_keeps_route_and_zero_velocity_until_stream_and_matches_recover():
    h,v,route,tick=setup(99.9)
    assert tick(100.)['ready']
    h._odom=lambda stamp=None:(Pose2(0,0,0),NS(sec=100,nanosec=0))
    sent=h.broadcaster.sendTransform.call_count
    status=tick(100.35)
    assert status['reason']=='sensor_recovery' and not status['sensor_fresh']
    assert json.loads(v.node._guard_publisher.publish.call_args.args[0].data)['reason_code']=='global_localization_recovering'
    assert json.loads(v.node._guard_publisher.publish.call_args.args[0].data)['blocked']
    assert h.broadcaster.sendTransform.call_count==sent
    assert not h.motion_latched and v.last_velocity()==(0.,0.)
    h._odom=lambda stamp=None:(Pose2(.07,0,0),NS(sec=int(clock.ros),
        nanosec=round((clock.ros-int(clock.ros))*1e9)))
    for t in (100.8,100.9):
        status=tick(t)
        assert status['recovering'] and not status['sensor_stream_ready']
        assert v.last_velocity()==(0.,0.)
    assert tick(101.)['sensor_stream_ready']
    tick(101.2);match(h,101.1)
    assert tick(101.3,command=False)['ready']
    assert v.last_velocity()==(0.,0.)
    assert tick(101.4)['ready'] and v.last_velocity()==(.2,.1)
    route._request_stop.assert_not_called()


def test_identity_hold_reaches_final_zero_velocity_and_preserves_original_task():
    h,v,route,tick=setup(99.9)
    for i in range(21):tick(100.+i*.1,identity=False)
    status=tick(102.05,identity=False)
    assert status['scope']=='xingji_3d' and status['reason']=='identity_recovery'
    assert status['recovering'] and not h.motion_latched
    assert v.last_velocity()==(0.,0.)
    route._request_stop.assert_not_called()
    tick(102.1);match(h,102.1)
    assert tick(102.2,command=False)['ready']
    assert v.last_velocity()==(0.,0.), 'old command cannot restart after recovery'
    tick(102.3)
    assert v.last_velocity()==(.2,.1)
    route._request_stop.assert_not_called()


def test_borderline_candidate_never_moves_anchor_or_bypasses_final_velocity_hold():
    h,v,route,tick=setup(99.9)
    tick(100.)
    match(h,100.,Pose2(1.205,2,.1))
    assert tick(100.1)['recovering']
    assert v.last_velocity()==(0.,0.)
    route._request_stop.assert_not_called()
    match(h,100.1)
    assert tick(100.2,command=False)['ready']
    assert v.last_velocity()==(0.,0.)
    tick(100.3)
    assert v.last_velocity()==(.2,.1)
    route._request_stop.assert_not_called()
