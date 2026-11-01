"""Replay the 11:19–11:20 failures through production localization callbacks."""
import json
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest
from car_vel.global_pose_authority import GlobalPoseAuthority, Pose2
from test_global_pose_authority_node import clock
from test_lio_stream_recovery import setup, tick


def result(h, at, x=1.01):
    stamp=NS(sec=int(at),nanosec=round((at-int(at))*1e9))
    future=Future()
    future.set_result(NS(status=4,result=NS(success=True,candidate_verified=True,
        refined_pose=h._message(Pose2(x,2,.1),stamp),fitness=.18,overlap=.86,
        transform_model='se3_map_odom_v1')))
    h._match_result(future,h.policy.identity)


def test_identity_heartbeat_delay_holds_without_exiting_3d_or_forgetting_anchor():
    h=setup()
    anchor=h.policy.transform
    h._odom=lambda stamp=None:(Pose2(0,0,0),NS(sec=int(clock.ros),
        nanosec=round((clock.ros-int(clock.ros))*1e9)))
    for at in (100.2,100.4,100.6,100.8,101.,101.2,101.4,101.6,101.8,102.01):
        clock.wall=clock.ros=at
        h.lio_at=at
        h._tick()
    status=json.loads(h.status_pub.publish.call_args.args[0].data)
    assert status['scope']=='xingji_3d'
    assert h.policy.transform==anchor and not h.legacy_required
    assert status['recovering'] and not status['ready'] and not h.motion_latched
    assert status['reason']=='identity_recovery'
    tick(h,102.1,102.1);result(h,102.1)
    tick(h,102.3,102.3);result(h,102.3)
    assert tick(h,102.4,102.4)['ready']


def test_repeated_350ms_tf_jitter_does_not_erase_good_recovery_matches():
    h=setup()
    tick(h,100.35,100.)
    for at in (100.8,100.9,101.):tick(h,at,at)
    result(h,101.)
    deadline=h.policy.recovery_deadline
    status=tick(h,101.35,101.)
    assert not status['ready'] and not status['sensor_fresh']
    for at in (101.4,101.5):
        assert tick(h,at,at)['recovering']
    assert tick(h,101.6,101.6)['ready']
    assert h.policy.accepted_stamp==101.
    assert not h.motion_latched and deadline==pytest.approx(103.8)


def test_one_borderline_candidate_holds_then_recovers_with_one_fresh_match():
    h=setup()
    original=h.policy.transform
    tick(h,100.1,100.1)
    result(h,100.1,x=1.205)
    assert h.policy.transform==original
    status=tick(h,100.2,100.2)
    assert status['recovering'] and not status['ready'] and not h.motion_latched
    result(h,100.2)
    assert tick(h,100.3,100.3)['ready']


def test_hard_failure_cannot_report_accepted_or_replace_the_last_anchor():
    p=GlobalPoseAuthority()
    kwargs=dict(rmse=.18,overlap=.86,ambiguous=False)
    p.accept(Pose2(0,0,0),Pose2(0,0,0),stamp=100.,now=100.,initial=True,**kwargs)
    anchor=p.transform
    assert not p.accept(Pose2(2,0,0),Pose2(0,0,0),stamp=100.1,now=100.2,**kwargs)
    reason=p.reason
    assert p.phase(100.2)=='failed'
    assert not p.accept(Pose2(.01,0,0),Pose2(0,0,0),stamp=100.3,now=100.4,**kwargs)
    assert p.reason==reason and p.transform==anchor


def test_stream_diagnostic_preserves_concrete_failure_reason():
    h=setup()
    status=tick(h,101.6,101.6)
    assert status['lio_timing']['failure_reason']=='source_gap_exceeded'


def test_identity_hold_deadline_does_not_slide_when_matches_keep_arriving():
    h=setup()
    h._odom=lambda stamp=None:(Pose2(0,0,0),NS(sec=int(clock.ros),
        nanosec=round((clock.ros-int(clock.ros))*1e9)))
    for i in range(1,53):
        at=100.+i*.1
        clock.wall=clock.ros=at
        h.lio_at=at
        h._tick()
        if i%2==0:result(h,at)
    status=json.loads(h.status_pub.publish.call_args.args[0].data)
    assert h.motion_latched and not status['ready'] and not status['recovering']
    assert h.policy.reason=='recovery_timeout'
