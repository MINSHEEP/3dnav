"""Recorded 12:38:03 native soft pause must not cancel a verified 3-D route."""
import json
from types import SimpleNamespace as NS
from car_vel.global_pose_authority import Pose2,compose
from test_match_recovery_flow import setup,match


def warning(**changes):
    return dict(dict(state='warning',code='odometry_sample_quarantined',reason='position_step',
        hard_stop=False,sample_quarantined=True,temporary_stop=True,recovery_only=False,
        has_planar_or_yaw_corroboration=True,consecutive_samples=1,required_samples=8,
        pose_linear_speed=.344,estimator_linear_speed=.256,pose_angular_speed=.077,
        estimator_angular_speed=.035,pose_vertical_speed=.043,estimator_vertical_speed=.052),**changes)


def test_recorded_slow_quarantined_step_keeps_anchor_and_route_while_paused():
    h,v,route,tick=setup(99.9)
    tick(100.)
    original=h.policy.transform
    h._lio(NS(data=json.dumps(warning())))
    status=tick(100.1)
    assert h.policy.transform==original
    assert status['recovering'] and not status['ready'] and not h.motion_latched
    assert v.last_velocity()==(0.,0.)
    route._request_stop.assert_not_called()
    h._lio(NS(data=json.dumps(dict(state='recovering',code='lio_auto_recovery',
        healthy_samples=3,required_samples=5))))
    tick(100.3)
    h._lio(NS(data=json.dumps(dict(state='healthy'))))
    assert tick(100.4,command=False)['ready']
    assert v.last_velocity()==(0.,0.)
    tick(100.5)
    assert v.last_velocity()==(.2,.1)
    route._request_stop.assert_not_called()


def test_short_tf_staleness_recovers_without_two_redundant_map_matches():
    h,v,route,tick=setup(99.9)
    tick(100.)
    h._odom=lambda stamp=None:(Pose2(0,0,0),NS(sec=100,nanosec=0))
    assert tick(100.32)['recovering']
    h._odom=lambda stamp=None:(Pose2(.10,0,0),NS(sec=int(v.now),nanosec=round((v.now-int(v.now))*1e9)))
    tick(100.4);tick(100.5)
    status=tick(100.6,command=False)
    assert status['ready'],status
    assert v.last_velocity()==(0.,0.)
    tick(100.7)
    assert v.last_velocity()==(.2,.1)
    route._request_stop.assert_not_called()


def test_fast_or_unproven_step_does_not_get_the_slow_transient_allowance():
    for changes in (dict(pose_linear_speed=2.),dict(hard_stop=True),dict(sample_quarantined=False),
                    dict(estimator_angular_speed=float('nan')),dict(required_samples=0)):
        h,v,route,tick=setup(99.9)
        tick(100.)
        h._lio(NS(data=json.dumps(warning(**changes))))
        assert not tick(100.1)['ready']
        assert v.last_velocity()==(0.,0.)


def test_recorded_598ms_gap_and_late_good_matches_resume_same_waypoint():
    h,v,route,tick=setup(99.9)
    tick(100.)
    h._odom=lambda stamp=None:(Pose2(0,0,0),NS(sec=100,nanosec=50_000_000))
    tick(100.05);tick(100.37)
    h._lio(NS(data=json.dumps(warning())))
    assert tick(100.55)['recovering']
    odom=Pose2(.190825,0,.037213)
    h._odom=lambda stamp=None:(odom,NS(sec=int(v.now),nanosec=round((v.now-int(v.now))*1e9)))
    for at in (100.65,100.75,100.85):
        assert tick(at)['recovering']
    h._lio(NS(data=json.dumps(dict(state='healthy'))))
    tick(100.95);match(h,100.95,compose(h.policy.transform,odom))
    assert tick(101.,command=False)['ready']
    assert v.last_velocity()==(0.,0.)
    tick(101.1)
    assert v.last_velocity()==(.2,.1)
    route._request_stop.assert_not_called()
