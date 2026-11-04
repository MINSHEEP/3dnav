"""Recorded short TF delays and five-scan health recovery reach final velocity."""
import ast
import json
from types import SimpleNamespace as NS

import pytest

from car_vel.global_pose_authority import Pose2
from test_match_recovery_flow import setup, match
from test_global_pose_authority_node import clock, SOURCE


def configured():
    h, v, route, tick = setup(99.9)
    # The production adapter installs this scoped stream budget. Compatibility
    # guard tests keep the old strict default; measurements are never restamped.
    h.lio_stream.freshness_sec = .45
    return h, v, route, tick


def warning():
    return dict(state='warning', code='odometry_sample_quarantined', reason='position_step',
        sample_quarantined=True, temporary_stop=True, hard_stop=False,
        consecutive_samples=1, required_samples=8,
        pose_linear_speed=.418, estimator_linear_speed=.242,
        pose_angular_speed=.030, estimator_angular_speed=.025,
        pose_vertical_speed=.014, estimator_vertical_speed=.044)


def test_recorded_305ms_boundary_does_not_repeatedly_clear_final_velocity():
    h, v, route, tick = configured()
    assert tick(100.)['ready']
    for index in range(1, 9):
        base = 100.+index*.4
        h._odom = lambda stamp=None, t=base: (
            Pose2(.01*index, 0, 0), NS(sec=int(t), nanosec=round((t-int(t))*1e9)))
        clock.wall = clock.ros = v.now = base
        match(h, base)
        assert tick(base)['ready']
        status = tick(base+.305)
        assert status['ready'] and not status['recovering'], status
        assert v.last_velocity() == (.2, .1)
    route._request_stop.assert_not_called()


def test_delay_above_tolerance_still_stops_until_three_new_samples():
    h, v, route, tick = configured()
    tick(100.)
    h._odom = lambda stamp=None: (Pose2(0, 0, 0), NS(sec=100, nanosec=0))
    assert tick(100.50)['recovering']
    assert v.last_velocity() == (0., 0.)
    h._odom = lambda stamp=None: (Pose2(.01, 0, 0), NS(sec=int(clock.ros),
        nanosec=round((clock.ros-int(clock.ros))*1e9)))
    for at in (100.6, 100.7):
        assert tick(at)['recovering']
        assert v.last_velocity() == (0., 0.)
    match(h, 100.7)
    assert tick(100.8, command=False)['ready']
    assert v.last_velocity() == (0., 0.)
    tick(100.9)
    assert v.last_velocity() == (.2, .1)
    route._request_stop.assert_not_called()


def test_recorded_1484ms_health_recovery_keeps_anchor_and_waits_for_fresh_velocity():
    h, v, route, tick = configured()
    tick(100.)
    original = h.policy.transform
    for index, elapsed in enumerate((.1, .395, .825, 1.125, 1.4, 1.584)):
        clock.wall = clock.ros = v.now = 100.+elapsed
        health = (warning() if index == 0 else {'state':'healthy'} if index == 5 else
                  dict(state='recovering', code='lio_auto_recovery',
                       healthy_samples=index, required_samples=5))
        h._lio(NS(data=json.dumps(health)))
        status = tick(100.+elapsed, command=False)
        assert h.policy.transform == original and not h.motion_latched, status
        if index < 5:
            assert status['recovering'] and v.last_velocity() == (0., 0.)
    assert status['ready'] and v.last_velocity() == (0., 0.)
    tick(101.7)
    assert v.last_velocity() == (.2, .1)
    route._request_stop.assert_not_called()


def test_health_recovery_has_fixed_bound_and_fault_still_revokes_anchor():
    h, v, route, tick = configured()
    tick(100.)
    h._lio(NS(data=json.dumps(warning())))
    for index in range(1, 31):
        at = 100.+index*.1
        clock.wall = clock.ros = at
        h._lio(NS(data=json.dumps(dict(state='recovering', code='lio_auto_recovery',
            healthy_samples=1, required_samples=5))))
        assert tick(at)['recovering'] and not h.motion_latched
    assert not tick(103.05)['ready']
    assert h.motion_latched and h.policy.transform is None
    assert v.last_velocity() == (0., 0.)


@pytest.mark.parametrize('body,mode', [('xingji','map_2d'), ('gangben_zsl1','terrain_3d')])
def test_legacy_health_budget_stays_one_second(body, mode):
    h, _, _, tick = setup(99.9)
    h.body, h.mode = body, mode
    h._lio(NS(data=json.dumps(warning())))
    assert h.lio_continuity.max_pause_sec == 1.


def test_production_installs_scoped_stream_budget():
    tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Name) and n.func.id == 'LioStreamGuard']
    assert any(any(k.arg == 'freshness_sec' and isinstance(k.value, ast.Constant)
                   and k.value.value == .45 for k in n.keywords) for n in calls)


def test_native_compensation_is_revoked_on_stop_and_real_health_fault():
    h, v, route, tick = configured()
    assert tick(100.)['interval_compensation']
    assert h.interval_policy_pub.publish.call_args.args[0].data is True
    h.active = False
    tick(100.1)
    assert h.interval_policy_pub.publish.call_args.args[0].data is False
    h.active = True
    tick(100.2)
    clock.wall = clock.ros = 100.3
    h._lio(NS(data=json.dumps(dict(state='fault',hard_stop=True,reason='estimator_invalid'))))
    assert not tick(100.3)['ready'] and h.motion_latched
    assert h.interval_policy_pub.publish.call_args.args[0].data is False
    assert h.failure['lio_health']['reason'] == 'estimator_invalid'
    assert v.last_velocity() == (0., 0.)


@pytest.mark.parametrize('body,mode', [('xingji','map_2d'), ('gangben_zsl1','terrain_3d')])
def test_native_compensation_is_disabled_in_legacy_scopes(body, mode):
    h, _, _, tick = configured()
    tick(100.)
    h.body, h.mode = body, mode
    tick(100.1)
    assert h.interval_policy_pub.publish.call_args.args[0].data is False
