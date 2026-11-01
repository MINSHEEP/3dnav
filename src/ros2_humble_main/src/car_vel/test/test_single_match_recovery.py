"""One fully validated new map match may release the bounded Xingji3D hold."""
import pytest

from car_vel.global_pose_authority import GlobalPoseAuthority, Pose2
from test_match_recovery_flow import setup, match


def initialized():
    policy = GlobalPoseAuthority()
    assert policy.accept(Pose2(0, 0, 0), Pose2(0, 0, 0), stamp=100.,
        now=100.1, rmse=.19, overlap=.94, ambiguous=False, initial=True)
    assert policy.phase(103.1) == 'recovering'
    return policy


def accepted(policy, stamp=103.5, **changes):
    kwargs = dict(stamp=stamp, now=stamp+.1, rmse=.219, overlap=.94,
                  ambiguous=False)
    kwargs.update(changes)
    return policy.accept(Pose2(.01, 0, 0), Pose2(0, 0, 0), **kwargs)


def test_one_fresh_verified_match_resumes_inside_original_window():
    policy = initialized()
    assert accepted(policy)
    assert policy.recovery_samples == 1
    assert policy.phase(103.6) == 'ready'
    assert policy.recovery_deadline is None


def test_one_match_recovery_retains_route_and_requires_new_velocity():
    h, velocity, route, tick = setup()
    assert tick(100.)['ready']
    assert tick(100.2)['recovering']
    assert velocity.last_velocity() == (0., 0.)
    tick(100.4)
    match(h, 100.3)
    assert tick(100.5, command=False)['ready']
    assert velocity.last_velocity() == (0., 0.), 'old velocity must stay discarded'
    assert tick(100.6)['ready']
    assert velocity.last_velocity() == (.2, .1)
    route._request_stop.assert_not_called()
    assert route._state == 'navigating'
    status = tick(100.7)
    assert status['recovery_matches_required'] == 1


def test_recorded_pass_fail_pass_cadence_does_not_cancel_recovered_route():
    h, velocity, route, tick = setup(97.284045)
    assert tick(100.)['ready']
    for index in range(1, 10):
        status = tick(100. + .1 * index)
    assert status['recovering']
    match(h, 100.6838107)
    assert tick(101., command=False)['ready']
    assert velocity.last_velocity() == (0., 0.)
    h.policy.match_failed()  # Next native result rejected; keep quality gates.
    for index in range(1, 14):
        assert tick(101. + .1 * index)['ready']
    match(h, 102.1838107)
    assert tick(102.4)['ready']
    assert not h.motion_latched
    route._request_stop.assert_not_called()


@pytest.mark.parametrize('changes', [
    dict(overlap=.54), dict(rmse=.221), dict(ambiguous=True),
    dict(stamp=100.), dict(stamp=102., now=103.6),
])
def test_bad_or_old_single_result_never_releases_hold(changes):
    policy = initialized()
    assert not accepted(policy, **changes)
    assert policy.phase(103.6) != 'ready'


def test_stream_or_manual_pause_still_prevents_single_match_release():
    policy = initialized()
    assert accepted(policy)
    assert policy.phase(103.6, can_resume=False) == 'recovering'
    assert policy.phase(103.7, can_resume=True) == 'ready'


def test_single_result_after_terminal_timeout_cannot_restart():
    policy = initialized()
    assert policy.phase(105.01) == 'failed'
    assert not accepted(policy, stamp=105.1)
    assert policy.phase(105.2) == 'failed'
