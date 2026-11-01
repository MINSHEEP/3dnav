"""Healthy LIO recovery must not expire while the independent map match runs."""
from types import SimpleNamespace as NS

from car_vel.global_pose_authority import LioStreamGuard, Pose2
from test_match_recovery_flow import setup, match


def test_stream_with_new_samples_does_not_wait_for_a_map_match():
    stream = LioStreamGuard()
    stream.observe(Pose2(0, 0, 0), 100., 100., 100., True)
    stream.observe(Pose2(0, 0, 0), 100., 100.35, 100.35, True)
    for index in range(27):
        at = 101.3 + index * .1
        stream.observe(Pose2(.01, 0, 0), at, at+.04, at+.04, True)
    assert not stream.failed
    assert stream.stream_ready and not stream.recovering


def test_recorded_gap_keeps_anchor_until_independent_match_completes():
    h, velocity, route, tick = setup(99.9)
    for index in range(6):
        tick(100. + index * .1)
    match(h, 100.5)
    h._odom = lambda stamp=None: (Pose2(0, 0, 0), NS(sec=100, nanosec=500_000_000))
    assert tick(100.85)['recovering']
    h._odom = lambda stamp=None: (Pose2(.009386, 0, 0), NS(
        sec=int(velocity.now), nanosec=round((velocity.now-int(velocity.now))*1e9)))
    for index in range(24):
        status = tick(101.8 + index * .1)
        assert status['recovering'] and not h.motion_latched
        assert velocity.last_velocity() == (0., 0.)
    # Original accepted anchor is4s old; still inside its unchanged5s budget.
    match(h, 103.9)
    status = tick(104.2, command=False)
    assert status['ready'] and not h.motion_latched
    assert velocity.last_velocity() == (0., 0.)
    tick(104.3)
    assert velocity.last_velocity() == (.2, .1)
    route._request_stop.assert_not_called()
