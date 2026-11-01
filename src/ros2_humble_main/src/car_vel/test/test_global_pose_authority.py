"""Replay the measured map correction, using the same odom pose on both sides."""
import math
import pytest
from car_vel.global_pose_authority import Pose2, GlobalPoseAuthority, between, compose, lio_step_valid


def quality(**kw):
    return dict(rmse=0.12, overlap=0.8, ambiguous=False, **kw)


def initialized():
    policy = GlobalPoseAuthority()
    policy.set_identity(('xingji', 1, 'map', 'load1', 0))
    assert policy.accept(Pose2(0, 0, 0), Pose2(0, 0, 0), stamp=100.,
                         now=100.1, initial=True, **quality())
    return policy


def test_recorded_170cm_53degree_correction_never_enters_navigation():
    p = initialized()
    before = Pose2(-30.6720123, 20.0455196, math.radians(-179.751666))
    after = Pose2(-30.3803995, 18.3664659, math.radians(-126.717662))
    # Establish the accepted transform at the pre-jump robot location.
    p.transform = between(Pose2(0, 0, 0), before)
    assert not p.accept(after, Pose2(0, 0, 0), stamp=100.2,
                        now=100.3, **quality())
    assert p.transform == before
    assert p.reason == 'innovation_exceeded'


def test_real_small_corrections_are_applied_not_frozen():
    p = initialized()
    assert p.accept(Pose2(0.24, 0.01, 0.02), Pose2(0.2, 0, 0),
                    stamp=100.3, now=100.4, **quality())
    assert compose(p.transform, Pose2(1, 0, 0)).x > 1.03


@pytest.mark.parametrize('change', [dict(rmse=.3), dict(overlap=.4), dict(ambiguous=True)])
def test_quality_never_replaced_by_innovation_only(change):
    p = initialized()
    q = quality(); q.update(change)
    assert not p.accept(Pose2(.01, 0, 0), Pose2(0, 0, 0),
                        stamp=100.3, now=100.4, **q)


def test_expired_duplicate_future_and_identity_change():
    p = initialized()
    assert not p.accept(Pose2(0, 0, 0), Pose2(0, 0, 0),
                        stamp=100., now=100.4, **quality())
    assert not p.accept(Pose2(0, 0, 0), Pose2(0, 0, 0),
                        stamp=100.4, now=102., **quality())
    assert not p.accept(Pose2(0, 0, 0), Pose2(0, 0, 0),
                        stamp=103., now=102., **quality())
    assert p.ready(100.4)
    assert not p.ready(104.)
    p.set_identity(('xingji', 1, 'map', 'load2', 0))
    assert p.transform is None
    assert not p.accept(Pose2(0, 0, 0), Pose2(0, 0, 0),
                        stamp=104., now=104.1, **quality())


def test_turning_far_from_map_origin_uses_robot_innovation():
    p = initialized()
    odom = Pose2(100, -100, .5)
    mapped = Pose2(12, 7, 1.0)
    p.transform = between(odom, mapped)
    corrected = Pose2(12.02, 7.01, 1.001)
    assert p.accept(corrected, odom, stamp=100.2, now=100.3, **quality())
    actual = compose(p.transform, odom)
    assert actual.x == pytest.approx(corrected.x)
    assert actual.y == pytest.approx(corrected.y)


def test_recorded_seventy_sample_window_accepts_normal_updates_rejects_jump():
    import json
    from pathlib import Path
    data = json.loads((Path(__file__).parent/'fixtures/global_jump_20261028.json').read_text())
    p = GlobalPoseAuthority()
    p.set_identity(('xingji', 'recorded-map'))
    accepted = rejected = 0
    for index, row in enumerate(data['frames']):
        result = p.accept(Pose2(*row['mapped']), Pose2(*row['odom']), stamp=row['stamp'],
            now=row['stamp']+.01, initial=index == 0, **quality())
        # The boundary is the independently decoded first anomalous TF, not
        # a label generated from an acceptance-policy result.
        if row['stamp'] < data['event_receive']:
            assert result
            accepted += 1
        else:
            assert not result
            rejected += 1
    assert (accepted, rejected) == (30, 40)
    assert not p.ready(data['frames'][-1]['stamp'])


def test_lio_reset_is_detected_even_if_health_heartbeat_is_still_healthy():
    assert lio_step_valid(Pose2(1,2,.1), Pose2(1.04,2,.12), .1)
    assert not lio_step_valid(Pose2(1,2,.1), Pose2(0,0,0), .1)
    assert not lio_step_valid(Pose2(1,2,.1), Pose2(1,2,.1), -1.)
