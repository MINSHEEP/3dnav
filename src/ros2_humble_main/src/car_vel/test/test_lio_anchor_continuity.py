"""Localization anchor continuity is independent of permission to move."""
import pytest
from car_vel.global_pose_authority import LioAnchorContinuity


def warning(**changes):
    return dict(dict(state='warning', code='odometry_sample_quarantined',
        reason='scan_matching_quality', hard_stop=False, sample_quarantined=True,
        temporary_stop=True, recovery_only=True,
        has_planar_or_yaw_corroboration=False), **changes)


def test_quality_recovery_is_bounded_and_cannot_be_extended_by_heartbeats():
    policy = LioAnchorContinuity()
    assert policy.observe({'state':'healthy'}, 10.)
    assert policy.observe(warning(), 10.1)
    assert policy.paused
    assert policy.observe(warning(), 10.8)
    assert policy.observe({'state':'recovering','code':'lio_auto_recovery',
                           'healthy_samples':1,'required_samples':5}, 11.)
    assert not policy.observe(warning(), 11.11)
    assert not policy.allows_anchor(11.12)


@pytest.mark.parametrize('bad', [
    warning(reason='position_step'), warning(hard_stop=True),
    warning(recovery_only=False), warning(sample_quarantined=False),
    warning(has_planar_or_yaw_corroboration=True),
    {'state':'resetting'}, {'state':'fault'}, None, [],
])
def test_unproven_recovery_or_actual_fault_requires_new_anchor(bad):
    policy = LioAnchorContinuity()
    assert policy.observe({'state':'healthy'}, 10.)
    assert not policy.observe(bad, 10.1)
    assert not policy.observe({'state':'recovering','code':'lio_auto_recovery'}, 10.2)


def test_recovering_at_startup_is_not_a_valid_anchor():
    policy = LioAnchorContinuity()
    assert not policy.observe({'state':'recovering','code':'lio_auto_recovery'},10.)
    assert not policy.observe(warning(),10.1)


def test_silence_expires_allowance_and_late_healthy_does_not_hide_it():
    for poll in (False, True):
        policy = LioAnchorContinuity()
        policy.observe({'state':'healthy'},10.)
        policy.observe(warning(),10.1)
        if poll:
            assert not policy.allows_anchor(11.2)
        else:
            assert not policy.observe({'state':'healthy'},11.2)
        assert policy.observe({'state':'healthy'},11.3)
