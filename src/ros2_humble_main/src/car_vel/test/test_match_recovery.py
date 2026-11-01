"""Bounded measured-match recovery, independent of ROS transport."""
import pytest
from car_vel.global_pose_authority import GlobalPoseAuthority, Pose2


def initialized():
    p = GlobalPoseAuthority()
    assert p.accept(Pose2(0,0,0), Pose2(0,0,0), stamp=100.,now=100.1,
        rmse=.19,overlap=.94,ambiguous=False,initial=True)
    return p


def accept(p, stamp, **changes):
    return p.accept(Pose2(.01,0,0),Pose2(0,0,0), **dict(dict(
        stamp=stamp,now=stamp+.1,rmse=.219,overlap=.94,ambiguous=False),**changes))


def test_recorded_moderate_quality_is_tracking_only():
    p = initialized()
    assert accept(p,100.2)
    q=GlobalPoseAuthority()
    assert not q.accept(Pose2(0,0,0),Pose2(0,0,0),stamp=100.,now=100.1,
        rmse=.219,overlap=.94,ambiguous=False,initial=True)
    assert not accept(p,100.4,rmse=.221)


def test_recovery_requires_one_new_good_result_within_fixed_deadline():
    p=initialized()
    assert p.phase(102.14)=='ready'  # Previously expired at 2 seconds.
    assert p.phase(103.1)=='recovering'
    deadline=p.recovery_deadline
    assert accept(p,103.5)
    assert p.phase(103.6)=='ready'
    p.begin_recovery(103.7, 1., 103.7)
    assert not accept(p,103.5)  # A duplicate must reset, not advance the count.
    assert p.phase(103.7)=='recovering'
    assert accept(p,103.9)
    assert p.recovery_deadline < deadline
    assert p.phase(104.)=='ready'


def test_failure_between_good_results_resets_count_and_deadline_never_slides():
    p=initialized()
    assert p.phase(103.1)=='recovering'
    deadline=p.recovery_deadline
    assert accept(p,103.4)
    p.match_failed()
    assert p.recovery_samples==0
    assert p.phase(103.5)=='recovering'
    assert accept(p,104.9)
    assert p.recovery_deadline==deadline
    assert p.phase(104.95,can_resume=False)=='recovering'
    assert p.phase(105.01)=='failed'
    assert not accept(p,105.1)
    assert p.phase(105.2)=='failed'


@pytest.mark.parametrize('bad', [dict(mapped=Pose2(2,0,0)),dict(ambiguous=True)])
def test_large_jump_or_ambiguity_is_never_auto_recovered(bad):
    p=initialized()
    assert p.phase(103.1)=='recovering'
    mapped=bad.get('mapped',Pose2(.01,0,0))
    assert not p.accept(mapped,Pose2(0,0,0),stamp=103.2,now=103.3,
        rmse=.19,overlap=.94,ambiguous=bad.get('ambiguous',False))
    assert p.phase(103.3)=='failed'
    assert not accept(p,103.4)
    assert p.phase(103.5)=='failed'
