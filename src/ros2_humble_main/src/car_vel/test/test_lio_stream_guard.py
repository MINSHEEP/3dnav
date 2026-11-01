import math
import pytest
from car_vel.global_pose_authority import LioStreamGuard,Pose2


def initialized():
    g=LioStreamGuard()
    g.observe(Pose2(0,0,0),100.,100.,100.,True)
    return g


def test_recorded_gaps_hold_for_three_distinct_fresh_samples():
    for gap in (.503402233,.798900366):
        g=initialized()
        g.observe(Pose2(.07,0,0),100.+gap,100.+gap,100.+gap,True)
        deadline=g.deadline
        assert g.recovering and not g.failed and not g.stream_ready
        assert g.details()['step_dt_sec']==pytest.approx(gap,abs=1e-6)
        g.observe(Pose2(.07,0,0),100.+gap,100.+gap+.01,100.+gap+.01,True)
        assert g.samples==1  # Duplicate TF cannot count as recovery evidence.
        for i in (1,2):
            t=100.+gap+.1*i
            g.observe(Pose2(.07,0,0),t,t,t,True)
        assert g.stream_ready and not g.recovering and g.deadline is None
        assert deadline==pytest.approx(100.+gap+g.recovery)
        g.finish()
        assert not g.recovering and g.stream_ready


@pytest.mark.parametrize('stamp,pose,reason',[
    (99.9,Pose2(0,0,0),'lio_time_reversed'),
    (100.1,Pose2(2,0,0),'lio_pose_jump'),
    (100.8,Pose2(.31,0,0),'lio_gap_pose_jump'),
    (100.8,Pose2(0,0,math.radians(21)),'lio_gap_pose_jump'),
    (101.6,Pose2(0,0,0),'source_gap_exceeded'),
    (100.1,Pose2(float('nan'),0,0),'nonfinite_lio'),
])
def test_real_discontinuities_remain_fail_closed(stamp,pose,reason):
    g=initialized()
    g.observe(pose,stamp,max(100.,stamp),max(100.,stamp),True)
    assert g.failed and g.reason==reason


def test_stale_tf_and_lookup_errors_never_forge_freshness_or_extend_deadline():
    g=initialized()
    g.observe(Pose2(0,0,0),100.,100.4,100.4,True)
    deadline=g.deadline
    assert not g.fresh and g.recovering
    g.missing(100.8,100.8,True,'lookup pending')
    assert not g.fresh and g.deadline==deadline
    assert g.details()['lookup_error']=='lookup pending'
    g.missing(101.6,101.6,True,'still missing')
    assert g.failed and g.reason=='source_timeout'


def test_ros_clock_freeze_is_still_bounded_by_monotonic_receipt_age():
    g=initialized()
    g.observe(Pose2(0,0,0),100.,100.,100.4,True)
    assert not g.fresh and g.recovering
    g.observe(Pose2(0,0,0),100.,100.,101.6,True)
    assert g.failed


def test_fresh_stream_finishes_without_waiting_for_global_map_match():
    g=initialized()
    for i in range(1,36):
        t=100.+i*.1
        stamp=100. if i<=4 else t
        g.observe(Pose2(0,0,0),stamp,t,t,True)
    assert not g.failed and not g.recovering and g.stream_ready
    assert g.reason=='ready'
