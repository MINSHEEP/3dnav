"""Replay delayed TF reads through the production localization adapter."""
import json
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock

import test_global_pose_authority_node as fixture
from car_vel.global_pose_authority import Pose2


def setup():
    h=fixture.harness()
    h._sync_tf_parameters=Mock()
    h.actual_tf=False
    h.was_ready=h.active=True
    h.policy.accept(Pose2(1,2,.1),Pose2(0,0,0),stamp=99.9,now=100.,
        rmse=.17,overlap=.85,ambiguous=False,initial=True)
    h._tick()
    return h


def tick(h,now,stamp,x=0.):
    fixture.clock.ros=fixture.clock.wall=now
    h.mode_at=h.body_at=h.lio_at=now
    measured_stamp=NS(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))
    h._odom=lambda stamp=None:(Pose2(x,0,0),measured_stamp)
    h._tick()
    return json.loads(h.status_pub.publish.call_args.args[0].data)


def test_brief_tf_staleness_is_a_zero_speed_hold_not_a_permanent_latch():
    h=setup()
    original=h.policy.transform
    status=tick(h,100.35,100.)
    assert not h.motion_latched
    assert h.policy.transform==original
    assert not status['ready'] and status['recovering']


def test_recorded_800ms_gap_preserves_anchor_for_revalidation():
    h=setup()
    original=h.policy.transform
    status=tick(h,100.798900366,100.798900366,.07)
    assert h.policy.transform==original, 'a bounded sample gap alone is not a reset'
    assert not h.motion_latched
    assert not status['ready'] and status['recovering']


def test_transient_tf_lookup_error_is_bounded_and_does_not_publish_fake_tf():
    h=setup()
    count=h.broadcaster.sendTransform.call_count
    fixture.clock.ros=fixture.clock.wall=100.4
    def missing(stamp=None):raise RuntimeError('TF lookup temporarily unavailable')
    h._odom=missing
    h._tick()
    status=json.loads(h.status_pub.publish.call_args.args[0].data)
    assert status['recovering'] and not status['sensor_fresh'] and not h.motion_latched
    assert h.broadcaster.sendTransform.call_count==count
    assert status['lio_timing']['lookup_error']=='TF lookup temporarily unavailable'
    fixture.clock.ros=fixture.clock.wall=101.6
    h._tick()
    assert h.motion_latched and h.policy.transform is None


def test_invalid_pose_is_not_treated_as_a_transient_lookup_error():
    h=setup()
    def invalid(stamp=None):raise ValueError('invalid orientation')
    h._odom=invalid
    h._tick()
    assert h.motion_latched and h.policy.transform is None


def test_tracking_goal_has_two_second_budget_without_overlapping_workers(monkeypatch):
    h=setup()
    monkeypatch.setitem(fixture.namespace,'Relocalize',NS(Goal=lambda:NS()))
    del h._match
    h.action=Mock()
    h.action.server_is_ready.return_value=True
    pending=Future()
    h.action.send_goal_async.return_value=pending
    h.next_match=0.
    h._match(Pose2(0,0,0),NS(sec=100,nanosec=0),100.)
    goal=h.action.send_goal_async.call_args.args[0]
    assert goal.timeout_sec==2. and goal.tracking and goal.robust_validation
    assert goal.search_xy==h.policy.max_xy
    assert goal.search_yaw==h.policy.max_yaw
    h._match(Pose2(0,0,0),NS(sec=101,nanosec=0),101.)
    h.action.send_goal_async.assert_called_once()
    pending.set_result(NS(accepted=False))
