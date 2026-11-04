"""First failure must survive anchor reset and later healthy sensor ticks."""
import json
from concurrent.futures import Future
from types import SimpleNamespace as NS

import test_global_pose_authority_node as fixture
from car_vel.global_pose_authority import Pose2


def anchored():
    h = fixture.harness()
    h._sync_tf_parameters = lambda _: None
    h.actual_tf = False
    h.active = h.was_ready = True
    h.policy.accept(Pose2(1, 2, .1), Pose2(0, 0, 0), stamp=99.9, now=100.,
                    rmse=.17, overlap=.85, ambiguous=False, initial=True)
    return h


def test_verified_idle_anchor_loss_preserves_startup_failure_evidence():
    h = anchored()
    h.active = False
    h._capture_failure('source_timeout', 'source stopped before route launch')
    assert h.failure['code'] == 'source_timeout'
    assert h.failure['accepted_stamp'] == 99.9


def test_health_reset_retains_original_cause_after_lio_recovers():
    h = anchored()
    h._lio(NS(data=json.dumps(dict(state='fault', reason='position_step',
                                    consecutive_samples=8, required_samples=8))))
    h._tick()
    status = json.loads(h.status_pub.publish.call_args.args[0].data)
    assert status['failure']['code'] == 'lio_health_rejected'
    assert status['failure']['lio_health']['reason'] == 'position_step'
    assert status['failure']['accepted_stamp'] == 99.9
    first = status['failure']
    h._lio(NS(data='{"state":"healthy"}'))
    fixture.clock.ros = fixture.clock.wall = 100.1
    h._tick()
    status = json.loads(h.status_pub.publish.call_args.args[0].data)
    assert status['failure'] == first
    assert 'position_step' in status['error']
    assert not status['ready']


def test_invalid_tf_error_is_not_overwritten_by_generic_motion_lock():
    h = anchored()
    h._odom = lambda stamp=None: (_ for _ in ()).throw(ValueError('invalid orientation'))
    h._tick()
    status = json.loads(h.status_pub.publish.call_args.args[0].data)
    assert status['failure']['code'] == 'invalid_lio_pose'
    assert status['failure']['accepted_stamp'] == 99.9
    assert 'invalid orientation' in status['error']
    assert h.motion_latched


def test_recorded_server_rejections_keep_the_last_reason_when_recovery_expires():
    h = anchored()
    # Server full-3D fit passed at RMSE .158 / 90.6% overlap, but its
    # projected transform failed .223965 > .22; subsequent z fits exceeded .45.
    message = ('局部匹配：三维配准垂直修正 z 0.565013 m，上限 0.45 m；'
               '投影RMSE 0.223965 m，上限 0.22')
    for sample in range(51):
        now = 100. + sample / 10.
        fixture.clock.ros = fixture.clock.wall = now
        h.mode_at = h.body_at = h.lio_at = now
        stamp = NS(sec=int(now), nanosec=round((now-int(now))*1e9))
        h._odom = lambda requested=None, measured=stamp: (Pose2(0, 0, 0), measured)
        future = Future()
        future.set_result(NS(status=6, result=NS(success=False,
            candidate_verified=False, message=message)))
        h._match_result(future, h.policy.identity)
        h._tick()
    status = json.loads(h.status_pub.publish.call_args.args[0].data)
    assert status['reason'] == 'recovery_timeout'
    assert status['failure']['code'] == 'matching_recovery_timeout'
    assert status['failure']['accepted_stamp'] == 99.9
    assert '.565013' in status['error']
    assert status['sensor_fresh']
