"""Verified 3D geometry must survive every production transport boundary."""
import math
import random
import json
from concurrent.futures import Future
from unittest.mock import Mock
from types import SimpleNamespace as NS

import test_global_pose_authority_node as fixture
from car_vel.global_pose_authority import GlobalPoseAuthority, Pose2, compose, inverse, between
from car_vel.rigid_pose import Pose3, TRANSFORM_MODEL


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return NS(x=sr*cp*cy-cr*sp*sy, y=cr*sp*cy+sr*cp*sy,
              z=cr*cp*sy-sr*sp*cy, w=cr*cp*cy+sr*sp*sy)


def pose_message():
    return NS(header=NS(), pose=NS(covariance=[0.]*36,
        pose=NS(position=NS(x=0., y=0., z=0.),
                orientation=NS(x=0., y=0., z=0., w=1.))))


def test_verified_native_pose_parser_retains_height_and_tilt():
    measured = NS(position=NS(x=-21.820239, y=20.422333, z=.434135),
                  orientation=quaternion(-.025442109, -.023617731, -3.089208062))
    parsed = fixture.namespace['pose2'](measured)
    assert math.isclose(getattr(parsed, 'z', 0.), .434135, abs_tol=1e-8)
    assert math.isclose(getattr(parsed, 'roll', 0.), -.025442109, abs_tol=1e-8)
    assert math.isclose(getattr(parsed, 'pitch', 0.), -.023617731, abs_tol=1e-8)


def test_actual_pose_writer_retains_height_and_tilt(monkeypatch):
    h = fixture.harness()
    del h._message  # Use production serializer, not the fixture transport stub.
    monkeypatch.setitem(fixture.namespace, 'PoseWithCovarianceStamped', pose_message)
    value = NS(x=1., y=2., z=.434135, yaw=.3, roll=.02, pitch=-.03)
    message = h._message(value, NS(sec=100, nanosec=0))
    assert message.pose.pose.position.z == .434135
    expected = quaternion(.02, -.03, .3)
    actual = message.pose.pose.orientation
    assert all(math.isclose(getattr(actual, k), getattr(expected, k), abs_tol=1e-10)
               for k in ('x', 'y', 'z', 'w'))


def test_rigid_composition_and_inverse_roundtrip_for_slopes_and_stairs():
    randomizer = random.Random(20261029)
    for _ in range(300):
        anchor = Pose3(*(randomizer.uniform(-3., 3.) for _ in range(3)),
                       .08, -.06, randomizer.uniform(-math.pi, math.pi))
        odom = Pose3(1.4, -.7, randomizer.uniform(-.2, .8), .03, .02, -.4)
        mapped = compose(anchor, odom)
        recovered = compose(inverse(anchor), mapped)
        assert all(math.isclose(getattr(recovered, k), getattr(odom, k), abs_tol=1e-10)
                   for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'))
        assert all(math.isclose(getattr(between(odom, mapped), k), getattr(anchor, k), abs_tol=1e-10)
                   for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'))
    assert type(compose(Pose2(1, 2, .3), Pose2(0, 0, 0))) is Pose2


def test_height_reference_follows_measured_stair_motion_not_constant_zero():
    h = GlobalPoseAuthority()
    odom = Pose3(0., 0., .1, 0., 0., 0.)
    initial = Pose3(1., 2., .3, 0., 0., .1)
    assert h.accept(initial, odom, stamp=99.9, now=100., rmse=.158,
                    overlap=.906, ambiguous=False, initial=True)
    for step in range(1, 8):
        current = Pose3(step*.1, 0., .1+step*.15, 0., 0., .01)
        expected = compose(h.transform, current)
        stamp = 100.+step*.2
        assert h.accept(expected, current, stamp=stamp, now=stamp+.1,
                        rmse=.158, overlap=.906, ambiguous=False)
        assert compose(h.transform, current).z > .3
    # An unexplained vertical correction is still rejected, with anchor intact.
    before = h.transform
    expected = compose(before, current)
    jump = Pose3(expected.x, expected.y, expected.z+1., expected.roll,
                 expected.pitch, expected.yaw)
    assert not h.accept(jump, current, stamp=102., now=102.1, rmse=.158,
                        overlap=.906, ambiguous=False)
    assert h.transform == before and h.hard_failure


def test_initial_rigid_attitude_and_nonfinite_coordinates_still_require_validation():
    for mapped in (Pose3(0., 0., .4, .2, 0., 0.),
                   Pose3(0., 0., math.nan, 0., 0., 0.)):
        h = GlobalPoseAuthority()
        assert not h.accept(mapped, Pose3(0., 0., 0., 0., 0., 0.),
            stamp=99.9, now=100., rmse=.158, overlap=.906,
            ambiguous=False, initial=True)
        assert h.transform is None


def test_tf_and_navigation_pose_represent_the_same_rigid_anchor():
    h = fixture.harness()
    h._sync_tf_parameters = lambda _: None
    h.actual_tf = False
    odom = Pose3(2., 1., .2, .01, .02, .3)
    mapped = Pose3(3., 2., .434135, -.025442109, -.023617731, .31)
    assert h.policy.accept(mapped, odom, stamp=99.9, now=100., rmse=.158,
                           overlap=.906, ambiguous=False, initial=True)
    h._odom = lambda stamp=None: (odom, NS(sec=100, nanosec=0))
    h._tick()
    tf = h.broadcaster.sendTransform.call_args.args[0].transform
    actual = Pose3.from_quaternion(tf.translation.x, tf.translation.y, tf.translation.z,
        tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)
    predicted = compose(actual, odom)
    assert all(math.isclose(getattr(predicted, k), getattr(mapped, k), abs_tol=1e-9)
               for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'))
    assert h.pose_pub.publish.call_args.args[0].mapped == compose(h.policy.transform, odom)


def test_native_initial_result_to_next_tracking_request_preserves_measured_pose(monkeypatch):
    from test_global_relocalization_coordinator import harness as coordinator_harness
    coordinator = coordinator_harness()
    message = pose_message()
    message.header.stamp = NS(sec=99, nanosec=900_000_000)
    message.pose.pose.position = NS(x=1., y=2., z=.434135)
    message.pose.pose.orientation = quaternion(-.025442109, -.023617731, .1)
    p = coordinator._pose_dictionary(message)
    h = fixture.harness()
    h.initial_request_id = 'initial'
    h._seed(NS(data=json.dumps(dict(stage='validation_passed', state='refined',
        source_pose='refined', candidate_verified=True, request_id='initial',
        map_name='map', map_load_id='load', body_generation=1,
        pose_stamp=dict(sec=99, nanosec=900_000_000), pose=p,
        metrics=dict(fitness_m=.158, overlap=.906, transform_model=TRANSFORM_MODEL)))))
    assert h.seed_request == 'initial'
    assert math.isclose(h.policy.transform.z, .434135)
    h._seed(NS(data=json.dumps(dict(state='refined', stage='complete', terminal=True))))
    del h._match
    del h._message
    monkeypatch.setitem(fixture.namespace, 'PoseWithCovarianceStamped', pose_message)
    monkeypatch.setitem(fixture.namespace, 'Relocalize', NS(Goal=NS))
    h.action, h.next_match = Mock(), 0.
    h.action.server_is_ready.return_value = True
    h.action.send_goal_async.return_value = Future()
    h._match(Pose3(.1, 0., .15, 0., 0., 0.), NS(sec=100, nanosec=0), 100.)
    rough = h.action.send_goal_async.call_args.args[0].rough_pose.pose.pose
    predicted = compose(h.policy.transform, Pose3(.1, 0., .15, 0., 0., 0.))
    assert math.isclose(rough.position.z, predicted.z)
    assert rough.position.z != 0.


def test_old_flattened_native_result_cannot_silently_initialize_new_model():
    from test_global_relocalization_coordinator import harness as coordinator_harness
    for model in ('', 'legacy_planar_v1'):
        coordinator = coordinator_harness()
        coordinator._process_action_result(NS(success=True, code=0, candidate_verified=True,
            fitness=.158, overlap=.906, correction_xy=.03, correction_yaw=.01,
            refined_pose=NS(), transform_model=model), 4)
        assert coordinator._await_authority is None
        assert '版本不匹配' in coordinator._publish_status.call_args.args[3]
    coordinator = coordinator_harness(strict=False)
    coordinator._process_action_result(NS(success=True, code=0, candidate_verified=False,
        fitness=.158, overlap=.906, correction_xy=.03, correction_yaw=.01,
        refined_pose=NS(), transform_model='legacy_planar_v1'), 4)
    coordinator._begin_amcl_confirmation.assert_called_once()


def test_rigid_anchor_survives_recovery_through_shared_route_and_velocity_chain(monkeypatch):
    from test_match_recovery_flow import setup, match
    h, velocity, route, tick = setup()
    h.policy.reset()
    assert h.policy.accept(Pose3(1., 2., .434135, .02, -.01, .1), Pose2(0, 0, 0),
        stamp=97.05, now=97.1, rmse=.158, overlap=.906, ambiguous=False, initial=True)
    del h._message
    monkeypatch.setitem(fixture.namespace, 'PoseWithCovarianceStamped', pose_message)
    assert tick(100.)['ready']
    assert tick(100.2)['recovering']
    assert velocity.last_velocity() == (0., 0.)
    tick(100.4)
    match(h, 100.3, Pose3(1.01, 2., .44, .02, -.01, .1))
    assert tick(100.6, command=False)['ready']
    match(h, 100.5, Pose3(1.02, 2., .445, .02, -.01, .1))
    assert tick(100.7, command=False)['ready']
    assert math.isclose(h.policy.transform.z, .445)
    route._request_stop.assert_not_called()
    assert velocity.last_velocity() == (0., 0.)
    tick(100.8, command=True)
    assert velocity.last_velocity()[0] > 0.
