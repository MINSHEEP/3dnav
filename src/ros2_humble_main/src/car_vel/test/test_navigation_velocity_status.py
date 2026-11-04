"""The UI must distinguish command timeout, terrain stop and allowed ceiling."""
import json
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

class NavigationVelocityStatusTests(unittest.TestCase):
    def setUp(self):
        from test_velocity_arbiter_callbacks import VelocityArbiterCallbacksTest
        self.fixture = VelocityArbiterCallbacksTest(methodName='runTest')
        self.fixture.setUp()
        self.node = self.fixture.node
        self.fixture.body('xingji')
        self.node._navigation_active_callback(NS(data=True))
        self.terrain()

    def terrain(self, **changes):
        value = dict(safe=True, speed_limit_mps=.25, navigation_mode='terrain_3d',
            navigation_active=True, evidence_source='current_anchored', state='ready',
            reason='保持越障步态，速度由Nav2控制', observed_terrain_type='curb',
            speed_policy='nav2_body_limits', terrain_speed_limit_enabled=False,
            speed_limit_source='nav2_body_limits', stamp_ns=int(self.fixture.now*1e9))
        value.update(changes)
        self.node._terrain_status_callback(NS(data=json.dumps(value)))

    def guard(self):
        return json.loads(self.node._guard_publisher.publish.call_args.args[0].data)

    def test_requested_and_dispatched_commands_are_separate_from_ceiling(self):
        self.fixture.command()
        self.node._tick()
        status = self.guard()
        self.assertEqual(status['schema_version'], 2)
        self.assertEqual(status['request_velocity'], [.2, 0., .1])
        self.assertEqual(status['commanded_velocity'], [.2, 0., .1])
        self.assertEqual(status['speed_limit_mps'], .25)
        self.assertEqual(status['speed_policy'], 'nav2_body_limits')
        self.assertEqual(status['terrain_type'], 'curb')
        self.assertFalse(status['blocked'])
        self.assertEqual(self.fixture.last_velocity(), (.2, .1))

    def test_expired_controller_command_is_reported_and_not_replayed(self):
        self.fixture.command()
        self.node._tick()
        self.fixture.now += .35
        self.node._lio_pause_callback(NS(data=False))
        self.node._navigation_pause_callback(NS(data=False))
        self.terrain()
        self.node._tick()
        status = self.guard()
        self.assertEqual(self.fixture.last_velocity(), (0., 0.))
        self.assertTrue(status['blocked'])
        self.assertEqual(status['reason_code'], 'upstream_stale')
        self.assertFalse(status['input_fresh'])
        self.assertIsNone(status['request_velocity'])
        self.assertEqual(status['commanded_velocity'], [0., 0., 0.])
        self.assertIn('过期', status['reason'])

    def test_terrain_hazard_is_distinguished_from_upstream_timeout(self):
        self.terrain(safe=False, speed_limit_mps=0., reason='检测到明确向下断层')
        self.fixture.command()
        self.node._tick()
        status = self.guard()
        self.assertEqual(self.fixture.last_velocity(), (0., 0.))
        self.assertTrue(status['blocked'])
        self.assertEqual(status['reason_code'], 'terrain_unsafe')
        self.assertIn('向下断层', status['reason'])
        self.assertEqual(status['request_velocity'], [.2, 0., .1])

    def test_upstream_zero_is_not_falsely_shown_as_a_protection_failure(self):
        self.node._nav_callback(self.fixture.module.Twist())
        self.node._tick()
        status = self.guard()
        self.assertFalse(status['blocked'])
        self.assertEqual(status['reason_code'], 'upstream_zero')
        self.assertEqual(status['request_velocity'], [0., 0., 0.])

    def test_raw_controller_timing_is_passive_and_reports_real_receive_rate(self):
        self.fixture.command()
        message = self.fixture.module.Twist()
        message.linear.x = .25
        before = self.node.publisher.publish.call_count
        self.node._controller_diagnostic_callback(message)
        self.fixture.now += .2
        self.node._controller_diagnostic_callback(message)
        self.assertEqual(self.node.publisher.publish.call_count, before)
        self.fixture.command()
        self.node._tick()
        status = self.guard()
        self.assertEqual(status['controller_stream']['topic'], '/cmd_vel_nav')
        self.assertAlmostEqual(status['controller_stream']['receive_rate_hz'], 5.)
        self.assertEqual(self.fixture.last_velocity(), (.2, .1))

    def test_raw_and_smoothed_stream_interruptions_are_distinguished(self):
        message = self.fixture.module.Twist()
        message.linear.x = .25
        self.node._controller_diagnostic_callback(message)
        self.fixture.command()
        self.fixture.now += .35
        self.node._controller_diagnostic_callback(message)  # Raw source still arrives.
        self.node._lio_pause_callback(NS(data=False))
        self.node._navigation_pause_callback(NS(data=False))
        self.terrain()
        self.node._tick()
        self.assertEqual(self.guard()['stream_reason_code'], 'smoother_output_stale')
        self.assertEqual(self.fixture.last_velocity(), (0., 0.))
        self.fixture.now += .55
        self.node._lio_pause_callback(NS(data=False))
        self.node._navigation_pause_callback(NS(data=False))
        self.terrain()
        self.node._nav_callback(self.fixture.module.Twist())  # Fresh zero from smoother.
        self.node._tick()
        status = self.guard()
        self.assertEqual(status['stream_reason_code'], 'controller_stream_stale')
        self.assertIsNone(status['controller_stream']['request_velocity'])
        self.assertEqual(self.fixture.last_velocity(), (0., 0.))


if __name__ == '__main__':
    unittest.main()
