"""Recoverable LIO pauses must never become hardware emergency commands."""
import ast
from pathlib import Path
import unittest

import test_robot_body_selection as body_tests
import test_velocity_arbiter_callbacks as arbiter_tests

Message = arbiter_tests.Message


ROOT = Path(__file__).resolve().parents[1]
LIO = ROOT.parent / 'lio/small_point_lio'
MANAGER = ROOT.parent / 'nav_service/lzp_service/lzp_service/route_navigation_manager.py'


def cpp_method(text, name, next_name):
    return text.split('SmallPointLioNode::' + name, 1)[1].split(
        'SmallPointLioNode::' + next_name, 1)[0]


class LioMotionPauseTests(unittest.TestCase):
    def setUp(self):
        self.fixture = arbiter_tests.VelocityArbiterCallbacksTest(methodName='runTest')
        self.fixture.setUp()
        self.node = self.fixture.node

    def state(self, paused):
        self.node._lio_pause_callback(Message(paused))

    def tick(self):
        self.node._tick()
        return self.fixture.last_velocity()

    def test_missing_lio_state_fails_closed_in_2d_and_3d(self):
        for mode in ('map_2d', 'terrain_3d'):
            self.fixture.mode(mode)
            self.node._lio_pause_at = None
            self.fixture.command()
            self.assertEqual(self.tick(), (0.0, 0.0))
            self.assertIn('LIO', self.node._guard_reason)

    def test_pause_resume_discards_old_commands_and_requires_fresh_input(self):
        self.state(False)
        self.fixture.command()
        self.assertEqual(self.tick(), (0.2, 0.1))
        self.state(True)
        self.assertEqual(self.fixture.last_velocity(), (0.0, 0.0))
        self.fixture.command()
        self.assertEqual(self.tick(), (0.0, 0.0))
        self.state(False)
        self.assertEqual(self.tick(), (0.0, 0.0))
        self.fixture.command()
        self.assertEqual(self.tick(), (0.2, 0.1))

    def test_healthy_heartbeat_does_not_interrupt_continuous_commands(self):
        self.state(False)
        for index in range(80):
            self.fixture.now += 0.05
            self.fixture.body('xingji')
            self.fixture.terrain(True)
            self.fixture.command()
            if index % 10 == 0:
                self.state(False)
                self.node._navigation_pause_callback(Message(False))
            self.assertEqual(self.tick(), (0.2, 0.1))

    def test_stale_or_invalid_state_stops_even_with_fresh_commands(self):
        self.fixture.mode('map_2d')
        self.state(False)
        self.fixture.now += 2.6
        self.fixture.command()
        self.assertEqual(self.tick(), (0.0, 0.0))
        for invalid in ('false', None, 0, 1):
            self.state(invalid)
            self.fixture.command()
            self.assertEqual(self.tick(), (0.0, 0.0))

    def test_real_emergency_is_not_cleared_by_lio_recovery(self):
        self.state(False)
        self.fixture.command()
        self.state(True)
        self.node._emergency_callback(Message())
        self.state(False)
        self.fixture.command()
        self.assertTrue(self.node._emergency)
        self.assertEqual(self.tick(), (0.0, 0.0))

    def test_navigation_hold_survives_native_recovery_until_manager_releases(self):
        self.state(False)
        self.node._navigation_pause_callback(Message(False))
        self.fixture.command()
        self.assertEqual(self.tick(), (0.2, 0.1))
        self.node._navigation_pause_callback(Message(True))
        self.state(False)
        self.fixture.command()
        self.assertEqual(self.tick(), (0.0, 0.0))
        self.node._navigation_pause_callback(Message(False))
        self.assertEqual(self.tick(), (0.0, 0.0))
        self.fixture.command()
        self.assertEqual(self.tick(), (0.2, 0.1))

    def test_missing_navigation_heartbeat_cannot_release_a_fresh_lio_state(self):
        self.fixture.mode('map_2d')
        self.fixture.now += 2.6
        self.state(False)
        self.fixture.command()
        self.assertEqual(self.tick(), (0.0, 0.0))
        self.assertIn('导航恢复', self.node._guard_reason)

    def test_xingji_keeps_support_without_emergency_or_reset_during_pause(self):
        body_fixture = body_tests.XingjiWorkerCallbacksTests(methodName='runTest')
        body_fixture.setUp()
        body_fixture.arm()
        self.fixture.mode('map_2d')
        self.state(False)

        def forward():
            self.node._tick()
            body_fixture.node.nav_velocity(
                self.node.publisher.publish.call_args.args[0])
            body_fixture.node._control_tick()

        self.fixture.command()
        forward()
        self.assertGreater(body_fixture.node.sent[-1][0], 0)
        self.state(True)
        self.fixture.command()
        forward()
        self.assertEqual(body_fixture.node.sent[-1], (0, 0, 0))
        self.state(False)
        forward()
        self.assertEqual(body_fixture.node.sent[-1], (0, 0, 0))
        self.fixture.command()
        forward()
        self.assertGreater(body_fixture.node.sent[-1][0], 0)
        self.assertNotIn(('command', 6), body_fixture.node.sent)
        self.assertNotIn(('command', 4), body_fixture.node.sent)


class LioPublisherWiringTests(unittest.TestCase):
    def test_recovery_and_confirmed_fault_use_transient_and_latched_zero_stops(self):
        text = (LIO / 'src/small_point_lio_node.cpp').read_text(encoding='utf-8')
        begin = cpp_method(text, 'begin_auto_recovery_pause(',
                           'advance_auto_recovery_validation(')
        resume = cpp_method(text, 'advance_auto_recovery_validation(',
                            'reset_estimator_for_recovery(')
        reset = cpp_method(text, 'reset_estimator_for_recovery(', 'latch_lio_fault(')
        fault = cpp_method(text, 'latch_lio_fault(', 'SmallPointLioNode(')
        self.assertNotIn('emergency_stop_publisher->publish', begin)
        self.assertNotIn('emergency_stop_publisher->publish', reset)
        self.assertNotIn('clear_emergency_stop_publisher->publish', resume)
        self.assertIn('recoverable_stop_publisher->publish', begin)
        self.assertIn('recoverable_resume_publisher->publish', resume)
        self.assertIn('motion_stop_publisher->publish', fault)
        self.assertNotIn('emergency_stop_publisher->publish', fault)
        self.assertIn('set_motion_paused(true)', begin)
        self.assertIn('set_motion_paused(false)', resume)
        self.assertIn('repeat_motion_pause_state()', text)

    def test_mapping_remaps_only_recoverable_events(self):
        mapping = (ROOT / 'launch/mapping.launch.py').read_text(encoding='utf-8')
        self.assertIn("'recoverable_stop_topic': lio_stop_topic", mapping)
        self.assertIn("'recoverable_resume_topic': lio_resume_topic", mapping)
        self.assertNotIn("'emergency_stop_topic': lio_stop_topic", mapping)
        self.assertNotIn("'clear_emergency_stop_topic': lio_resume_topic", mapping)

    def test_auto_recovery_never_clears_hardware_emergency(self):
        tree = ast.parse(MANAGER.read_text(encoding='utf-8'))
        method = next(item for item in ast.walk(tree) if (
            isinstance(item, ast.FunctionDef)
            and item.name == '_resume_after_auto_recovery'))
        text = ast.unparse(method)
        self.assertNotIn('_clear_emergency_stop_publisher.publish', text)
        self.assertIn('manual_reset_required', text)

    def test_navigation_timeout_cannot_send_hardware_emergency(self):
        tree = ast.parse(MANAGER.read_text(encoding='utf-8'))
        for name in ('_begin_lio_reset', '_fail_auto_recovery'):
            method = next(item for item in ast.walk(tree) if (
                isinstance(item, ast.FunctionDef) and item.name == name))
            text = ast.unparse(method)
            self.assertNotIn('_emergency_stop_publisher.publish', text)
            self.assertIn('_send_zero_velocity()', text)
            self.assertIn('_publish_all_status()', text)


if __name__ == '__main__':
    unittest.main()
