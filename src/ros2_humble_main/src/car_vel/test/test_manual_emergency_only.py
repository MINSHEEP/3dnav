"""Production stop boundaries with fake ROS/SDK transport, never real robots."""
import ast
import json
from pathlib import Path
import types
import unittest
from unittest.mock import Mock

import test_robot_body_selection as body_tests
import test_velocity_arbiter_callbacks as arbiter_tests

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'car_vel'
SERVICES = ROOT.parent / 'nav_service/lzp_service/lzp_service'


def xingji():
    fixture = body_tests.XingjiWorkerCallbacksTests('runTest')
    fixture.setUp()
    tree = ast.parse((SOURCE / 'dog_remote_bridge.py').read_text(encoding='utf-8'))
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'DogRemoteBridge')
    names = {'_emergency_stop', '_safety_stop', '_send_command', '_reset_robot', '_command_callback', '_send_zero_burst'}
    definitions = [item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == 'command_payload']
    definitions += [item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name in names]
    constants = {}
    for item in tree.body:
        if isinstance(item, ast.Assign):
            for target in item.targets:
                if isinstance(target, ast.Name) and (target.id.startswith(('CMD_', 'MSG_')) or target.id == 'ZERO_VELOCITY'):
                    try:
                        constants[target.id] = ast.literal_eval(item.value)
                    except ValueError:
                        pass
    namespace = dict(constants, json=json, time=types.SimpleNamespace(sleep=lambda _: None))
    exec(compile(ast.Module(body=definitions, type_ignores=[]), '<production native stop methods>', 'exec'), namespace)
    base = type(fixture.node).__bases__[0]
    for name in names & namespace.keys():
        setattr(base, name, namespace[name])
    fixture.payloads = []
    fixture.node._send_json = lambda payload: fixture.payloads.append(payload) or True
    # Preserve the original fixture's zero-output history while exercising the
    # actual zero datagrams as well, rather than replacing the SDK stop method.
    real_zero = fixture.node._send_zero_burst
    def zero():
        fixture.node.sent.append((0, 0, 0))
        return real_zero()
    fixture.node._send_zero_burst = zero
    fixture.arm()
    return fixture


class ManualEmergencyOnlyTests(unittest.TestCase):
    def test_xingji_legacy_emergency_topic_is_zero_latched_not_cmd6(self):
        f = xingji()
        f.node.subscriptions['/emergency_stop'](None)
        self.assertTrue(f.node.emergency_latched)
        self.assertNotIn(6, [p.get('cmd_code') for p in f.payloads])
        self.assertEqual(f.node.sent[-1], (0, 0, 0))
        self.assertFalse(f.node.nav_armed)

    def test_xingji_motion_stop_blocks_old_targets_until_manual_rearm(self):
        f = xingji()
        f.node.nav_velocity(f.command())
        f.node.subscriptions['/motion_stop'](None)
        f.node.nav_velocity(f.command())
        f.node._control_tick()
        self.assertTrue(f.node.emergency_latched)
        self.assertFalse(f.node.nav_armed)
        f.node.clear_latch()
        f.node._control_tick()
        self.assertFalse(f.node.emergency_latched)
        self.assertFalse(f.node.nav_armed)
        self.assertTrue(all('cmd_code' not in p for p in f.payloads), 'soft reset must not send posture-reset CMD4')

    def test_xingji_only_explicit_manual_command_keeps_cmd6(self):
        f = xingji()
        f.node._command_callback(types.SimpleNamespace(data='{"command":"emergency"}'))
        self.assertEqual([p['cmd_code'] for p in f.payloads if 'cmd_code' in p], [6])
        self.assertTrue(f.node.hardware_emergency_latched)
        f.node.subscriptions['/motion_stop'](None)
        self.assertTrue(f.node.hardware_emergency_latched, 'soft event cannot downgrade a manual emergency')
        f.node.clear_latch()
        self.assertEqual([p['cmd_code'] for p in f.payloads if 'cmd_code' in p], [6, 4])
        self.assertFalse(f.node.hardware_emergency_latched)

    def test_xingji_failed_zero_transmission_never_escalates_to_cmd6(self):
        f = xingji()
        failed = []
        f.node._send_json = lambda payload: failed.append(payload) or False
        f.node.subscriptions['/motion_stop'](None)
        self.assertEqual(len(failed), 3)
        self.assertTrue(all(payload == {'vel_move_frontback': 0.0,
            'vel_move_leftright': 0.0, 'vel_turn_leftright': 0.0} for payload in failed))
        self.assertTrue(f.node.emergency_latched)
        self.assertFalse(f.node.nav_armed)

    def test_xingji_soft_latch_refuses_standing_without_manual_reset(self):
        f = xingji()
        f.node.remote_active = True
        f.node.subscriptions['/motion_stop'](None)
        f.node._stand_robot = Mock()
        f.node._command_callback(types.SimpleNamespace(data='{"command":"stand"}'))
        f.node._stand_robot.assert_not_called()
        self.assertTrue(f.node.emergency_latched)

    def test_zsl_zero_exception_is_bounded_and_explicit_not_passive(self):
        sdk = body_tests.FakeSdk()
        adapter = body_tests.Zsl1Adapter(body_tests.sdk_profile(), sdk=sdk)
        def fail(*args):
            sdk.calls.append(('move', args))
            raise OSError('transport down')
        sdk.move = fail
        adapter.fail_safe('lost state')
        self.assertEqual([name for name, _ in sdk.calls], ['init', 'move'])
        self.assertFalse(adapter.zero_stop_command_succeeded)
        self.assertIn('无法确认实际停车', adapter.error)
        self.assertTrue(adapter.emergency)

    def test_zsl_legacy_stop_and_new_stop_never_passive(self):
        for topic in ('/emergency_stop', '/motion_stop'):
            with self.subTest(topic=topic):
                f = body_tests.Zsl1RosCallbacksTests('runTest')
                f.setUp()
                f.warm()
                f.sdk.calls.clear()
                f.subscriptions[topic](None)
                self.assertEqual(f.sdk.calls, [('move', (0.0, 0.0, 0.0))])
                self.assertTrue(f.node.adapter.emergency)
                self.assertFalse(f.node.motion_authorized)
                self.assertFalse(f.node.adapter.hardware_emergency_latched)
                f.node.nav_velocity(f.twist())
                f.node.tick()
                self.assertEqual(len(f.sdk.calls), 1)

    def test_zsl_sdk_failure_attempts_zero_once_without_passive_or_recursion(self):
        sdk = body_tests.FakeSdk()
        adapter = body_tests.Zsl1Adapter(body_tests.sdk_profile(), sdk=sdk)
        sdk.fail_move = True
        adapter.set_velocity((0.3, 0, 0.2))
        with self.assertRaises(RuntimeError):
            for _ in range(20):
                adapter.tick(.05, True)
        self.assertEqual([name for name, _ in sdk.calls], ['init', 'move', 'move'])
        self.assertEqual(sdk.calls[-1], ('move', (0.0, 0.0, 0.0)))
        self.assertTrue(adapter.emergency)
        self.assertFalse(adapter.hardware_emergency_latched)
        self.assertIn('零速', adapter.error)
        for _ in range(20):
            adapter.tick(.05, False)
        self.assertEqual(len(sdk.calls), 3)

    def test_zsl_manual_emergency_still_passive_and_soft_event_cannot_clear_it(self):
        adapter = body_tests.Zsl1Adapter(body_tests.sdk_profile(), sdk=body_tests.FakeSdk())
        adapter.emergency_stop()
        self.assertEqual(adapter.sdk.calls[-1][0], 'passive')
        adapter.soft_stop()
        self.assertTrue(adapter.hardware_emergency_latched)
        adapter.reset()
        self.assertFalse(adapter.hardware_emergency_latched)
        self.assertFalse(any(name == 'standUp' for name, _ in adapter.sdk.calls))

    def test_zsl_callback_fault_is_soft_and_latched(self):
        f = body_tests.Zsl1RosCallbacksTests('runTest')
        f.setUp()
        f.node._fault(RuntimeError('read feedback failed'))
        self.assertTrue(f.node.adapter.emergency)
        self.assertEqual(f.sdk.calls[-1], ('move', (0.0, 0.0, 0.0)))
        self.assertFalse(any(name == 'passive' for name, _ in f.sdk.calls))
        f.node._stop_publisher.publish.assert_called_once()
        calls = len(f.sdk.calls)
        f.subscriptions['/motion_stop'](None)  # Echo of the worker's own notice.
        self.assertEqual(len(f.sdk.calls), calls, 'fault notification must not retry a rejected zero request')

    def test_arbiter_manual_stop_also_clears_all_old_velocity_sources(self):
        f = arbiter_tests.VelocityArbiterCallbacksTest('runTest')
        f.setUp()
        f.command()
        f.node._manual_command_callback(arbiter_tests.Message('{"command":"emergency"}'))
        f.node._tick()
        self.assertEqual(f.last_velocity(), (0.0, 0.0))
        f.node._clear_callback(None)
        f.node._tick()
        self.assertEqual(f.last_velocity(), (0.0, 0.0))
        f.command()
        f.node._tick()
        self.assertEqual(f.last_velocity(), (.2, .1))

    def test_manual_r_reset_clears_arbiter_latch_without_replaying_old_commands(self):
        f = arbiter_tests.VelocityArbiterCallbacksTest('runTest')
        f.setUp()
        f.command()
        f.node._emergency_callback(None)
        f.node._manual_command_callback(arbiter_tests.Message('{"command":"reset"}'))
        f.node._tick()
        self.assertEqual(f.last_velocity(), (0.0, 0.0))
        self.assertFalse(f.node._emergency)

    def test_campus_failure_uses_soft_topic_and_cancels_goal(self):
        tree = ast.parse((SERVICES / 'campus_mission_manager.py').read_text(encoding='utf-8'))
        method = next(item for item in ast.walk(tree) if isinstance(item, ast.FunctionDef) and item.name == '_fail')
        namespace = dict(Empty=object)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<production campus failure>', 'exec'), namespace)
        node = types.SimpleNamespace(_publish_zero=Mock(), _connector_active_publisher=Mock(),
            _cancel_navigation_publisher=Mock(), _stop_publisher=Mock(), _finish=Mock(), _current_mode='normal')
        namespace['Bool'] = lambda **kw: kw
        namespace['_unused'] = None
        namespace['_fail'](node, 'terrain stale', latch_stop=True)
        node._publish_zero.assert_called_once()
        node._cancel_navigation_publisher.publish.assert_called_once()
        node._stop_publisher.publish.assert_called_once()

    def test_lio_fault_and_campus_publish_motion_stop_not_hardware_emergency(self):
        cpp = (ROOT.parent / 'lio/small_point_lio/src/small_point_lio_node.cpp').read_text(encoding='utf-8')
        fault = cpp.split('SmallPointLioNode::latch_lio_fault(', 1)[1].split('SmallPointLioNode::SmallPointLioNode(', 1)[0]
        self.assertIn('motion_stop_publisher->publish', fault)
        self.assertNotIn('emergency_stop_publisher->publish', fault)
        campus = (SERVICES / 'campus_mission_manager.py').read_text(encoding='utf-8')
        self.assertIn("Empty, '/motion_stop'", campus)
        self.assertNotIn("Empty, '/emergency_stop'", campus)


if __name__ == '__main__':
    unittest.main()
