"""Real autonomous traversal -> native model commands, no robot/ROS I/O."""
import ast
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT.parent / 'nav_service/lzp_service')]
from lzp_service.automatic_terrain_navigation import AutomaticTerrainController
from lzp_service.terrain_navigation import TerrainObservation


def bridge(*, model=1, feedback='ok', send_ok=True):
    source = ROOT / 'car_vel/dog_remote_bridge.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'DogRemoteBridge')
    selected = {'_set_traversal_mode_callback', '_configure_navigation_safety_mode',
                '_navigation_gait_confirmed', '_toggle_model', '_model_label', '_normalise_failures',
                '_ensure_standing_callback', '_find_status_object', '_handle_packet'}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in selected]
    clock = NS(now=100.)
    def sleep(seconds):
        clock.now += seconds
    scope = dict(threading=threading, time=NS(monotonic=lambda: clock.now, sleep=sleep),
                 json=json, MODE_STANDING=2)
    for item in tree.body:
        if isinstance(item, ast.Assign) and len(item.targets) == 1 and isinstance(item.targets[0], ast.Name):
            try:
                scope[item.targets[0].id] = ast.literal_eval(item.value)
            except (ValueError, TypeError):
                pass
    program = ast.Module(body=[ast.ClassDef(name='Bridge', bases=[], keywords=[],
        body=methods, decorator_list=[])], type_ignores=[])
    exec(compile(ast.fix_missing_locations(program), str(source), 'exec'), scope)
    node = scope['Bridge']()
    node.clock = clock
    node.state_lock = threading.RLock()
    node.emergency_latched = False
    node.connection_ready = True
    node.last_status_time = node.last_model_status_time = clock.now
    node.status_timeout = 2.
    node.stand_confirmation_timeout = 2.
    node.navigation_gait_confirmation_timeout = .25
    node.robot_status = {'model': model, 'motion_mode': 2, 'hardware_error': 0}
    node.current_model = model
    node.high_speed_model, node.obstacle_model = 1, 0
    node.keyboard_high_speed_mode = model == 1
    node.remote_active = False
    node.keyboard_obstacle_open = node.obstacle_stop_enabled = True
    node.navigation_collision_configured = True
    node._ensure_standing = Mock(return_value=True)
    node._cancel_action = Mock()
    node._send_zero_burst = Mock()
    node._event = Mock()
    node._query_status = Mock()
    node.get_logger = lambda: Mock()
    node.parsed_packets = node.non_status_packets = 0
    node.commands = []
    def send(code, value):
        node.commands.append((code, value))
        if code == 2 and send_ok and feedback == 'ok':
            clock.now += .01
            node.robot_status['model'] = value
            node.current_model = value
            node.last_status_time = node.last_model_status_time = clock.now
        return send_ok
    node._send_command = send
    return node


class XingjiNavigationGaitTests(unittest.TestCase):
    def test_flat_recovery_never_sends_native_high_speed(self):
        controller = AutomaticTerrainController()
        controller.confirm_mode('obstacle', success=True)
        for sequence in range(1, 6):
            decision = controller.update(TerrainObservation(), observation_fresh=True,
                navigation_active=True, campus_active=False, observation_sequence=sequence)
        node = bridge()
        response = node._set_traversal_mode_callback(NS(mode=decision.requested_mode), NS())
        self.assertTrue(response.success)
        self.assertNotIn((2, 1), node.commands, 'flat -> normal must not mean native high-speed')
        self.assertIn((2, 0), node.commands)

    def test_all_autonomous_aliases_keep_obstacle_and_collision_stop(self):
        for mode in ('normal', 'ramp', 'obstacle', 'rough_ground', 'stairs_up', 'stairs_down'):
            with self.subTest(mode=mode):
                node = bridge()
                response = node._set_traversal_mode_callback(NS(mode=mode), NS())
                self.assertTrue(response.success)
                self.assertNotIn((2, 1), node.commands)
                self.assertIn((2, 0), node.commands)
                self.assertIn((5, 1), node.commands)
                self.assertFalse(node.keyboard_high_speed_mode)

    def test_confirmed_obstacle_requests_are_idempotent_without_zero_bursts(self):
        node = bridge(model=0)
        for mode in ('normal', 'obstacle', 'rough_ground', 'normal'):
            self.assertTrue(node._set_traversal_mode_callback(NS(mode=mode), NS()).success)
        self.assertEqual(node.commands, [])
        node._send_zero_burst.assert_not_called()
        node._cancel_action.assert_not_called()

    def test_missing_feedback_or_failed_send_never_reports_ready(self):
        for feedback, sent in (('missing', True), ('ok', False)):
            with self.subTest(feedback=feedback, sent=sent):
                node = bridge(feedback=feedback, send_ok=sent)
                response = node._set_traversal_mode_callback(NS(mode='normal'), NS())
                self.assertFalse(response.success)
                self.assertNotIn((2, 1), node.commands)

    def test_page_initialization_reports_failed_confirmation(self):
        node = bridge(feedback='missing')
        response = node._ensure_standing_callback(NS(), NS())
        self.assertFalse(response.success)

    def test_native_gait_send_failure_does_not_use_old_feedback(self):
        node = bridge(model=0)
        node.navigation_collision_configured = False
        sent = node._send_command
        def reject_model(code, value):
            if code == 2:
                node.commands.append((code, value))
                return False
            return sent(code, value)
        node._send_command = reject_model
        self.assertFalse(node._ensure_standing_callback(NS(), NS()).success)
        self.assertFalse(node._navigation_gait_confirmed())
        self.assertIn((5, 1), node.commands)
        self.assertIn((2, 0), node.commands)

    def test_manual_toggle_invalidates_navigation_confirmation(self):
        node = bridge(model=0)
        self.assertTrue(node._navigation_gait_confirmed())
        node._toggle_model()
        self.assertFalse(node._navigation_gait_confirmed())
        self.assertTrue(node._set_traversal_mode_callback(NS(mode='normal'), NS()).success)
        self.assertEqual(node.commands[-1], (2, 0))

    def test_manual_remote_toggle_is_not_pinned(self):
        node = bridge(model=0)
        node.remote_active = True
        node._toggle_model()
        self.assertIn((2, 1), node.commands)

    def test_boot_and_disabled_collision_stop_are_explicitly_configured(self):
        for attribute in ('navigation_collision_configured', 'obstacle_stop_enabled'):
            node = bridge(model=0)
            setattr(node, attribute, False)
            self.assertTrue(node._set_traversal_mode_callback(NS(mode='normal'), NS()).success)
            self.assertIn((5, 1), node.commands)
            self.assertIn((2, 0), node.commands)

    def test_old_model_feedback_is_not_refreshed_by_unrelated_status(self):
        node = bridge(model=0, feedback='missing')
        node.clock.now += 2.1
        node._handle_packet({'motion_mode': 2, 'battery': 90})
        self.assertFalse(node._navigation_gait_confirmed())
        self.assertFalse(node._set_traversal_mode_callback(NS(mode='normal'), NS()).success)
        node._handle_packet({'model': 0, 'motion_mode': 2})
        self.assertEqual(node.last_model_status_time, node.clock.now)
        # A delayed packet after a timed-out request does not reauthorize motion.
        self.assertFalse(node._navigation_gait_confirmed())
        def reply():
            node.clock.now += .01
            node._handle_packet({'model': 0, 'motion_mode': 2})
        node._query_status.side_effect = reply
        self.assertTrue(node._set_traversal_mode_callback(NS(mode='normal'), NS()).success)
        self.assertTrue(node._navigation_gait_confirmed())

    def test_invalid_or_faulted_feedback_never_confirms(self):
        for update in ({'model': 1}, {'model': None}, {'motion_mode': 1},
                       {'hardware_error': 1}, {'hw_failure': [3]}):
            node = bridge(model=0)
            node.robot_status.update(update)
            self.assertFalse(node._navigation_gait_confirmed())

    def test_campus_success_cancel_failure_restore_never_selects_high_speed(self):
        path = ROOT.parent / 'nav_service/lzp_service/lzp_service/campus_mission_manager.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        definition = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                          and n.name == 'CampusMissionManager')
        methods = [n for n in definition.body if isinstance(n, ast.FunctionDef)
                   and n.name in ('_finish', '_fail')]
        scope = {'SetTraversalMode': NS(Request=lambda: NS()), 'Bool': lambda **kw: NS(**kw),
                 'Empty': lambda: NS()}
        program = ast.Module(body=[ast.ClassDef(name='Campus', bases=[], keywords=[],
            body=methods, decorator_list=[])], type_ignores=[])
        exec(compile(ast.fix_missing_locations(program), str(path), 'exec'), scope)
        for terminal in ('completed', 'cancelled', 'failed'):
            with self.subTest(terminal=terminal):
                dog = bridge()
                campus = scope['Campus']()
                campus._current_mode = 'obstacle'
                campus._mode_client = Mock()
                campus._mode_client.call_async.side_effect = lambda req: dog._set_traversal_mode_callback(req, NS())
                for name in ('_publish_zero', '_connector_active_publisher', '_active_publisher',
                             '_cancel_navigation_publisher', '_stop_publisher', '_publish_status'):
                    setattr(campus, name, Mock())
                if terminal == 'failed':
                    campus._fail('test fault', latch_stop=True)
                else:
                    campus._finish(terminal, 'test terminal')
                self.assertNotIn((2, 1), dog.commands)
                self.assertIn((2, 0), dog.commands)


class XingjiNavigationVelocityTests(unittest.TestCase):
    def setUp(self):
        from test_robot_body_selection import XingjiWorkerCallbacksTests
        self.fixture = XingjiWorkerCallbacksTests(methodName='runTest')
        self.fixture.setUp()
        self.node = self.fixture.node
        # Use the real feedback predicate inside the real worker timer.
        reference = bridge(model=0)
        self.node.connection_ready = True
        self.node.last_model_status_time = 100.
        self.node.obstacle_model = 0
        self.node.obstacle_stop_enabled = True
        self.node.navigation_collision_configured = True
        self.node._normalise_failures = reference._normalise_failures
        self.node._navigation_gait_confirmed = lambda: type(reference)._navigation_gait_confirmed(self.node)
        self.fixture.arm()

    def tick(self):
        self.node.nav_velocity(self.fixture.command())
        self.node._control_tick()

    def test_each_autonomous_page_uses_same_guard(self):
        # Both Nav2 routes and campus connectors converge on /cmd_vel_safe.
        for source in ('terrain_navigation', 'campus_navigation', 'inspection'):
            with self.subTest(source=source):
                self.fixture.arm()
                self.node.robot_status['model'] = 0
                self.tick()
                self.assertGreater(self.node.sent[-1][0], 0)
                self.node.robot_status['model'] = 1
                self.tick()
                self.assertEqual(self.node.sent[-1], (0, 0, 0))
                self.assertFalse(self.node.nav_armed)
                # Restored feedback does not resurrect an old movement command.
                self.node.robot_status['model'] = 0
                self.tick()
                self.assertEqual(self.node.sent[-1], (0, 0, 0))
        self.assertNotIn(('command', 6), self.node.sent)
        self.assertNotIn(('command', 4), self.node.sent)

    def test_high_speed_manual_remote_and_mapping_are_unchanged(self):
        self.node.robot_status['model'] = 1
        self.node.remote_active = True
        self.tick()
        self.assertEqual(self.node.sent[-1], 'remote')
        self.node.remote_active = False
        self.fixture.arm_mapping()
        self.tick()
        self.assertGreater(self.node.sent[-1][0], 0)

    def test_stale_explicit_model_stops_even_with_other_fresh_status(self):
        self.tick()
        self.node.last_model_status_time = 95.
        self.tick()
        self.assertFalse(self.node.nav_armed)
        self.assertEqual(self.node.sent[-1], (0, 0, 0))


if __name__ == '__main__':
    unittest.main()
