"""Offline behavior tests: real configuration/supervisor/SDK adapter, fake I/O."""
import ast
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import Mock, patch
from concurrent.futures import Future as ConcurrentFuture

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / 'nav_service' / 'lzp_service'))
from car_vel.zsl1_adapter import Zsl1Adapter
from car_vel.velocity_shaping import TurnCommandShaper
from car_vel.motion_diagnostics import CommandStreamTiming, MotionTrace, age, velocity, body_reason
from lzp_service.robot_body import BodyConfigStore, BodySupervisor, default_config, validate_profile


class FakeSdk:
    def __init__(self):
        self.calls = []
        self.connected = True
        self.mode = 51
        self.velocity = [0, 0, 0]
        self.gyro = [0, 0, 0]
        self.fail_move = False
    def initRobot(self, *args): self.calls.append(('init', args))
    def checkConnect(self): return self.connected
    def getCurrentCtrlmode(self): return self.mode
    def getBodyVelocity(self): return self.velocity
    def getBodyGyro(self): return self.gyro
    def getBatteryPower(self): return 85
    def move(self, *args):
        self.calls.append(('move', args))
        return 0x3013 if self.fail_move else 0
    def passive(self): self.calls.append(('passive', ())); self.mode = 0; return 0
    def standUp(self): self.calls.append(('standUp', ())); self.mode = 1; return 0
    def lieDown(self): self.calls.append(('lieDown', ())); self.mode = 51; return 0


def sdk_profile():
    profile = default_config()['profiles']['gangben_zsl1']
    profile['local_ip'] = '192.168.168.15'
    return profile


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.sdk = FakeSdk()
        self.adapter = Zsl1Adapter(sdk_profile(), sdk=self.sdk)
    def test_init_does_not_stand_or_move(self):
        self.assertEqual([name for name, _ in self.sdk.calls], ['init'])
    def test_physical_units_no_xingji_scale(self):
        self.adapter.set_velocity((0.2, 0, 0.1))
        for _ in range(30): self.adapter.tick(0.05, True)
        self.assertEqual(self.sdk.calls[-1], ('move', (0.2, 0.0, 0.1)))
    def test_limits_and_slew(self):
        self.adapter.set_velocity((99, -99, 99))
        for _ in range(60): self.adapter.tick(0.05, True)
        self.assertEqual(self.sdk.calls[-1], ('move', (0.5, -0.2, 0.6)))
        self.assertTrue(all(abs(args[0]) <= 0.5 for name, args in self.sdk.calls if name == 'move'))
    def test_exact_zero_is_immediate_not_passive(self):
        self.adapter.set_velocity((0.2, 0, 0.2))
        for _ in range(20): self.adapter.tick(0.05, True)
        self.adapter.set_velocity((0, 0, 0)); self.adapter.tick(0.05, True)
        self.assertEqual(self.sdk.calls[-1], ('move', (0, 0, 0)))
        self.assertFalse(any(name == 'passive' for name, _ in self.sdk.calls))
    def test_deadband_does_not_raise_planner_target(self):
        self.adapter.set_velocity((0.01, 0.05, 0.01))
        for _ in range(50): self.adapter.tick(0.05, True)
        self.assertEqual(len(self.sdk.calls), 1)
    def test_timeout_stop_and_no_old_target(self):
        self.adapter.set_velocity((0.3, 0, 0.2))
        for _ in range(30): self.adapter.tick(0.05, True)
        self.adapter.tick(0.05, False)
        self.assertEqual(self.adapter.target, (0, 0, 0))
        self.adapter.tick(0.05, True)
        self.assertEqual(self.sdk.calls[-1], ('move', (0, 0, 0)))
    def test_nan_inf_stop(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            self.adapter.set_velocity((value, 0.2, 0.2))
            self.assertEqual(self.adapter.target, (0, 0, 0))
    def test_emergency_latches_and_reset_never_stands(self):
        self.adapter.emergency_stop()
        self.adapter.set_velocity((1, 1, 1))
        self.assertEqual(self.adapter.target, (0, 0, 0))
        self.adapter.reset()
        self.assertFalse(self.adapter.emergency)
        self.assertFalse(any(name == 'standUp' for name, _ in self.sdk.calls))
    def test_sdk_failure_attempts_stop_and_latches(self):
        self.sdk.fail_move = True
        self.adapter.set_velocity((0.3, 0, 0.2))
        with self.assertRaises(RuntimeError):
            for _ in range(10): self.adapter.tick(0.05, True)
        self.assertTrue(self.adapter.emergency)
        self.assertEqual(self.sdk.calls[-1], ('move', (0.0, 0.0, 0.0)))
        self.assertNotIn(('passive', ()), self.sdk.calls)
        self.assertEqual(self.adapter.target, (0, 0, 0))
    def test_parked_requires_finite_stationary_feedback(self):
        self.assertTrue(self.adapter.state()['parked'])
        self.sdk.velocity = [float('nan'), 0, 0]
        self.assertFalse(self.adapter.state()['parked'])
        self.sdk.velocity = [0.2, 0, 0]
        self.assertFalse(self.adapter.state()['parked'])
    def test_cached_action_state_does_not_count_as_parked(self):
        self.sdk.mode = 1
        self.assertFalse(self.adapter.state()['parked'])
    def test_idle_does_not_enter_move_as_keepalive(self):
        for _ in range(80): self.adapter.tick(0.05, False)
        self.assertEqual([name for name, _ in self.sdk.calls], ['init'])


class FakeWorker:
    def __init__(self, owner, body, profile, generation, frozen):
        self.owner, self.body = owner, body
        self.running = True
        self.parked = True
        self.fail_ready = False
        owner.events.append(('start', body, frozen))
    def alive(self): return self.running
    def poll(self): return []
    def request(self, operation, timeout):
        self.owner.events.append((operation, self.body))
        if operation == 'prepare' and not self.parked:
            return {'success': False, 'message': 'not parked'}
        if operation == 'ready' and self.fail_ready:
            raise RuntimeError('SDK timeout')
        return {'success': True}
    def close(self): self.owner.events.append(('close', self.body)); self.running = False


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = BodyConfigStore(Path(self.temp.name) / 'body.json')
        self.events = []
        self.created = []
        self.fail_new = False
        def factory(*args):
            worker = FakeWorker(self, *args)
            worker.fail_ready = self.fail_new
            self.created.append(worker)
            return worker
        self.supervisor = BodySupervisor(factory, self.store)
        self.supervisor.start()
    def apply(self, **overrides):
        args = dict(body='gangben_zsl1', profile=sdk_profile(), expected_version=1, idle_check=lambda: None)
        args.update(overrides)
        self.supervisor.apply(**args)
    def test_order_freeze_exit_start_ready_save_activate(self):
        self.apply()
        self.assertEqual(self.events, [('start','xingji',False), ('prepare','xingji'),
            ('close','xingji'), ('start','gangben_zsl1',True), ('ready','gangben_zsl1'),
            ('activate','gangben_zsl1')])
        self.assertEqual(self.store.load()['selected'], 'gangben_zsl1')
        self.assertEqual(self.store.load()['version'], 2)
    def test_not_parked_does_not_stop_old_or_poison_its_health(self):
        self.created[0].parked = False
        with self.assertRaises(ValueError): self.apply()
        self.assertTrue(self.created[0].alive())
        self.assertEqual(len(self.created), 1)
        self.assertEqual(self.supervisor.error, '')
    def test_busy_tasks_rejected_before_touching_worker(self):
        def busy(): raise ValueError('busy')
        with self.assertRaises(ValueError): self.apply(idle_check=busy)
        self.assertEqual(len(self.events), 1)
    def test_stale_browser_version_rejected(self):
        with self.assertRaises(ValueError): self.apply(expected_version=9)
        self.assertEqual(len(self.events), 1)
    def test_unknown_model_rejected(self):
        with self.assertRaises(ValueError): self.apply(body='gangben_zsl1w')
        self.assertEqual(len(self.events), 1)
    def test_unconfigured_local_ip_rejected(self):
        profile = sdk_profile(); profile['local_ip'] = ''
        with self.assertRaises(ValueError): self.apply(profile=profile)
        self.assertEqual(len(self.events), 1)
    def test_failed_new_connection_does_not_restore_or_persist(self):
        self.fail_new = True
        with self.assertRaises(RuntimeError): self.apply()
        self.assertFalse(any(worker.alive() for worker in self.created))
        self.assertEqual(self.store.load()['selected'], 'xingji')
        self.assertIsNone(self.supervisor.snapshot()['active_body'])
        self.assertEqual(len(self.created), 2)
    def test_failed_save_stops_candidate(self):
        with patch.object(self.store, 'save', side_effect=OSError('disk full')):
            with self.assertRaises(OSError): self.apply()
        self.assertFalse(any(worker.alive() for worker in self.created))
        self.assertEqual(self.supervisor.config['selected'], 'xingji')
    def test_corrupt_config_never_defaults_to_xingji(self):
        self.store.path.write_text('{bad', encoding='utf-8')
        factory = Mock()
        supervisor = BodySupervisor(factory, self.store)
        supervisor.start()
        factory.assert_not_called()
        self.assertIsNone(supervisor.config)
    def test_non_object_config_fails_closed(self):
        self.store.path.write_text('[]', encoding='utf-8')
        factory = Mock()
        supervisor = BodySupervisor(factory, self.store)
        supervisor.start()
        factory.assert_not_called()
        self.assertTrue(supervisor.error)
    def test_initial_native_hang_has_bounded_lifetime(self):
        self.supervisor.started_at = 0.0
        self.supervisor.clock = lambda: 13.0
        self.supervisor.poll()
        self.assertFalse(self.created[0].alive())
        self.assertIn('初始化超时', self.supervisor.error)
    def test_dead_worker_never_auto_respawns(self):
        self.created[0].running = False
        self.supervisor.poll(); self.supervisor.poll()
        self.assertEqual(len(self.created), 1)
        self.assertTrue(self.supervisor.error)
    def test_profiles_retained_after_round_trip(self):
        self.apply()
        self.apply(body='xingji', profile=default_config()['profiles']['xingji'], expected_version=2)
        self.assertEqual(self.store.load()['profiles']['gangben_zsl1']['local_ip'], '192.168.168.15')
    def test_nan_and_excessive_speed_rejected(self):
        for value in (float('nan'), float('inf'), 4, -0.5, True):
            profile = sdk_profile(); profile['max_vx'] = value
            with self.assertRaises(ValueError): validate_profile('gangben_zsl1', profile)
    def test_invalid_and_multicast_addresses_rejected(self):
        for value in ('bad; command', '224.0.0.1', '0.0.0.0', '127.0.0.1'):
            profile = sdk_profile(); profile['robot_ip'] = value
            with self.assertRaises(ValueError): validate_profile('gangben_zsl1', profile)


class ProductionContractTests(unittest.TestCase):
    def test_only_supervised_worker_in_production(self):
        launch = (ROOT/'launch'/'car_vel.launch.py').read_text(encoding='utf-8')
        backend = (ROOT/'launch'/'launch_manager.launch.py').read_text(encoding='utf-8')
        self.assertNotIn("executable='cmdvel_bridge_ros2'", launch)
        self.assertNotIn("executable='dog_remote_bridge'", backend)
        mapping = (ROOT/'launch'/'mapping.launch.py').read_text(encoding='utf-8')
        self.assertNotIn("executable='cmdvel_bridge_ros2'", mapping)
        mapping_page = (ROOT/'car_vel'/'web_frontend'/'mapping.html').read_text(encoding='utf-8')
        self.assertIn("'/dog_remote/ensure_standing'", mapping_page)
        worker = (ROOT/'car_vel'/'robot_body_bridge.py').read_text(encoding='utf-8')
        sdk = (ROOT/'car_vel'/'zsl1_bridge.py').read_text(encoding='utf-8')
        self.assertIn("Twist, '/cmd_vel_safe'", worker)
        self.assertIn("Twist, '/cmd_vel_safe'", sdk)
        self.assertNotIn("Twist, '/cmd_vel'", worker + sdk)
    def test_settings_entry_and_remote_capabilities_wired(self):
        web = ROOT/'car_vel'/'web_frontend'
        page = (web/'settings.html').read_text(encoding='utf-8')
        self.assertIn('bodySettings.attach(session)', page)
        script = (web/'robot_body_settings.js').read_text(encoding='utf-8')
        self.assertIn("'/robot_body/select'", script)
        self.assertIn('expected_version: formVersion', script)
        remote = (web/'remote_control.html').read_text(encoding='utf-8')
        self.assertIn('!specialActionsSupported', remote)
        self.assertIn('button.disabled = !specialActionsSupported', remote)


class XingjiWorkerCallbacksTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        source = ast.parse((ROOT/'car_vel'/'robot_body_bridge.py').read_text(encoding='utf-8'))
        definition = next(item for item in ast.walk(source)
                          if isinstance(item, ast.ClassDef) and item.name == 'XingjiBody')
        class Base:
            def __init__(self, **kwargs):
                self.remote_active = self.emergency_latched = False
                self.last_status_time = 100.0
                self.status_timeout = 3.0
                self.active_action = ''
                self.robot_status = {'motion_mode': 2, 'model': 0}
                self.sent = []
                self.received = []
                self.subscriptions = {}
                self.subscription_qos = {}
                self.state_lock = threading.RLock()
                self.requested_velocity = (0, 0, 0)
                self.last_velocity_command = 0.0
                self.log=Mock()
            def get_logger(self):return self.log
            def get_clock(self):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=int(self.last_status_time*1e9)))
            def create_subscription(self, _type, topic, callback, _qos):
                self.subscriptions[topic] = callback
                self.subscription_qos[topic] = _qos
            def _send_velocity(self, values): self.sent.append(values);return True
            def _send_zero_burst(self): self.sent.append((0, 0, 0))
            def _control_tick(self): self.sent.append('remote')
            def _navigation_gait_confirmed(self):
                return self.robot_status.get('model') == 0
            def _enable_callback(self, message): self.remote_active = message.data
            def _command_callback(self, message): self.received.append(message)
            def _velocity_callback(self, message): self.received.append(message)
            def _ensure_standing_callback(self, request, response): response.success = True; return response
            def _set_traversal_mode_callback(self, request, response): response.success = True; return response
            def _reset_robot(self): self.emergency_latched = False
            def _send_command(self, code): self.sent.append(('command', code))
            def _cancel_action(self): pass
            def _event(self, level, message): self.received.append((level, message))
            def _emergency_stop(self): self.emergency_latched = True; self.sent.append(('command', 6))
        namespace = dict(DogRemoteBridge=Base, TurnCommandShaper=TurnCommandShaper,
                         profile=default_config()['profiles']['xingji'], frozen=False,
                         Parameter=lambda *args, **kwargs: None, Twist=object,
                         Bool=object, Empty=object,
                         QoSProfile=types.SimpleNamespace,
                         HistoryPolicy=types.SimpleNamespace(KEEP_LAST='keep_last'),
                         ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT='best_effort'),
                         DurabilityPolicy=types.SimpleNamespace(VOLATILE='volatile'),
                         CommandStreamTiming=CommandStreamTiming,
                         MotionTrace=MotionTrace,age=age,velocity=velocity,body_reason=body_reason,generation=1,
                         json=json,
                         time=types.SimpleNamespace(monotonic=lambda: self.now))
        exec(compile(ast.Module(body=[definition], type_ignores=[]), '<production XingjiBody>', 'exec'), namespace)
        self.node = namespace['XingjiBody']()
    def command(self, x=0.1, yaw=0.08):
        return types.SimpleNamespace(linear=types.SimpleNamespace(x=x,y=0), angular=types.SimpleNamespace(z=yaw))
    def arm(self): self.node._ensure_standing_callback(None, types.SimpleNamespace())
    def arm_mapping(self):
        self.arm()
        self.node.mapping_teleop_active_callback(types.SimpleNamespace(data=True))
    def test_navigation_requires_explicit_standing_confirmation(self):
        self.node.nav_velocity(self.command()); self.node._control_tick()
        self.assertEqual(self.node.sent, [])
        self.arm()
        self.node.nav_velocity(self.command()); self.node._control_tick()
        self.assertAlmostEqual(self.node.sent[-1][0], 0.15)
    def test_existing_turn_ramp_and_command_timeout(self):
        self.arm()
        for _ in range(12):
            self.now += 0.05
            self.node.nav_velocity(self.command(0, 0.08)); self.node._control_tick()
        values = [value[2] for value in self.node.sent]
        self.assertAlmostEqual(values[0], 0.02)
        self.assertAlmostEqual(values[-1], 0.15)
        self.now += 0.31; self.node._control_tick()
        self.assertEqual(self.node.sent[-1], (0, 0, 0))
    def test_safe_velocity_prefers_latest_and_reports_a_receive_gap(self):
        qos = self.node.subscription_qos['/cmd_vel_safe']
        self.assertEqual((qos.history, qos.depth, qos.reliability, qos.durability),
                         ('keep_last', 1, 'best_effort', 'volatile'))
        self.arm()
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        first_yaw = self.node.sent[-1][2]
        self.now += 0.15
        self.node._control_tick()
        self.assertGreater(self.node.sent[-1][2], first_yaw)
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertGreater(self.node.sent[-1][0], 0)
        self.now += 0.36
        self.node._control_tick()
        self.assertEqual(self.node.sent[-1], (0, 0, 0))
        record = self.node._motion_trace.last_record
        self.assertEqual(record['reason_code'], 'safe_command_stale')
        stream = record['safe_velocity_stream']
        self.assertFalse(stream['fresh'])
        self.assertAlmostEqual(stream['max_gap_sec'], 0.36)
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertGreater(self.node.sent[-1][0], 0)
        self.assertAlmostEqual(self.node.sent[-1][2], first_yaw)
    def test_remote_excludes_navigation_and_drops_old_target(self):
        self.arm(); self.node.nav_velocity(self.command())
        self.node._enable_callback(types.SimpleNamespace(data=True))
        self.node.nav_velocity(self.command()); self.node._control_tick()
        self.assertEqual(self.node.sent[-1], 'remote')
        self.node._enable_callback(types.SimpleNamespace(data=False))
        self.node._control_tick()
        self.assertFalse(self.node.nav_armed)
    def test_disconnect_needs_new_standing_confirmation(self):
        self.arm(); self.node.nav_velocity(self.command()); self.node._control_tick()
        self.now += 3.1; self.node._control_tick()
        self.node.last_status_time = self.now
        self.node.nav_velocity(self.command()); self.node._control_tick()
        self.assertFalse(self.node.nav_armed)
        self.assertEqual(self.node.sent[-1], (0, 0, 0))
    def test_freeze_barrier_rejects_all_motion_sources(self):
        self.node.body_status = {'connected': True, 'motion_mode': 1}
        self.node.ever_connected = True
        self.assertTrue(self.node.prepare_switch()[0])
        self.node._enable_callback(types.SimpleNamespace(data=True))
        self.node._command_callback('stand')
        self.node._velocity_callback(self.command())
        self.node.nav_velocity(self.command()); self.node._control_tick()
        self.assertFalse(self.node.remote_active)
        self.assertEqual(self.node.received, [])
        self.assertEqual(self.node.sent, [(0, 0, 0)])
    def test_used_offline_or_standing_body_cannot_switch(self):
        self.node.ever_connected = True
        self.node.body_status = {'connected': False}
        self.assertFalse(self.node.prepare_switch()[0])
        self.node.body_status = {'connected': True, 'motion_mode': 2}
        self.assertFalse(self.node.prepare_switch()[0])

    def test_mapping_lio_pause_sends_zero_without_hardware_emergency(self):
        self.arm_mapping(); self.node.nav_velocity(self.command()); self.node._control_tick()
        self.node.subscriptions['/xingji/mapping/lio_stop'](None)
        self.assertEqual(self.node.sent[-1], (0, 0, 0))
        self.assertNotIn(('command', 6), self.node.sent)
        self.node.nav_velocity(self.command()); self.node._control_tick()
        self.assertEqual(self.node.sent[-1], (0, 0, 0))
        self.node.subscriptions['/xingji/mapping/lio_resume'](None)
        self.assertFalse(self.node.emergency_latched)
        self.assertFalse(self.node.nav_armed)

    def test_mapping_lio_recovery_accepts_fresh_teleop_without_toggle(self):
        self.arm_mapping()
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.node.mapping_lio_stop(None)
        self.node.mapping_lio_resume(None)

        # A held pre-pause direction must not restart motion on recovery.
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertEqual(self.node.sent[-1], (0, 0, 0))

        # Releasing, then pressing again should work without re-toggling the UI.
        self.node.mapping_teleop_neutral_callback(None)
        self.node.nav_velocity(self.command(0, 0))
        self.node._control_tick()
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertGreater(self.node.sent[-1][0], 0)

    def test_mapping_recovery_does_not_arm_unconfirmed_or_stale_body(self):
        self.node.mapping_lio_stop(None)
        self.node.mapping_lio_resume(None)
        self.node.nav_velocity(self.command(0, 0))
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertFalse(self.node.nav_armed)
        self.assertEqual(self.node.sent[-1], (0, 0, 0))

        self.arm_mapping()
        self.node.mapping_lio_stop(None)
        self.now += self.node.status_timeout + 0.1
        self.node._control_tick()
        self.node.last_status_time = self.now
        self.node.mapping_lio_resume(None)
        self.node.nav_velocity(self.command(0, 0))
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertFalse(self.node.nav_armed)
        self.assertEqual(self.node.sent[-1], (0, 0, 0))

    def test_real_emergency_during_lio_pause_cannot_auto_rearm(self):
        self.arm_mapping()
        self.node.mapping_lio_stop(None)
        self.node._emergency_stop()  # Explicit remote emergency, not legacy soft topic.
        self.node.clear_latch()
        self.node.mapping_lio_resume(None)
        self.node.nav_velocity(self.command(0, 0))
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertFalse(self.node.nav_armed)
        self.assertEqual(self.node.sent[-1], (0, 0, 0))

    def test_post_recovery_status_loss_cancels_pending_rearm(self):
        self.arm_mapping()
        self.node.mapping_lio_stop(None)
        self.node.mapping_lio_resume(None)
        self.now += self.node.status_timeout + 0.1
        self.node._control_tick()
        self.node.last_status_time = self.now
        self.node.nav_velocity(self.command(0, 0))
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertFalse(self.node.nav_armed)
        self.assertEqual(self.node.sent[-1], (0, 0, 0))

    def test_repeated_soft_pauses_keep_only_pending_neutral_rearm(self):
        self.arm_mapping()
        self.node.mapping_lio_stop(None)
        self.node.mapping_lio_resume(None)
        self.node.mapping_lio_stop(None)
        self.node.mapping_lio_resume(None)
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertEqual(self.node.sent[-1], (0, 0, 0))
        self.node.mapping_teleop_neutral_callback(None)
        self.node.nav_velocity(self.command(0, 0))
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertGreater(self.node.sent[-1][0], 0)

    def test_arbiter_timeout_zero_is_not_a_key_release(self):
        self.arm_mapping()
        self.node.mapping_lio_stop(None)
        self.node.mapping_lio_resume(None)
        self.node.nav_velocity(self.command(0, 0))
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertFalse(self.node.nav_armed)
        self.assertEqual(self.node.sent[-1], (0, 0, 0))

    def test_disabling_mapping_teleop_revokes_pending_authorization(self):
        self.arm_mapping()
        self.node.mapping_lio_stop(None)
        self.node.mapping_lio_resume(None)
        self.node.mapping_teleop_active_callback(types.SimpleNamespace(data=False))
        self.node.mapping_teleop_neutral_callback(None)
        self.node.nav_velocity(self.command(0, 0))
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertFalse(self.node.nav_armed)
        self.assertEqual(self.node.sent[-1], (0, 0, 0))

    def test_disabling_active_mapping_teleop_stops_without_lio_pause(self):
        self.arm_mapping()
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.node.mapping_teleop_active_callback(types.SimpleNamespace(data=False))
        self.assertFalse(self.node.nav_armed)
        self.assertEqual(self.node.sent[-1], (0, 0, 0))

    def test_inactive_mapping_page_does_not_disarm_other_navigation(self):
        self.arm()
        self.node.mapping_teleop_active_callback(types.SimpleNamespace(data=False))
        self.node.nav_velocity(self.command())
        self.node._control_tick()
        self.assertGreater(self.node.sent[-1][0], 0)

    def test_standing_confirmation_is_rejected_during_lio_pause(self):
        self.node.mapping_lio_stop(None)
        result = self.node._ensure_standing_callback(None, types.SimpleNamespace())
        self.assertFalse(result.success)
        self.assertFalse(self.node.nav_armed)

    def test_real_emergency_still_reaches_hardware_during_mapping_pause(self):
        self.node.subscriptions['/xingji/mapping/lio_stop'](None)
        self.node._emergency_stop()  # Button/T path is the only native emergency.
        self.node.subscriptions['/xingji/mapping/lio_resume'](None)
        self.assertIn(('command', 6), self.node.sent)
        self.assertTrue(self.node.emergency_latched)

    def test_mapping_lio_topics_follow_selected_body(self):
        from car_vel import mapping_lio_stop
        from car_vel.mapping_lio_stop import topics_for_body
        self.assertEqual(topics_for_body('xingji'),
                         ('/xingji/mapping/lio_stop', '/xingji/mapping/lio_resume'))
        self.assertEqual(topics_for_body('gangben_zsl1'),
                         ('/gangben/mapping/lio_stop', '/gangben/mapping/lio_resume'))
        with patch.object(mapping_lio_stop, 'BodyConfigStore') as store:
            store.return_value.load.return_value = {'selected': 'xingji'}
            self.assertEqual(mapping_lio_stop.mapping_lio_topics(),
                             topics_for_body('xingji'))
            store.return_value.load.return_value = {'selected': 'gangben_zsl1'}
            self.assertEqual(mapping_lio_stop.mapping_lio_topics(),
                             topics_for_body('gangben_zsl1'))
        mapping = (ROOT/'launch'/'mapping.launch.py').read_text(encoding='utf-8')
        self.assertIn('mapping_lio_topics()', mapping)
        self.assertIn("'recoverable_stop_topic': lio_stop_topic", mapping)
        self.assertNotIn("'emergency_stop_topic': lio_stop_topic", mapping)
        navigation = (ROOT/'launch'/'car_vel.launch.py').read_text(encoding='utf-8')
        self.assertNotIn('emergency_stop_topic', navigation)
        lio = (ROOT.parent/'lio/small_point_lio/launch/small_point_lio.launch.py').read_text(encoding='utf-8')
        self.assertIn("LaunchConfiguration('recoverable_stop_topic')", lio)
        self.assertIn("default_value='/emergency_stop'", lio)

    def test_mapping_pause_blocks_remote_velocity_and_actions(self):
        self.node.remote_active = True
        self.node.mapping_lio_stop(None)
        self.node._velocity_callback(self.command())
        self.node._command_callback(types.SimpleNamespace(data='{"command":"stand"}'))
        self.node._control_tick()
        self.assertEqual(self.node.requested_velocity, (0, 0, 0))
        self.assertNotIn(('command', 6), self.node.sent)
        self.assertEqual(self.node.sent[-1], (0, 0, 0))
        self.node.mapping_lio_resume(None)
        self.node._control_tick()
        self.assertEqual(self.node.sent[-1], 'remote')
        self.assertEqual(self.node.requested_velocity, (0, 0, 0))

    def test_unrelated_clear_does_not_reset_during_mapping_pause(self):
        self.node.mapping_lio_stop(None)
        self.node.clear_latch()
        self.assertTrue(self.node.mapping_lio_paused)
        self.assertFalse(self.node.emergency_latched)


class Zsl1RosCallbacksTests(unittest.TestCase):
    def setUp(self):
        from test_velocity_arbiter_callbacks import load_module, Node
        def module(name, **values):
            result = types.ModuleType(name); result.__dict__.update(values); return result
        class AwaitableFuture(ConcurrentFuture):
            def __await__(self):
                while not self.done():
                    yield self
                return self.result()
        with patch.dict(sys.modules, {
            'rclpy.task': module('rclpy.task', Future=AwaitableFuture),
            'rclpy.callback_groups': module('rclpy.callback_groups', ReentrantCallbackGroup=object),
            'std_srvs': module('std_srvs'),
            'std_srvs.srv': module('std_srvs.srv', Trigger=object),
            'lzp_interface': module('lzp_interface'),
            'lzp_interface.srv': module('lzp_interface.srv', SetTraversalMode=object),
        }):
            self.module = load_module('zsl1_bridge.py')
        self.now = 100.0
        self.module.time = types.SimpleNamespace(monotonic=lambda: self.now, sleep=lambda _: None)
        self.sdk = FakeSdk()
        profile = sdk_profile(); profile['navigation_verified'] = True
        self.module.Zsl1Adapter = lambda config: Zsl1Adapter(config, sdk=self.sdk)
        self.subscriptions = {}
        def capture_subscription(_node, _type, topic, callback, _qos):
            self.subscriptions[topic] = callback
        with patch.object(Node, 'create_service', create=True), patch.object(
                Node, 'create_subscription', capture_subscription):
            self.node = self.module.Zsl1Bridge(profile)
    def twist(self):
        return types.SimpleNamespace(linear=types.SimpleNamespace(x=0.3, y=0), angular=types.SimpleNamespace(z=0.2))
    def warm(self):
        self.node.stand()
        self.finish_stand()
        for _ in range(20):
            self.now += 0.05
            self.node.nav_velocity(self.twist()); self.node.tick()
    def finish_stand(self):
        for _ in range(82):
            self.now += 0.05
            self.node.heartbeat(None)
            self.node.tick()
    def immediate_response(self, coroutine):
        try:
            coroutine.send(None)
        except StopIteration as done:
            return done.value
        self.fail('Service should reject this request immediately')
    def test_no_motion_before_explicit_stand(self):
        self.node.nav_velocity(self.twist()); self.node.tick()
        self.assertEqual([name for name,_ in self.sdk.calls], ['init'])
    def test_stand_and_navigation_then_timeout_stop(self):
        self.warm(); self.assertTrue(self.node.adapter.moving)
        self.now += 0.31; self.node.tick()
        self.assertEqual(self.sdk.calls[-1], ('move', (0, 0, 0)))
    def test_link_loss_clears_authorization(self):
        self.warm(); self.sdk.connected = False; self.node.tick()
        self.assertFalse(self.node.motion_authorized)
        self.sdk.connected = True
        self.node.nav_velocity(self.twist()); self.node.tick()
        self.assertEqual(self.sdk.calls[-1], ('move', (0, 0, 0)))
    def test_unverified_navigation_is_rejected_but_remote_stand_available(self):
        self.node.profile['navigation_verified'] = False
        response = self.immediate_response(self.node.ensure_standing(None, types.SimpleNamespace()))
        self.assertFalse(response.success)
        self.node.stand()
        self.finish_stand()
        self.assertTrue(self.node.motion_authorized)
        self.node.nav_velocity(self.twist())
        self.assertEqual(self.node.command_at, 0)
    def test_unsupported_terrain_never_calls_sdk(self):
        response = self.immediate_response(self.node.traversal(types.SimpleNamespace(mode='stairs_up'), types.SimpleNamespace()))
        self.assertFalse(response.success)
        self.assertEqual(len(self.sdk.calls), 1)
    def test_unsupported_command_does_not_send_alternative(self):
        self.node.remote_active = True
        self.node.command(types.SimpleNamespace(data=json.dumps({'command':'toggle_obstacle_stop'})))
        self.assertEqual(len(self.sdk.calls), 1)
    def test_remote_heartbeat_loss_cannot_resume_as_navigation(self):
        self.node.stand()
        self.finish_stand()
        self.node.enable(types.SimpleNamespace(data=True))
        self.node.remote_velocity(self.twist()); self.node.tick()
        self.now += 0.5; self.node.tick()
        self.assertFalse(self.node.remote_active)
        self.node.nav_velocity(self.twist())
        self.assertFalse(self.node.motion_authorized)
    def test_freeze_and_activate_never_stands(self):
        self.assertTrue(self.node.prepare_switch()[0])
        self.node.command(types.SimpleNamespace(data=json.dumps({'command':'stand'})))
        self.node.nav_velocity(self.twist()); self.node.tick()
        self.node.activate()
        self.assertEqual(len(self.sdk.calls), 1)
        self.assertFalse(self.node.motion_authorized)

    def test_mapping_lio_stop_uses_normal_zero_move_not_passive(self):
        self.warm()
        self.subscriptions['/gangben/mapping/lio_stop'](None)
        self.assertEqual(self.sdk.calls[-1], ('move', (0, 0, 0)))
        self.assertNotIn(('passive', ()), self.sdk.calls)
        self.assertTrue(self.node.mapping_lio_paused)
        self.assertFalse(self.node.motion_authorized)
        self.node.nav_velocity(self.twist()); self.node.tick()
        self.assertEqual(self.node.adapter.target, (0, 0, 0))
        self.assertNotIn(('passive', ()), self.sdk.calls)

    def test_mapping_lio_resume_cannot_replay_velocity_or_reauthorize(self):
        self.warm()
        self.subscriptions['/gangben/mapping/lio_stop'](None)
        self.subscriptions['/gangben/mapping/lio_resume'](None)
        self.assertFalse(self.node.mapping_lio_paused)
        self.assertFalse(self.node.motion_authorized)
        self.node.nav_velocity(self.twist()); self.node.tick()
        self.assertEqual(self.sdk.calls[-1], ('move', (0, 0, 0)))
        self.assertNotIn(('passive', ()), self.sdk.calls)

    def test_real_emergency_still_calls_passive_during_mapping_pause(self):
        self.warm()
        self.subscriptions['/gangben/mapping/lio_stop'](None)
        self.node.command(types.SimpleNamespace(data='{"command":"emergency"}'))
        self.subscriptions['/gangben/mapping/lio_resume'](None)
        self.assertIn(('passive', ()), self.sdk.calls)
        self.assertTrue(self.node.adapter.emergency)

    def test_mapping_pause_blocks_steel_stand_and_remote_movement(self):
        self.node.mapping_lio_stop(None)
        self.node.remote_active = True
        self.node.command(types.SimpleNamespace(data=json.dumps({'command':'stand'})))
        self.node.remote_velocity(self.twist()); self.node.tick()
        self.assertEqual([name for name,_ in self.sdk.calls], ['init'])
        self.assertFalse(self.node.motion_authorized)

    def test_mapping_resume_rearms_stationary_walk_mode_without_passive(self):
        self.warm()
        self.sdk.mode = 18
        self.node.mapping_lio_stop(None)
        self.node.mapping_lio_resume(None)
        self.assertFalse(self.node.motion_authorized)
        stand_calls = sum(name == 'standUp' for name, _ in self.sdk.calls)
        self.node.stand()
        self.finish_stand()
        self.assertTrue(self.node.motion_authorized)
        self.assertEqual(sum(name == 'standUp' for name, _ in self.sdk.calls), stand_calls)
        self.assertNotIn(('passive', ()), self.sdk.calls)

    def test_sdk_failure_during_mapping_stop_keeps_zero_speed_fault_latch(self):
        self.warm()
        self.sdk.fail_move = True
        self.node.mapping_lio_stop(None)
        self.assertTrue(self.node.adapter.emergency)
        self.assertNotIn(('passive', ()), self.sdk.calls)
        self.assertEqual(self.sdk.calls[-1], ('move', (0.0, 0.0, 0.0)))

    def test_mapping_rearm_waits_for_stationary_feedback(self):
        self.warm()
        self.sdk.mode = 18
        self.node.mapping_lio_stop(None)
        self.node.mapping_lio_resume(None)
        self.sdk.velocity = [0.2, 0, 0]
        self.node.stand()
        for _ in range(50):
            self.now += 0.05
            self.node.tick()
        self.assertFalse(self.node.motion_authorized)
        self.sdk.velocity = [0, 0, 0]
        for _ in range(35):
            self.now += 0.05
            self.node.tick()
        self.assertTrue(self.node.motion_authorized)
        self.assertNotIn(('passive', ()), self.sdk.calls)


if __name__ == '__main__':
    unittest.main()
