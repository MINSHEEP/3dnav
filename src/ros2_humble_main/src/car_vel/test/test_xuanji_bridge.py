"""Three focused contracts; real ROS messages/services, no native robot connection."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT.parent / 'nav_service/lzp_service')]
import pytest
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('astrall_interfaces', reason='璇玑 SDK overlay 未加载')
pytest.importorskip('lzp_interface', reason='本项目 ROS 接口尚未编译')
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from lifecycle_msgs.srv import ChangeState
from astrall_interfaces.msg import ConnectionStatus, SportStatus, PowerStatus
from astrall_interfaces.srv import SetControlAuthority, SetMode
from car_vel.xuanji_bridge import XuanjiBridge
from car_vel.xuanji_navigation import selected_nav2_params
from lzp_service.robot_body import BodyConfigStore, default_config, validate_profile
import yaml


class XuanjiTests(unittest.TestCase):
    def test_saved_profiles_and_navigation_are_isolated(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robot_body.json'
            old = default_config()
            del old['profiles']['xuanji']
            old['selected'] = 'gangben_zsl1'
            old['version'] = 9
            path.write_text(json.dumps(old))
            loaded = BodyConfigStore(path).load()
            self.assertEqual(loaded['profiles']['gangben_zsl1'], old['profiles']['gangben_zsl1'])
            self.assertEqual((loaded['selected'], loaded['version']), ('gangben_zsl1', 9))
            profile = loaded['profiles']['xuanji']
            self.assertEqual(validate_profile('xuanji', profile)['max_vy'], 0)
            with self.assertRaises(ValueError):
                validate_profile('xuanji', dict(profile, max_vy=0.1))
            baseline = ROOT.parent / 'navigation/robot_navigation2/config/nav2_params.yaml'
            before = baseline.read_bytes()
            with patch('car_vel.xuanji_navigation.BodyConfigStore.load', return_value=loaded), patch('pathlib.Path.home', return_value=Path(directory)):
                self.assertEqual(selected_nav2_params(str(baseline)), str(baseline))
                loaded['selected'] = 'xuanji'
                nav = yaml.safe_load(Path(selected_nav2_params(str(baseline))).read_text())
            shared = yaml.safe_load(before)
            self.assertEqual(nav['planner_server'], shared['planner_server'])
            self.assertEqual(nav['local_costmap'], shared['local_costmap'])
            self.assertEqual(nav['global_costmap'], shared['global_costmap'])
            self.assertEqual(nav['velocity_smoother']['ros__parameters']['min_velocity'][0], 0)
            for key, value in nav['bt_navigator']['ros__parameters'].items():
                if key.startswith('default_nav_'):
                    self.assertNotIn('<BackUp', Path(value).read_text())
            self.assertEqual(before, baseline.read_bytes())

    def setUp(self):
        if self._testMethodName == 'test_saved_profiles_and_navigation_are_isolated':
            return
        rclpy.init(domain_id=171)
        self.sdk = Node('test_astrall_boundary')
        namespace = f'/xuanji_sdk_{os.getpid()}'
        self.calls, self.velocities = [], []
        self.sdk.create_service(ChangeState, namespace + '/astrall_driver/change_state', self.lifecycle)
        self.sdk.create_service(SetControlAuthority, namespace + '/set_control_authority', self.authority)
        self.sdk.create_service(SetMode, namespace + '/set_mode', self.mode)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.connection_pub = self.sdk.create_publisher(ConnectionStatus, namespace + '/connection_status', qos)
        self.sport_pub = self.sdk.create_publisher(SportStatus, namespace + '/sport_status', 1)
        self.sdk.create_subscription(Twist, namespace + '/cmd_vel', lambda msg: self.velocities.append(msg), 10)
        self.control = False
        self.actual_mode = SportStatus.FIXEDDOWN
        self.sdk.create_timer(0.05, self.feedback)
        self.profile = default_config()['profiles']['xuanji']
        self.profile['navigation_verified'] = True
        self.process = Mock()
        self.process.poll.return_value = None
        launcher = patch('car_vel.xuanji_bridge.subprocess.Popen', return_value=self.process)
        self.launch = launcher.start()
        self.addCleanup(launcher.stop)
        self.bridge = XuanjiBridge(self.profile)
        self.assertIn('mode_switch_timeout_ms:=8000', self.launch.call_args[0][0])
        self.bridge.driver_active = True
        self.bridge.on_power(PowerStatus(percentage=0.8))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.sdk)
        self.until(lambda: not self.bridge.action)
        self.assertFalse(self.bridge.armed)
        self.assertFalse(self.control)

    def tearDown(self):
        if not hasattr(self, 'bridge'):
            return
        self.bridge.close()
        self.executor.shutdown()
        self.bridge.destroy_node()
        self.sdk.destroy_node()
        rclpy.shutdown()

    def feedback(self):
        self.connection_pub.publish(ConnectionStatus(connected=True, has_control_authority=self.control))
        self.sport_pub.publish(SportStatus(status=self.actual_mode))

    def lifecycle(self, request, response):
        self.calls.append(('lifecycle', request.transition.id))
        response.success = True
        return response

    def authority(self, request, response):
        self.calls.append(('authority', request.authority))
        response.result.success = True
        response.result.code = 0x8008
        return response

    def mode(self, request, response):
        self.calls.append(('mode', request.mode))
        response.result.code = 0x8006  # ACK only; feedback is controlled separately.
        return response

    def pump(self, duration=0.15):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def until(self, condition):
        deadline = time.monotonic() + 4
        while not condition() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(condition(), self.bridge.error)

    def arm(self, via_service=False, mapping=False):
        if via_service:
            client = self.sdk.create_client(Trigger, '/dog_remote/ensure_mapping_motion' if mapping else '/dog_remote/ensure_standing')
            self.until(client.service_is_ready)
            future = client.call_async(Trigger.Request())
        else:
            future = self.bridge.stand()
        self.until(lambda: ('authority', 1) in self.calls)
        self.pump()
        self.assertFalse(self.bridge.armed, 'authority ACK cannot authorize movement')
        self.control = True
        self.until(lambda: ('mode', SetMode.Request.FIXEDSTAND) in self.calls)
        self.pump()
        self.assertFalse(future.done(), 'mode ACK cannot certify standing')
        self.actual_mode = SportStatus.FIXEDSTAND
        self.until(lambda: ('mode', SetMode.Request.MOVE) in self.calls)
        self.actual_mode = SportStatus.MOVE
        self.until(future.done)
        self.assertTrue(future.result().success if via_service else future.result()[0])
        self.assertTrue(self.bridge.armed)

    def test_mode_confirmation_motion_limits_and_watchdog(self):
        self.arm(via_service=True)
        move = Twist()
        move.linear.x, move.linear.y = 9.0, 2.0
        self.bridge.nav_velocity(move)
        self.pump(0.1)
        self.assertEqual((self.velocities[-1].linear.x, self.velocities[-1].linear.y), (0.5, 0.0))
        move.angular.z = 3.0
        self.bridge.nav_velocity(move)
        self.pump(0.1)
        self.assertEqual((self.velocities[-1].linear.x, self.velocities[-1].angular.z), (0.0, 0.6))
        move.angular.z, move.linear.x = 0.0, -1.0
        self.bridge.nav_velocity(move)
        self.pump(0.1)
        self.assertEqual(self.velocities[-1].linear.x, 0.0)
        move.linear.x = 0.3
        self.bridge.nav_velocity(move)
        self.pump(0.4)
        self.assertEqual(self.velocities[-1].linear.x, 0.0, 'stale commands must stop')
        self.bridge.remote_active = True
        self.bridge.heartbeat_at = time.monotonic() - 1
        self.pump()
        self.assertFalse(self.bridge.armed)
        self.assertTrue(self.bridge.emergency_latched)
        self.bridge.reset()
        self.until(lambda: not self.bridge.action)
        self.assertFalse(self.bridge.emergency_latched)
        self.assertFalse(self.bridge.armed, 'reset cannot restart motion')
        self.assertEqual(self.launch.call_count, 2)
        self.process.send_signal.assert_called_once()
        self.bridge.on_power(PowerStatus(percentage=0.52))
        with patch.object(self.bridge.status_publisher, 'publish') as publish:
            self.bridge.publish_status()
            self.assertEqual(json.loads(publish.call_args[0][0].data)['battery'], 52)
        self.bridge.on_diagnostic(DiagnosticArray(status=[DiagnosticStatus(
            level=DiagnosticStatus.ERROR, message='SDK initialization failed: ASTRALL_RES_TIMEOUT (0x8005)')]))
        with self.assertRaisesRegex(RuntimeError, '0x8005'):
            self.bridge.ready()

    def test_mapping_reverse_and_pause_without_navigation_authorization(self):
        self.profile['navigation_verified'] = False
        self.arm(via_service=True, mapping=True)
        move = Twist()
        move.linear.x, move.linear.y = -0.2, 0.0
        self.bridge.nav_velocity(move)
        self.pump()
        self.assertEqual(self.velocities[-1].linear.x, -0.2)
        self.bridge.mapping_stop(None)
        self.bridge.nav_velocity(move)
        self.pump()
        self.assertEqual(self.velocities[-1].linear.x, 0.0)
        self.bridge.mapping_resume(None)
        self.assertFalse(self.bridge.armed)
        self.bridge.mapping_teleop(Bool(data=False))
        self.bridge.armed = True
        self.bridge.nav_velocity(move)
        self.pump()
        self.assertEqual(self.velocities[-1].linear.x, 0.0)

    def test_remote_stop_down_and_switch(self):
        self.bridge.enable(Bool(data=True))
        heartbeat = self.sdk.create_timer(0.1, lambda: self.bridge.heartbeat(None))
        self.bridge.command(String(data='{"command":"request_control"}'))
        self.until(lambda: ('authority', 1) in self.calls)
        self.control = True
        self.until(lambda: not self.bridge.action)
        self.bridge.command(String(data='{"command":"stand"}'))
        self.until(lambda: ('mode', SetMode.Request.FIXEDSTAND) in self.calls)
        self.pump(3.2)
        self.assertEqual(self.bridge.action, 'fixed_stand')
        self.assertFalse(self.bridge.armed)
        self.actual_mode = SportStatus.FIXEDSTAND
        self.until(lambda: not self.bridge.action)
        self.assertFalse(self.bridge.armed, 'remote stand must not enter MOVE automatically')
        self.bridge.command(String(data='{"command":"move_mode"}'))
        self.until(lambda: ('mode', SetMode.Request.MOVE) in self.calls)
        self.bridge.command(String(data='{"command":"stand"}'))
        self.assertEqual(self.bridge.action, 'move', 'an in-flight MOVE must not be replaced by stand')
        self.actual_mode = SportStatus.MOVE
        self.until(lambda: not self.bridge.action)
        self.assertTrue(self.bridge.armed)
        move = Twist()
        move.linear.x = 0.2
        self.bridge.remote_velocity(move)
        self.pump()
        self.assertEqual(self.velocities[-1].linear.x, 0.2)
        move.linear.x = -0.2
        self.bridge.remote_velocity(move)
        self.pump()
        self.assertEqual(self.velocities[-1].linear.x, -0.2)
        move.linear.x, move.linear.y = 0.0, 0.8
        self.bridge.remote_velocity(move)
        self.pump()
        self.assertEqual(self.velocities[-1].linear.y, 0.2)
        move.linear.y = -0.8
        self.bridge.remote_velocity(move)
        self.pump()
        self.assertEqual(self.velocities[-1].linear.y, -0.2)
        self.bridge.command(String(data='{"command":"stop"}'))
        self.pump(0.05)
        self.assertEqual(self.velocities[-1].linear.x, 0.0)
        self.assertTrue(self.bridge.armed, 'ordinary release must allow the next movement')
        self.bridge.command(String(data='{"command":"lie_down"}'))
        self.until(lambda: ('mode', SetMode.Request.FIXEDDOWN) in self.calls)
        self.assertFalse(self.bridge.armed)
        self.actual_mode = SportStatus.FIXEDDOWN
        self.until(lambda: not self.bridge.action)
        self.bridge.enable(Bool(data=False))
        self.assertTrue(self.bridge.prepare_switch()[0])
        self.bridge.remote_velocity(move)
        self.pump(0.1)
        self.assertEqual(self.velocities[-1].linear.x, 0.0)
        self.bridge.activate()
        self.bridge.enable(Bool(data=True))
        self.bridge.command(String(data='{"command":"emergency"}'))
        self.until(lambda: ('mode', SetMode.Request.DAMPING) in self.calls)
        self.actual_mode = SportStatus.DAMPING
        self.until(lambda: not self.bridge.action)
        self.assertTrue(self.bridge.emergency_latched)
        self.assertFalse(self.bridge.armed)
        self.bridge.reset()
        self.until(lambda: not self.bridge.action)
        self.assertFalse(self.bridge.emergency_latched)
        self.assertFalse(self.bridge.armed)
        self.sdk.destroy_timer(heartbeat)


if __name__ == '__main__':
    unittest.main()
