"""Debug navigation scenarios with an in-memory ROS peer; no robot or audio."""
import base64
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_vel.voice_debug_navigation import DebugNavigation, map_image
from car_vel.voice_debug_server import DebugController, DebugError, PHRASES
from test_voice_debug import App


class FakePeer:
    def __init__(self, now):
        self.now = now
        self.calls = []
        self.messages = {}
        self.clients = []
        self.points = [{'id': 100 + index, 'sequence': index, 'name': f'点{index + 1}',
                        'x': index, 'y': 0, 'yaw': 0} for index in range(11)]
        self.route = {'id': 21, 'name': 'luxian1', 'map_name': 'shiyanshi1', 'points': self.points}
        self.selected = []
        self.cursor = 0
        self.releases = []
        self.fail_preview = False
        self.pose_success = True
        self.hold_start = None
        self.start_received = threading.Event()
        self.odometry_available = True
        self.fail_start = False
        self.initial_pose = None
        self.amcl_sequence = 0

    def factory(self, url, callback):
        client = FakeClient(self, callback)
        self.clients.append(client)
        return client

    def status(self, mode='route', state='awaiting_release'):
        point = self.selected[min(self.cursor, len(self.selected) - 1)] if self.selected else {}
        return {'mode': mode, 'state': state, 'route_id': self.route['id'],
                'map_name': 'shiyanshi1', 'current_point': point}

    def call(self, service, args, cancel):
        self.calls.append((service, args))
        if service == '/switch_map':
            self.messages['/current_map_info'] = {'success': True, 'map_name': 'shiyanshi1', 'load_id': 'load-1'}
        elif service == '/map_server/load_map':
            return {'result': 0}
        elif service == '/navigation/set_mode':
            return {'success': True, 'mode': args['mode']}
        elif service == '/list_routes':
            return {'success': True, 'routes': [self.route]}
        elif service == '/get_route':
            return {'success': True, 'route': self.route}
        elif service == '/navigation/preview_3d_route':
            return {'valid': not self.fail_preview, 'preview_id': 'preview-1',
                    'message': '目标点为未知或禁止通行，且容差范围内没有同连通域替代格'}
        elif service == '/start_route':
            if self.fail_start:
                return {'success': False, 'message': '二维路线启动失败'}
            self.start_received.set()
            if self.hold_start:
                # Simulate a request accepted after the page has pressed STOP.
                self.hold_start.wait(2)
            first = next(index for index, point in enumerate(self.points) if point['id'] == args['start_point_id'])
            self.selected = self.points[first:first + args['point_limit']]
            self.cursor = 0
            self.messages['/route_execution_status'] = self.status()
        elif service == '/control_route':
            self.releases.append((self.selected[self.cursor]['id'], self.now[0]))
            self.cursor += 1
            self.messages['/route_execution_status'] = (
                self.status() if self.cursor < len(self.selected) else self.status('idle', 'completed'))
        return {'success': True}


class FakeClient:
    def __init__(self, peer, callback):
        self.peer, self.callback = peer, callback
        self.closed = threading.Event()
        self.published = []

    def connect(self, cancel, timeout=60):
        pass

    def close(self):
        self.closed.set()

    def send(self, data):
        pass

    def subscribe(self, *args, **kwargs):
        pass

    def call(self, service, kind, args, cancel, timeout=15):
        return self.peer.call(service, args, cancel)

    def latest(self, topic):
        return (self.peer.now[0], self.peer.messages.get(topic, {}))

    def wait(self, topic, predicate, cancel, timeout=60, after=0):
        if topic == '/nav2_ready':
            result = {'data': True}
        elif topic == '/map':
            result = {'info': {'width': 2, 'height': 2, 'resolution': 0.5,
                              'origin': {'position': {'x': -1, 'y': -2, 'z': 0},
                                         'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}},
                      'data': [-1, 0, 100, 50]}
        elif topic == '/Odometry':
            if not self.peer.odometry_available:
                raise RuntimeError('等待导航状态超时：/Odometry')
            result = {'header': {'stamp': {'sec': 123, 'nanosec': 456}}}
        elif topic == '/scan':
            result = {'header': {'stamp': {'sec': 123, 'nanosec': 456}}, 'ranges': [1, 2, 3]}
        elif topic == '/localization/navigation_pose':
            if not self.peer.pose_success:
                raise RuntimeError('二维定位校正失败')
            self.peer.amcl_sequence += 1
            self.peer.now[0] += 0.3
            initial = self.peer.initial_pose['pose']['pose']
            result = {'header': {'frame_id': 'map', 'stamp': {'sec': 124, 'nanosec': self.peer.amcl_sequence}},
                      'pose': {'pose': initial, 'covariance': [0.001] * 36}}
        else:
            result = self.peer.messages[topic]
        if not predicate(result):
            raise RuntimeError('Fake peer predicate did not match: ' + topic)
        return result

    def publish(self, topic, kind, message, **kwargs):
        self.published.append((topic, message))
        if topic == '/current_map_info' and message != self.peer.messages.get(topic):
            self.peer.messages[topic] = message
            self.peer.messages['/route_execution_status'] = self.peer.status('idle', 'idle')
        if topic == '/initialpose':
            self.peer.initial_pose = message
        if topic == '/relocalization/request':
            self.peer.messages['/relocalization/status'] = {'data': json.dumps({
                'map_name': message['map_name'], 'request_id': message['request_id'],
                'request_stamp': message['rough_pose']['header']['stamp'],
                'terminal': True, 'success': self.peer.pose_success, 'message': '定位失败'})}


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.app = App()
        self.commands = []
        self.app.action_sender._send = self.commands.append
        self.now = [100.0]
        self.controller = DebugController(self.app, clock=lambda: self.now[0])
        self.token = self.controller.dispatch('/api/voice-debug/session', {})['session']
        self.runtime = SimpleNamespace(start=lambda: None, close=lambda: None, port=9091)
        self.peer = FakePeer(self.now)
        self.nav = DebugNavigation(self.controller, self.runtime,
                                   anchor_path=Path(self.folder.name) / 'anchor.json',
                                   client_factory=self.peer.factory, clock=lambda: self.now[0])
        self.controller.navigation = self.nav
        self.addCleanup(self.nav.close)
        self.sequence = 0

    def send(self, action, **data):
        self.sequence += 1
        return self.controller.dispatch('/api/voice-debug/' + action,
                                       dict(session=self.token, sequence=self.sequence, **data))

    def wait_for(self, predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail('Timed out: ' + str(self.nav.snapshot()))

    def prepared(self):
        self.nav.anchor = {'x': 1.2, 'y': -3.4, 'yaw': 0.6}
        self.send('navigation/prepare')
        self.wait_for(lambda: self.nav.ready and not self.nav.worker.is_alive())

    def test_saved_map_is_visible_before_robot_or_ros_startup(self):
        self.assertTrue(self.nav.snapshot()['map_available'],
                        'The first-use map must be visible before stand/ROS/map-switch succeed')
        self.assertGreater(self.nav.map_cache['width'], 1)
        self.assertFalse(self.nav.ready)

    def test_debug_preparation_selects_the_existing_2d_mode(self):
        self.prepared()
        request = next(args for name, args in self.peer.calls if name == '/navigation/set_mode')
        self.assertEqual(request['mode'], 'map_2d')

    def test_2d_route_does_not_call_3d_unknown_cell_preview(self):
        self.prepared()
        self.peer.fail_preview = True
        self.send('navigation/route1')
        self.wait_for(lambda: self.nav.state in ('completed', 'error'))
        self.assertNotEqual(self.nav.state, 'error', self.nav.message)
        self.assertFalse(any(name == '/navigation/preview_3d_route' for name, _ in self.peer.calls))
        request = next(args for name, args in self.peer.calls if name == '/start_route')
        self.assertEqual(request['navigation_mode'], 'map_2d')
        self.assertEqual(request['preview_id'], '')

    def test_stand_failure_keeps_the_first_use_map_visible(self):
        self.app.action_sender._motion_mode = 1
        self.app.action_sender.stand_up = lambda: False
        self.send('navigation/prepare')
        self.wait_for(lambda: self.nav.state == 'error')
        self.assertTrue(self.nav.snapshot()['map_available'],
                        'A failed stand must not hide the map used to set area a')
        self.assertFalse(self.nav.ready)

    def test_first_run_requires_a_real_coarse_pose_and_persists_only_debug_anchor(self):
        self.send('navigation/prepare')
        self.wait_for(lambda: self.nav.state == 'awaiting_anchor' and not self.nav.worker.is_alive())
        self.assertFalse(self.nav.ready)
        self.assertFalse(any(topic == '/initialpose'
                             for client in self.peer.clients for topic, _ in client.published))
        self.send('navigation/anchor', x=1.2, y=-3.4, yaw=0.6)
        self.wait_for(lambda: self.nav.ready and not self.nav.worker.is_alive())
        self.assertEqual(json.loads(self.nav.anchor_path.read_text())['map_name'], 'shiyanshi1')
        request = next(message for client in self.peer.clients for topic, message in client.published
                       if topic == '/initialpose')
        self.assertEqual(request['pose']['pose']['position']['x'], 1.2)
        self.assertEqual(request['header']['frame_id'], 'map')
        self.assertEqual(request['header']['stamp'], {'sec': 122, 'nanosec': 800000456})
        self.assertFalse(any(topic == '/relocalization/request'
                             for client in self.peer.clients for topic, _ in client.published))
        self.assertFalse(any(service == '/start_route' for service, _ in self.peer.calls))

    def test_failed_localization_never_enables_routes(self):
        self.nav.anchor = {'x': 1, 'y': 2, 'yaw': 0}
        self.peer.pose_success = False
        self.send('navigation/prepare')
        self.wait_for(lambda: self.nav.state == 'error')
        self.assertFalse(self.nav.ready)
        with self.assertRaises(DebugError):
            self.send('navigation/route1')

    def test_missing_odometry_reports_lio_fault_and_never_enables_navigation(self):
        self.nav.anchor = {'x': 1, 'y': 2, 'yaw': 0}
        self.peer.odometry_available = False
        self.peer.messages['/lio_health'] = {'data': json.dumps({
            'state': 'fault', 'code': 'odometry_diverged', 'reason': 'scan_matching_quality'})}
        self.send('navigation/prepare')
        self.wait_for(lambda: self.nav.state == 'error')
        self.assertRegex(self.nav.message, '定位.*故障')
        self.assertFalse(self.nav.ready)

    def test_2d_amcl_confirmation_rejects_bad_covariance_and_wrong_map_frame(self):
        self.prepared()
        sample = FakeClient(self.peer, lambda _: None).wait('/localization/navigation_pose', lambda _: True,
                                                          threading.Event())
        self.assertTrue(self.nav._amcl_sample_valid(sample))
        sample['pose']['covariance'][0] = float('nan')
        self.assertFalse(self.nav._amcl_sample_valid(sample))
        sample['pose']['covariance'][0] = 0.001
        sample['header']['frame_id'] = 'odom'
        self.assertFalse(self.nav._amcl_sample_valid(sample))

    def test_lio_fault_stops_2d_route_before_any_3d_recovery_can_resume_it(self):
        self.prepared()
        self.send('navigation/route1')
        self.wait_for(lambda: self.nav.driving)
        self.peer.messages['/lio_health'] = {'data': json.dumps({'state': 'fault'})}
        self.controller.tick()
        self.assertFalse(self.nav.driving)
        self.assertIn('二维导航已停止', self.nav.message)
        self.assertEqual(self.app.action_sender.velocities[-1], (0, 0, 0))

    def test_route1_visits_3_to_11_without_dwell_and_sends_first_phrase(self):
        self.prepared()
        started_at = self.now[0]
        self.send('navigation/route1')
        self.wait_for(lambda: self.nav.state == 'completed')
        start = next(args for name, args in self.peer.calls if name == '/start_route')
        self.assertEqual((start['start_point_id'], start['point_limit']), (102, 9))
        self.assertTrue(start['hold_at_each_point'])
        self.assertEqual([point for point, _ in self.peer.releases], list(range(102, 111)))
        self.assertTrue(all(released_at == started_at for _, released_at in self.peer.releases))
        self.assertEqual([packet['content']['input'] for packet in self.app.sent], [PHRASES['first']])
        self.assertFalse(self.nav.driving)
        self.assertFalse(self.app.web_microphone_enabled)
        self.assertEqual(self.app.action_sender.velocities[-1], (0, 0, 0))

    def test_route2_visits_3_to_11_and_preserves_current_speech(self):
        self.prepared()
        self.app._tts_round_active = True
        self.send('navigation/route2')
        self.wait_for(lambda: len(self.peer.releases) == 9)
        self.assertEqual(self.app.sent, [])
        self.app._tts_round_active = False
        self.wait_for(lambda: self.nav.state == 'completed')
        self.assertEqual([point for point, _ in self.peer.releases], list(range(102, 111)))
        self.assertEqual([packet['content']['input'] for packet in self.app.sent], [PHRASES['second']])

    def test_manual_takeover_blocks_late_navigation_velocity_and_does_not_stop_speech(self):
        self.prepared()
        self.app._tts_round_active = True
        self.send('navigation/route1')
        self.wait_for(lambda: self.nav.driving)
        generation = self.nav.generation
        velocity = {'topic': '/cmd_vel_safe', 'msg': {'linear': {'x': 0.3, 'y': 0}, 'angular': {'z': 0}}}
        self.nav._on_velocity(velocity, generation)
        self.controller.tick()
        self.assertEqual(self.app.action_sender.velocities[-1], (0.3, 0.0, 0.0))
        self.send('move', direction='left', speed=0.2)
        self.nav._on_velocity(velocity, generation)
        self.controller.tick()
        self.assertFalse(self.nav.driving)
        self.assertEqual(self.app.action_sender.velocities[-1], (0, 0.2, 0))
        self.assertTrue(self.app._tts_round_active)
        self.send('stop')
        self.assertEqual(self.app.action_sender.velocities[-1], (0, 0, 0))

    def test_stop_during_start_rpc_prevents_late_acceptance_from_rearming(self):
        self.prepared()
        self.peer.hold_start = threading.Event()
        self.send('navigation/route1')
        self.assertTrue(self.peer.start_received.wait(2))
        self.send('stop')
        self.peer.hold_start.set()
        self.wait_for(lambda: not self.nav.worker.is_alive())
        self.assertFalse(self.nav.driving)
        self.assertFalse(self.app._web_navigation)
        self.controller.tick()
        self.assertFalse(any(any(value for value in velocity)
                             for velocity in self.app.action_sender.velocities))
        self.assertTrue(any(topic == '/cancel_navigation'
                            for client in self.peer.clients for topic, _ in client.published))

    def test_rejected_2d_route_never_sends_phrase(self):
        self.prepared()
        self.peer.fail_start = True
        self.send('navigation/route2')
        self.wait_for(lambda: self.nav.state == 'error')
        self.assertFalse(self.nav.driving)
        self.assertEqual(self.app.sent, [])

    def test_stale_navigation_request_after_stop_is_ignored(self):
        self.send('stop')
        result = self.controller.dispatch('/api/voice-debug/navigation/prepare',
                                         {'session': self.token, 'sequence': 1})
        self.assertTrue(result['ignored'])
        self.assertIsNone(self.nav.worker)

    def test_velocity_connection_loss_stops_instead_of_silently_waiting(self):
        self.prepared()
        self.send('navigation/route1')
        self.wait_for(lambda: self.nav.driving)
        self.nav.velocity_client.closed.set()
        self.controller.tick()
        self.assertFalse(self.nav.driving)
        self.assertIn('速度连接中断', self.nav.message)
        self.assertEqual(self.app.action_sender.velocities[-1], (0, 0, 0))

    def test_page_lease_expiry_cancels_navigation_and_does_not_rearm_on_new_session(self):
        self.prepared()
        self.send('navigation/route1')
        self.wait_for(lambda: self.nav.driving)
        self.now[0] += 3
        self.controller.tick()
        self.assertFalse(self.nav.driving)
        self.assertFalse(self.app.web_microphone_enabled)
        self.controller.dispatch('/api/voice-debug/session', {})
        self.controller.tick()
        self.assertFalse(self.nav.driving)
        self.assertEqual(self.app.action_sender.velocities[-1], (0, 0, 0))

    def test_idle_terrain_return_to_normal_keeps_obstacle_mode_without_fault(self):
        self.prepared()
        self.nav._on_message({'op': 'call_service', 'id': 'mode-id',
                              'service': '/dog_remote/set_traversal_mode', 'args': {'mode': 'normal'}})
        self.assertEqual(self.commands[-1], {'msg_type': 3, 'cmd_code': 2, 'para': 0})

    def test_narration_waits_for_pending_cloud_reply_before_tts_arrives(self):
        self.app.web_text(PHRASES['first'])
        self.assertTrue(self.app.web_voice_busy())
        self.app._handle_message({'code': 200, 'contentType': 'TTS', 'content': {}})
        self.assertFalse(self.app.web_voice_busy())

    def test_map_encoding_keeps_origin_orientation_and_occupancy(self):
        bridge = FakeClient(self.peer, lambda _: None)
        grid = bridge.wait('/map', lambda _: True, threading.Event())
        result = map_image(grid)
        self.assertEqual(base64.b64decode(result['pixels']), bytes([110, 255, 0, 128]))
        self.assertEqual((result['width'], result['height'], result['resolution']), (2, 2, 0.5))
        self.assertEqual(result['origin']['position']['x'], -1)


if __name__ == '__main__':
    unittest.main()
