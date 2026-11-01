import ast
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from car_vel.lidar_network import NetworkStore, validate, local_address_warning


class NetworkTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.template = ROOT.parent / 'driver/livox_ros_driver2/config/MID360s_config.json'
        self.store = NetworkStore(self.template, self.temp.name)
        # Linux advisory lock is not available on Windows; mock only that OS boundary.
        if sys.platform == 'win32':
            self.lock = patch.dict(sys.modules, {'fcntl': types.SimpleNamespace(LOCK_EX=2, flock=lambda *a: None)})
            self.lock.start()
            self.addCleanup(self.lock.stop)

    def test_defaults_from_installed_template(self):
        self.assertEqual(self.store.load(), {'version': 0, 'lidar_ip': '192.168.1.130', 'host_ip': '192.168.1.50'})

    def test_invalid_addresses(self):
        for address in ('', 'x; reboot', '::1', 'localhost', '0.0.0.0', '127.0.0.1',
                        '224.1.1.1', '255.255.255.255', '0.1.2.3', '192.168.1.999', 123):
            with self.subTest(address=address), self.assertRaises(ValueError):
                validate({'version': 0, 'lidar_ip': address, 'host_ip': '192.168.1.50'})

    def test_same_addresses_rejected(self):
        with self.assertRaises(ValueError):
            self.store.save('192.168.1.50', '192.168.1.50', 0)
        self.assertFalse(self.store.path.exists())

    def test_atomic_save_and_version(self):
        self.store.save('192.168.2.130', '192.168.2.50', 0)
        self.assertEqual(self.store.load()['version'], 1)
        with self.assertRaisesRegex(ValueError, '刷新'):
            self.store.save('192.168.3.130', '192.168.3.50', 0)
        self.assertEqual(self.store.load()['host_ip'], '192.168.2.50')

    def test_failed_save_preserves_previous_file(self):
        self.store.save('192.168.2.130', '192.168.2.50', 0)
        before = self.store.path.read_bytes()
        with patch('car_vel.lidar_network.os.replace', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                self.store.save('192.168.3.130', '192.168.3.50', 1)
        self.assertEqual(before, self.store.path.read_bytes())
        self.assertFalse(list(Path(self.temp.name).glob('.lidar-*')))

    def test_snapshot_preserves_ports_calibration_and_source(self):
        original = self.template.read_bytes()
        expected = json.loads(original)
        self.store.save('192.168.2.130', '192.168.2.50', 0)
        snapshot = Path(self.store.snapshot())
        expected['lidar_configs'][0]['ip'] = '192.168.2.130'
        expected['Mid360s']['host_net_info'][0]['host_ip'] = '192.168.2.50'
        self.assertEqual(json.loads(snapshot.read_text()), expected)
        self.assertEqual(self.template.read_bytes(), original)

    def test_save_cannot_mutate_live_snapshot(self):
        first = Path(self.store.snapshot())
        before = first.read_bytes()
        self.store.save('192.168.2.130', '192.168.2.50', 0)
        second = Path(self.store.snapshot())
        self.assertNotEqual(first, second)
        self.assertEqual(first.read_bytes(), before)

    def test_corrupt_saved_config_fails_closed(self):
        self.store.path.write_text('{broken')
        with self.assertRaises(ValueError):
            self.store.snapshot()

    def test_unconfigured_host_warning(self):
        with patch('car_vel.lidar_network.socket.socket', side_effect=OSError('unavailable')):
            self.assertIn('网卡', local_address_warning('192.168.1.50'))

    def test_both_launches_use_snapshot_and_driver_accepts_override(self):
        for name in ('mapping.launch.py', 'car_vel.launch.py'):
            source = (ROOT / 'launch' / name).read_text(encoding='utf-8')
            ast.parse(source)
            self.assertIn("launch_arguments={'user_config_path': runtime_config_path()}.items()", source)
        driver = (self.template.parents[1] / 'launch_ROS2/msg_MID360s_launch.py').read_text(encoding='utf-8')
        self.assertIn("LaunchConfiguration('user_config_path')", driver)
        self.assertIn("DeclareLaunchArgument('user_config_path'", driver)
        ast.parse(driver)

    def test_deployment_wiring(self):
        manager = (ROOT / 'launch/launch_manager.launch.py').read_text(encoding='utf-8')
        ast.parse(manager)
        self.assertIn("executable='lidar_network_bridge'", manager)
        self.assertIn('lidar_network_bridge = car_vel.lidar_network_bridge:main', (ROOT / 'setup.py').read_text())
        cmake = ROOT.parent / 'nav_service/lzp_interface/CMakeLists.txt'
        self.assertIn('srv/SetLidarNetwork.srv', cmake.read_text(encoding='utf-8'))


class ServiceTests(unittest.TestCase):
    def setUp(self):
        NetworkTests.setUp(self)
        from test_velocity_arbiter_callbacks import load_module, Node
        interface = types.ModuleType('lzp_interface.srv')
        interface.SetLidarNetwork = object
        with patch.dict(sys.modules, {'lzp_interface.srv': interface}):
            self.module = load_module('lidar_network_bridge.py')
        self.module.installed_store = lambda: self.store
        self.module.local_address_warning = lambda ip: 'test host warning'
        with patch.object(Node, 'create_service', create=True):
            self.node = self.module.LidarNetworkBridge()

    def request(self, operation='get', version=0):
        return self.node.configure(types.SimpleNamespace(operation=operation,
            lidar_ip='192.168.2.130', host_ip='192.168.2.50', expected_version=version),
            types.SimpleNamespace())

    def test_get_does_not_write_settings(self):
        result = self.request()
        self.assertTrue(result.success)
        self.assertFalse(self.store.path.exists())
        self.assertEqual(json.loads(result.status_json)['effective_policy'], 'next_start')

    def test_save_reports_next_start_and_warning(self):
        result = self.request('save')
        self.assertTrue(result.success)
        self.assertIn('下次启动', result.message)
        self.assertEqual(json.loads(result.status_json)['warning'], 'test host warning')
        self.assertEqual(self.store.load()['version'], 1)

    def test_stale_save_reports_failure(self):
        self.request('save')
        self.assertFalse(self.request('save').success)
        self.assertEqual(self.store.load()['version'], 1)

    def test_corrupt_status_remains_available(self):
        self.store.path.write_text('{bad')
        state = self.node.status()
        self.assertIsNone(state['config'])
        self.assertTrue(state['error'])
        self.assertFalse(self.request('save').success)

    def test_unknown_operation_is_rejected(self):
        self.assertFalse(self.request('restart').success)
        self.assertFalse(self.store.path.exists())


if __name__ == '__main__':
    unittest.main()
