import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_vel.m701_connection import ConfigStore, ConnectionError, draft, identity, inventory, observation, resolve, validate


def device(port='/dev/ttyUSB0', serial='unique', location='/dev/serial/by-path/usb-port1'):
    return dict(port=port, aliases=['/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'],
                vid='1a86', pid='7523', serial=serial, interface='00', channel='0',
                location=location, driver='ch341', product='USB Serial', candidate=True)


def config(item=None, mode='auto'):
    item = item or device()
    return dict(version=0, mode=mode, port=item['port'], baudrate=9600,
                address=1, binding=identity(item))


class ConnectionTests(unittest.TestCase):
    def test_auto_follows_identity_not_tty_number(self):
        saved = config()
        wrong = device(serial='other')
        right = device('/dev/ttyCH341USB4')
        self.assertEqual(resolve(saved, [wrong, right])['port'], right['port'])

    def test_missing_never_falls_back(self):
        with self.assertRaisesRegex(ConnectionError, '不会切换'):
            resolve(config(), [device(serial='other')])

    def test_duplicate_identity_rejected(self):
        with self.assertRaisesRegex(ConnectionError, '多个串口'):
            resolve(config(), [device(), device('/dev/ttyUSB1')])

    def test_no_serial_binds_physical_port(self):
        item = device(serial='')
        moved = device('/dev/ttyCH348USB3', serial='', location='another-port')
        with self.assertRaises(ConnectionError): resolve(config(item), [moved])
        moved['location'] = item['location']
        self.assertEqual(resolve(config(item), [moved]), moved)

    def test_manual_reused_node_refused(self):
        saved = config(mode='manual')
        with self.assertRaises(ConnectionError):
            resolve(saved, [device(serial='other'), device('/dev/ttyUSB1')])

    def test_multi_channel_identity_distinguishes_ports(self):
        first, second = device(), device('/dev/ttyUSB1')
        second['channel'] = '1'
        self.assertEqual(resolve(config(second), [first, second]), second)

    def test_explicit_duplicate_serial_pins_unique_location(self):
        first, second = device(), device('/dev/ttyUSB1', location='another-port')
        payload = dict(mode='auto', port=second['port'], baudrate=9600, address=1,
                       identity=observation(second), confirmed=True)
        saved = draft(config(), payload, [first, second])
        self.assertEqual(saved['binding']['location'], 'another-port')
        self.assertEqual(resolve(saved, [second]), second)
        with self.assertRaises(ConnectionError): resolve(saved, [first])

    def test_draft_requires_confirmation_and_current_identity(self):
        item = device()
        request = dict(mode='auto', port=item['port'], baudrate=9600, address=1,
                       identity=observation(item), confirmed=True)
        self.assertEqual(draft(config(), request, [item])['binding'], identity(item))
        request['confirmed'] = False
        with self.assertRaises(ValueError): draft(config(), request, [item])
        request['confirmed'] = True; request['identity']['serial'] = 'outdated'
        with self.assertRaises(ConnectionError): draft(config(), request, [item])

    def test_validate_rejects_bad_numbers_and_fields(self):
        for key, value in [('baudrate', True), ('address', 8), ('address', float('nan')),
                           ('mode', 'guess'), ('port', '/etc/passwd')]:
            candidate = config(); candidate[key] = value
            with self.assertRaises(ValueError): validate(candidate)

    def test_atomic_store_and_corruption(self):
        with tempfile.TemporaryDirectory() as folder:
            store = ConfigStore(Path(folder) / 'config.json')
            store.save(config())
            self.assertEqual(store.load(config()), config())
            with patch('car_vel.m701_connection.os.replace', side_effect=OSError('disk full')):
                with self.assertRaises(OSError): store.save(dict(config(), address=2))
            self.assertEqual(store.load(config())['address'], 1)
            self.assertEqual(len(list(Path(folder).iterdir())), 1)
            store.path.write_text('[]')
            with self.assertRaises(ValueError): store.load(config())

    def test_inventory_includes_vendor_tty_names_without_opening(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); sysroot, devroot = root / 'sys', root / 'dev'
            devroot.mkdir()
            for name in ('ttyUSB0', 'ttyCH341USB0', 'ttyCH348USB0'):
                metadata = sysroot / 'class/tty' / name / 'device'
                metadata.mkdir(parents=True)
                for key, value in [('idVendor', '1a86'), ('idProduct', '7523'), ('serial', name)]:
                    (metadata / key).write_text(value)
                (devroot / name).touch()
            devices, _ = inventory(sysroot, devroot)
            self.assertEqual(len(devices), 3)
            self.assertTrue(all(d['candidate'] for d in devices))

    def test_diagnosis_distinguishes_no_serial_and_usb_without_driver(self):
        from car_vel.m701_connection import diagnose
        self.assertEqual(diagnose(config(), [], [])['code'], 'serial_not_found')
        result = diagnose(config(), [], [dict(vid='1a86', pid='7523', product='USB Serial')])
        self.assertEqual(result['code'], 'driver_missing')
        self.assertIn('1a86:7523', result['message'])
        self.assertEqual(result['suggested_port'], '')

    def test_legacy_missing_alias_only_suggests_unique_candidate(self):
        from car_vel.m701_connection import diagnose
        saved = dict(config(mode='manual'), binding=None,
                     port='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        item = device('/dev/ttyCH341USB0'); item['aliases'] = []
        result = diagnose(saved, [item], [])
        self.assertEqual(result['code'], 'selection_required')
        self.assertEqual(result['suggested_port'], item['port'])
        self.assertTrue(result['confirmation_required'])
        with self.assertRaises(ConnectionError):
            resolve(saved, [item])  # A recommendation never authorizes a probe.
        second = device('/dev/ttyCH348USB1', serial='another')
        second['aliases'] = []
        self.assertEqual(diagnose(saved, [item, second], [])['suggested_port'], '')
        self.assertEqual(diagnose(saved, [item, second], [])['code'], 'ambiguous_selection')

    def test_missing_binding_never_recommends_a_different_adapter(self):
        from car_vel.m701_connection import diagnose
        result = diagnose(config(), [device(serial='replacement')], [])
        self.assertEqual(result['code'], 'device_missing')
        self.assertEqual(result['suggested_port'], '')

    def test_manual_path_change_can_recommend_only_the_same_identity(self):
        from car_vel.m701_connection import diagnose
        saved = config(mode='manual')
        same = device('/dev/ttyCH341USB7'); same['aliases'] = []
        self.assertEqual(diagnose(saved, [same], [])['code'], 'path_changed')
        self.assertEqual(diagnose(saved, [same], [])['suggested_port'], same['port'])

    def test_read_only_cli_reports_inventory_and_config_without_ros(self):
        from car_vel import m701_connection
        with patch.object(m701_connection, 'inventory', return_value=([], [])), \
                patch.object(ConfigStore, 'load', return_value=config()), \
                patch('sys.stdout', new_callable=io.StringIO) as output:
            m701_connection.main()
        report = json.loads(output.getvalue())
        self.assertEqual(report['devices'], [])
        self.assertEqual(report['diagnostic']['code'], 'serial_not_found')

    def test_usbfs_claim_is_not_misreported_as_missing_driver(self):
        from car_vel.m701_connection import diagnose
        missing = [dict(vid='1a86', pid='7523', product='USB Serial',
                        interfaces=[dict(driver='usbfs')])]
        result = diagnose(config(), [], missing)
        self.assertEqual(result['code'], 'usb_claimed')
        self.assertIn('usbfs', result['message'])
        self.assertEqual(result['suggested_port'], '')

    def test_inventory_reports_claimed_usb_even_without_any_tty(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); sysroot, devroot = root / 'sys', root / 'dev'
            usb = sysroot / 'bus/usb/devices/1-1.2'
            interface = usb / 'interface0'  # Metadata, not ':' in a Linux-only filename, identifies it.
            interface.mkdir(parents=True)
            for key, value in [('idVendor', '1a86'), ('idProduct', '7523'),
                               ('busnum', '1'), ('devnum', '4')]:
                (usb / key).write_text(value)
            (interface / 'uevent').write_text('DRIVER=usbfs\n')
            (interface / 'bInterfaceNumber').write_text('00')
            devices, missing = inventory(sysroot, devroot)
            self.assertEqual(devices, [])
            self.assertEqual(missing[0]['interfaces'][0]['driver'], 'usbfs')
            self.assertEqual(missing[0]['usb_node'], str(devroot / 'bus/usb/001/004'))


class BridgeTests(unittest.TestCase):
    def setUp(self):
        from test_velocity_arbiter_callbacks import load_module, Node
        interface = types.ModuleType('lzp_interface.srv'); interface.ConfigureM701 = object
        with patch.dict(sys.modules, {'lzp_interface.srv': interface}):
            self.module = load_module('m701_air_quality_bridge.py')
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.store = ConfigStore(Path(self.temp.name) / 'config.json')
        self.devices = [device()]
        self.module.ConfigStore = lambda: self.store
        self.module.inventory = lambda: (self.devices, [])
        self.opens, self.writes = [], []
        outer = self
        class Serial:
            def __init__(self, **kwargs):
                outer.opens.append(kwargs); self.rx = b''; self.closed = False
            def reset_input_buffer(self): self.rx = b''
            def write(self, value):
                outer.writes.append(value)
                self.rx = bytes.fromhex('01 03 0E 01 E2 00 05 00 24 00 2D 00 38 01 31 02 86 A0 0C')
            def flush(self): pass
            @property
            def in_waiting(self): return len(self.rx)
            def read(self, count):
                value, self.rx = self.rx[:count], self.rx[count:]; return value
            def close(self): self.closed = True
        self.module.serial = types.SimpleNamespace(Serial=Serial, EIGHTBITS=8, PARITY_NONE='N', STOPBITS_ONE=1)
        with patch.object(Node, 'create_service', create=True):
            self.node = self.module.M701AirQualityBridge()

    def request(self, operation, **overrides):
        item = self.devices[0]
        payload = dict(mode='auto', port=item['port'], baudrate=9600, address=1,
                       identity=observation(item), confirmed=True)
        payload.update(overrides)
        return self.node._configure(types.SimpleNamespace(operation=operation,
            config_json=json.dumps(payload), expected_version=self.node._config['version']), types.SimpleNamespace())

    def test_listing_never_opens_or_writes(self):
        self.assertTrue(self.request('list').success)
        self.assertEqual(self.opens, []); self.assertEqual(self.writes, [])

    def test_unconfirmed_test_never_opens(self):
        self.assertFalse(self.request('test', confirmed=False).success)
        self.assertEqual(self.opens, [])

    def test_test_restores_config_and_does_not_save(self):
        old = copy.deepcopy(self.node._config)
        self.assertTrue(self.request('test').success)
        self.assertEqual(self.node._config, old)
        self.assertFalse(self.store.path.exists())
        self.assertIsNone(self.node._serial)
        self.assertEqual(len(self.writes), 1)
        self.assertTrue(self.opens[0]['exclusive'])

    def test_save_survives_restart_and_poll_reconnects_same_identity(self):
        self.assertTrue(self.request('save').success)
        self.assertEqual(self.store.load(config())['mode'], 'auto')
        self.devices[0]['port'] = '/dev/ttyCH341USB9'
        self.node._poll()
        self.assertEqual(self.opens[-1]['port'], '/dev/ttyCH341USB9')
        self.assertEqual(self.node._connection_error['code'], 'ok')

    def test_replaced_device_does_not_receive_request(self):
        self.request('save'); self.devices[0]['serial'] = 'another'
        self.node._poll()
        self.assertEqual(self.opens, []); self.assertEqual(self.writes, [])
        self.assertEqual(self.node._connection_error['code'], 'device_missing')

    def test_failed_save_keeps_current_configuration(self):
        old = copy.deepcopy(self.node._config)
        with patch.object(self.store, 'save', side_effect=OSError('read only')):
            self.assertFalse(self.request('save').success)
        self.assertEqual(self.node._config, old)

    def test_stale_version_rejected(self):
        response = self.node._configure(types.SimpleNamespace(operation='save', config_json='{}', expected_version=99), types.SimpleNamespace())
        self.assertFalse(response.success); self.assertEqual(self.opens, [])

    def test_reconnect_resets_warmup(self):
        self.node._poll(); self.assertIsNotNone(self.node._first_opened_at)
        self.node._close_serial(); self.assertIsNone(self.node._first_opened_at)

    def test_error_categories(self):
        for error, code in [(OSError('[Errno 13] Permission denied'), 'permission_denied'),
                            (OSError('Could not exclusively lock port: [Errno 11]'), 'port_busy'),
                            (TimeoutError('no response'), 'response_timeout')]:
            self.node._publish_invalid(error)
            self.assertEqual(self.node._connection_error['code'], code)

    def test_bad_response_has_distinct_status(self):
        self.node._open_serial()
        self.node.response_timeout = 0.01
        self.node._serial.write = lambda value: setattr(self.node._serial, 'rx', b'bad-frame')
        with self.assertRaisesRegex(ConnectionError, 'CRC'):
            self.node._read_response()

    def test_open_race_closes_port_before_any_probe(self):
        self.request('save')
        results = iter([(self.devices, []), ([device(serial='replacement')], [])])
        self.module.inventory = lambda: next(results)
        self.node._poll()
        self.assertEqual(self.writes, [])
        self.assertIsNone(self.node._serial)

    def test_missing_legacy_alias_offers_recovery_without_opening(self):
        self.devices[0]['port'] = '/dev/ttyCH341USB0'
        self.devices[0]['aliases'] = []
        original = copy.deepcopy(self.node._config)
        self.node._poll()
        status = self.node._connection_status()
        self.assertEqual(self.opens, [])
        self.assertEqual(self.writes, [])
        self.assertEqual(self.node._config, original)
        self.assertEqual(status['diagnostic']['code'], 'selection_required')
        self.assertEqual(status['diagnostic']['suggested_port'], '/dev/ttyCH341USB0')
        self.assertIn('设置页', self.node._connection_error['message'])

    def test_confirmed_rebinding_repairs_legacy_path_and_survives_renumber(self):
        self.devices[0]['port'] = '/dev/ttyCH341USB0'
        self.devices[0]['aliases'] = []
        self.node._poll()
        self.assertEqual(self.opens, [])
        self.assertTrue(self.request('test').success)
        self.assertTrue(self.request('save').success)
        self.devices[0]['port'] = '/dev/ttyCH341USB9'
        self.node._poll()
        self.assertEqual(self.node._connection_error['code'], 'ok')
        self.assertEqual(self.opens[-1]['port'], '/dev/ttyCH341USB9')
        self.assertEqual(self.store.load(config())['mode'], 'auto')

    def test_repeated_missing_port_does_not_flood_log_or_fake_data(self):
        self.devices.clear()
        for _ in range(3):
            self.node._poll()
        self.assertEqual(self.node.get_logger().warning.call_count, 1)
        self.assertEqual(self.node.publisher.publish.call_count, 3)
        for call in self.node.publisher.publish.call_args_list:
            payload = json.loads(call.args[0].data)
            self.assertFalse(payload['valid'])
            self.assertIsNone(payload['values'])

    def test_changed_failure_and_recovered_connection_log_immediately(self):
        self.devices.clear()
        self.node._poll()
        self.devices.append(device('/dev/ttyCH341USB0'))
        self.devices[0]['aliases'] = []
        self.node._poll()
        self.assertEqual(self.node.get_logger().warning.call_count, 2)
        self.devices[0]['aliases'] = [self.module.DEFAULT_M701_PORT]
        self.node._poll()
        self.assertEqual(self.node._connection_error['code'], 'ok')
        self.node._close_serial()
        self.devices.clear()
        self.node._poll()
        self.assertEqual(self.node.get_logger().warning.call_count, 3)

    def test_usb_claimed_status_never_opens_or_probes_a_port(self):
        self.module.inventory = lambda: ([], [dict(vid='1a86', pid='7523',
            interfaces=[dict(driver='usbfs')])])
        self.node._poll()
        self.assertEqual(self.node._connection_status()['diagnostic']['code'], 'usb_claimed')
        self.assertIn('usbfs', self.node._connection_error['message'])
        self.assertEqual(self.opens, [])
        self.assertEqual(self.writes, [])


if __name__ == '__main__':
    unittest.main()
