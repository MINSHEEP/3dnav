"""Persisted switch and production launch callbacks, with subprocess/ROS mocked."""
import importlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import test_launch_manager_fastdds_preflight as fixtures

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class MappingFilterSettingsTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / 'mapping_filter.json'
        self.store_type = importlib.import_module('lzp_service.mapping_filter').MappingFilterStore

    def store(self, default='protected'):
        return self.store_type(self.path, default_mode=default)

    def manager(self):
        fixture = fixtures.TestFastDdsPreflight('runTest')
        fixture.setUp()
        node = fixture.manager
        node._mapping_filter_store = self.store()
        node._active_mapping_filter_mode = None
        node._mapping_filter_message_type = lambda **kw: SimpleNamespace(**kw)
        node._mapping_filter_status_publisher = Mock()
        return node, fixture.module

    def save(self, node, enabled):
        return node._set_mapping_filter_enabled_callback(SimpleNamespace(data=enabled), SimpleNamespace())

    def start(self, node, module, mode='mapping'):
        commands = []
        def popen(command, **_kwargs):
            commands.append(command)
            return fixtures._Process()
        with patch.object(module.subprocess, 'Popen', side_effect=popen), patch.object(
                module.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, 'ok')):
            success, message = node._start_mode(mode)
        self.assertTrue(success, message)
        return commands

    def test_default_and_explicit_startup_mode_are_preserved_without_file(self):
        self.assertEqual(self.store().snapshot()['mode'], 'protected')
        self.assertEqual(self.store('bypass').snapshot()['mode'], 'bypass')
        self.assertEqual(self.store('strict').snapshot()['mode'], 'strict')
        self.assertFalse(self.path.exists())

    def test_save_is_persisted_across_backend_restart(self):
        state = self.store().set_enabled(False)
        self.assertEqual(state['mode'], 'bypass')
        self.assertFalse(state['enabled'])
        self.assertEqual(self.store().snapshot()['mode'], 'bypass')
        self.assertEqual(self.store().set_enabled(True)['mode'], 'protected')

    def test_saving_unchanged_default_still_pins_user_preference(self):
        state = self.store().set_enabled(True)
        self.assertTrue(self.path.is_file())
        self.assertEqual(state['version'], 1)
        self.assertTrue(self.store('bypass').snapshot()['enabled'])
        self.assertEqual(self.store().set_enabled(True)['version'], 1)

    def test_enabled_strict_is_not_silently_downgraded(self):
        self.assertEqual(self.store('strict').set_enabled(True)['mode'], 'strict')
        store = self.store()
        store.set_enabled(False)
        self.assertEqual(store.set_enabled(True)['mode'], 'protected')

    def test_corruption_fails_to_protected_and_cannot_be_silently_overwritten(self):
        self.path.write_text('{broken', encoding='utf-8')
        store = self.store('bypass')
        state = store.snapshot()
        self.assertEqual(state['mode'], 'protected')
        self.assertTrue(state['load_error'])
        with self.assertRaises(ValueError):
            store.set_enabled(False)
        self.assertEqual(self.path.read_text(encoding='utf-8'), '{broken')

    def test_type_and_write_failure_do_not_change_saved_state(self):
        store = self.store()
        for value in (0, 1, 'false', None):
            with self.subTest(value=value), self.assertRaises((TypeError, ValueError)):
                store.set_enabled(value)
        before = store.snapshot()
        with patch('lzp_service.mapping_filter.os.replace', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                store.set_enabled(False)
        self.assertEqual(store.snapshot(), before)
        self.assertFalse(self.path.exists())
        self.assertEqual(list(self.path.parent.glob('*.tmp')), [])

    def test_unreadable_configuration_is_visible_without_crashing_backend(self):
        with patch('lzp_service.mapping_filter.Path.exists', side_effect=PermissionError('denied')):
            store = self.store('bypass')
        self.assertEqual(store.snapshot()['mode'], 'protected')
        self.assertIn('denied', store.snapshot()['load_error'])

    def test_schema_rejects_boolean_and_unknown_versions(self):
        for schema in (True, 1.0, 2, None):
            with self.subTest(schema=schema):
                self.path.write_text(json.dumps(dict(schema_version=schema,version=1,
                    mode='bypass',enabled=False)),encoding='utf-8')
                self.assertTrue(self.store().snapshot()['load_error'])

    def test_production_settings_initialization_wires_standard_services(self):
        node, _module = self.manager()
        del node._mapping_filter_store
        values = {'mapping_filter_config_path': str(self.path), 'mapping_dynamic_filter_mode': 'strict'}
        node.get_parameter = lambda name: SimpleNamespace(value=values[name])
        node.create_publisher = Mock()
        node.create_service = Mock()
        def module(name, **values):
            result = ModuleType(name); result.__dict__.update(values); return result
        message_type = type('String', (), {'__init__': lambda self, **kw: self.__dict__.update(kw)})
        set_bool, trigger = object(), object()
        with patch.dict(sys.modules, {
            'rclpy.qos': module('rclpy.qos',QoSProfile=lambda **kw:kw,
                               DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1)),
            'std_msgs': module('std_msgs'), 'std_msgs.msg':module('std_msgs.msg',String=message_type),
            'std_srvs':module('std_srvs'), 'std_srvs.srv':module('std_srvs.srv',SetBool=set_bool,Trigger=trigger),
        }):
            node._init_mapping_filter_settings()
        self.assertEqual(node._mapping_dynamic_filter_mode(), 'strict')
        services = {call.args[1]: call.args[0] for call in node.create_service.call_args_list}
        self.assertIs(services['/mapping/filter/set_enabled'], set_bool)
        self.assertIs(services['/mapping/filter/get'], trigger)
        self.assertEqual(node.create_publisher.call_args.args[1], '/mapping/filter/status')
        response = node._get_mapping_filter_callback(None, SimpleNamespace())
        self.assertTrue(response.success)
        self.assertEqual(json.loads(response.message)['mode'], 'strict')

    def test_settings_callback_saves_without_start_stop_or_body_commands(self):
        node, module = self.manager()
        with patch.object(module.subprocess, 'Popen') as popen, patch.object(
                node, '_stop_all_modes') as stop:
            result = self.save(node, False)
        self.assertTrue(result.success)
        state = json.loads(result.message)
        self.assertEqual(state['mode'], 'bypass')
        self.assertFalse(state['mapping_running'])
        self.assertIsNone(state['active_mode'])
        popen.assert_not_called()
        stop.assert_not_called()
        self.assertEqual(json.loads(node._mapping_filter_status_publisher.publish.call_args.args[0].data)['mode'], 'bypass')

    def test_saved_mode_is_in_actual_mapping_launch_command_not_navigation(self):
        node, module = self.manager()
        self.save(node, False)
        command = self.start(node, module)[0]
        self.assertIn('dynamic_filter_mode:=bypass', command)
        state = node._mapping_filter_status()
        self.assertEqual(state['active_mode'], 'bypass')
        self.assertFalse(state['pending'])
        node.running_processes.clear()
        command = self.start(node, module, 'car_vel')[0]
        self.assertFalse(any(value.startswith('dynamic_filter_mode:=') for value in command))

    def test_running_mapping_keeps_its_mode_until_next_start(self):
        node, module = self.manager()
        self.start(node, module)
        with patch.object(module.subprocess, 'Popen') as popen:
            result = self.save(node, False)
            self.assertTrue(result.success)
            state = json.loads(result.message)
            self.assertEqual(state['active_mode'], 'protected')
            self.assertEqual(state['mode'], 'bypass')
            self.assertTrue(state['pending'])
            self.assertTrue(node._start_mode('mapping')[0])
            popen.assert_not_called()
        node.running_processes.clear()
        self.assertIn('dynamic_filter_mode:=bypass', self.start(node, module)[0])

    def test_dead_mapping_is_not_reported_as_active_after_save(self):
        node, module = self.manager()
        self.start(node, module)
        node.running_processes['mapping'] = SimpleNamespace(poll=lambda: 1)
        state = node._mapping_filter_status()
        self.assertFalse(state['mapping_running'])
        self.assertIsNone(state['active_mode'])
        self.assertFalse(state['pending'])

    def test_invalid_callback_input_and_disk_failure_are_visible(self):
        node, _module = self.manager()
        self.assertFalse(self.save(node, 'false').success)
        with patch('lzp_service.mapping_filter.os.replace', side_effect=OSError('disk full')):
            result = self.save(node, False)
        self.assertFalse(result.success)
        self.assertIn('disk full', result.message)
        self.assertEqual(node._mapping_dynamic_filter_mode(), 'protected')


if __name__ == '__main__':
    unittest.main()
