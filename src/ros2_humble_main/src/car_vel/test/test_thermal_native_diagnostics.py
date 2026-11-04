"""Native SDK evidence is captured without auto-restarting or opening USB."""
import ast
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

MODULE_DIR = Path(__file__).resolve().parents[1] / 'hikvision_camera'
sys.path.insert(0, str(MODULE_DIR))
import thermal_web


class ThermalNativeDiagnosticsTests(unittest.TestCase):
    def test_enables_all_python_thread_stacks(self):
        with patch.object(thermal_web, 'faulthandler', create=True) as faults:
            thermal_web.enable_native_diagnostics()
            faults.enable.assert_called_once_with(all_threads=True)

    def test_unavailable_fault_output_does_not_prevent_capture(self):
        faults = types.SimpleNamespace(enable=Mock(side_effect=RuntimeError('no fileno')))
        with patch.object(thermal_web, 'faulthandler', faults, create=True), \
                patch.object(thermal_web.sys, 'stderr', io.StringIO()):
            thermal_web.enable_native_diagnostics()

    def test_sdk_enumeration_failure_retains_phase_in_log(self):
        class Sdk:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                pass

            def enumerate_devices(self):
                raise RuntimeError('simulated enumeration failure')

        demo = types.SimpleNamespace(HCUsbSDK=lambda _path: Sdk())
        args = thermal_web.parse_args(['--bundle-dir', str(MODULE_DIR)])
        producer = thermal_web.HCUsbProducer(Mock(), args)
        output = io.StringIO()
        with patch.object(thermal_web, 'load_vendor_demo', return_value=demo), \
                patch.object(thermal_web.sys, 'stderr', output):
            with self.assertRaisesRegex(RuntimeError, 'enumeration failure'):
                producer._run_session()
        self.assertIn('phase=initializing_sdk', output.getvalue())
        self.assertIn('phase=enumerating_devices', output.getvalue())

    def test_fault_handler_is_enabled_before_native_capture_thread_starts(self):
        tree = ast.parse((MODULE_DIR / 'thermal_web.py').read_text(encoding='utf-8'))
        main = next(item for item in tree.body
                    if isinstance(item, ast.FunctionDef) and item.name == 'main')
        statements = [ast.unparse(item) for item in main.body]
        enabled = next(index for index, text in enumerate(statements)
                       if text.startswith('enable_native_diagnostics()'))
        started = next(index for index, text in enumerate(statements)
                       if text.startswith('producer.start()'))
        self.assertLess(enabled, started)
        self.assertNotIn('respawn', ast.unparse(main))


if __name__ == '__main__':
    unittest.main()
