import sys
import os
import threading
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'hikvision_camera'))
from thermal_recovery import probe_usb, RetryBudget, IsolatedThermalProducer
from thermal_monitor import ThermalMonitor, ThermalConfigStore
from test_thermal_monitor import sample
import thermal_web


def exit_worker(args, connection, stop, threshold):
    os._exit(6)  # abrupt native-like exit bypasses all Python cleanup


def frame_worker(args, connection, stop, threshold):
    import time
    connection.send(('state', {'running': True}))
    connection.send(('sample', sample(25, epoch=time.time())))
    connection.close()


class RecoveryTests(unittest.TestCase):
    def test_capture_is_invalidated_if_session_dies_during_window(self):
        with tempfile.TemporaryDirectory() as root:
            monitor = ThermalMonitor(ThermalConfigStore(Path(root) / 'config.json'))
            monitor.set_backend_state(running=True)
            monitor.ingest(sample(80))
            waiting = threading.Event()
            original_wait = monitor._condition.wait
            errors = []
            def wait(timeout):
                waiting.set()
                return original_wait(timeout)
            def capture():
                try:
                    monitor.capture(2)
                except RuntimeError as error:
                    errors.append(str(error))
            with patch.object(monitor._condition, 'wait', side_effect=wait):
                thread = threading.Thread(target=capture)
                thread.start()
                self.assertTrue(waiting.wait(1))
                monitor.set_backend_state(running=False, error='device removed')
                thread.join(1)
                self.assertFalse(thread.is_alive())
            self.assertEqual(len(errors), 1)
            self.assertIn('抓拍作废', errors[0])

    def test_supervisor_never_initializes_sdk_without_usb(self):
        with tempfile.TemporaryDirectory() as root:
            monitor = ThermalMonitor(ThermalConfigStore(Path(root) / 'config.json'))
            args = thermal_web.parse_args(['--bundle-dir', root])
            adapter = Path(root) / 'PythonDemo' / 'thermal_max_cv.py'
            adapter.parent.mkdir()
            adapter.write_text('# test only')
            producer = IsolatedThermalProducer(monitor, args, probe=lambda: (None, 'USB尚未识别'))
            with patch.object(producer, '_session', side_effect=AssertionError('must wait')), \
                    patch.object(producer._stop, 'wait', side_effect=lambda _delay: producer._stop.set()):
                producer._run()
            self.assertFalse(monitor.status()['ready'])

    def test_abrupt_child_exit_leaves_parent_usable_and_next_session_can_ingest(self):
        with tempfile.TemporaryDirectory() as root:
            monitor = ThermalMonitor(ThermalConfigStore(Path(root) / 'config.json'))
            args = thermal_web.parse_args([])
            producer = IsolatedThermalProducer(monitor, args, probe=lambda: ('A', ''),
                                               worker=exit_worker)
            _, error = producer._session('A')
            self.assertIn('6', error)
            self.assertFalse(monitor.status()['ready'])
            producer.worker = frame_worker
            producer._session('A')
            self.assertEqual(monitor.status()['frame_sequence'], 1)
            self.assertFalse(monitor.status()['ready'])
            self.assertIsNone(monitor.status()['max_temperature_c'])

    def test_no_enumeration_reports_waiting_without_touching_sdk(self):
        with tempfile.TemporaryDirectory() as root:
            token, error = probe_usb(Path(root))
        self.assertIsNone(token)
        self.assertIn('USB尚未识别', error)

    def test_correct_identity_permissions_and_new_device_generation(self):
        with tempfile.TemporaryDirectory() as root:
            p = Path(root) / '1-2'
            p.mkdir()
            for name, value in dict(idVendor='2bdf', idProduct='0102',
                                    busnum='1', devnum='25', serial='ABC').items():
                (p / name).write_text(value)
            with patch('thermal_recovery.os.access', return_value=False):
                self.assertIn('权限', probe_usb(Path(root))[1])
            with patch('thermal_recovery.os.access', return_value=True):
                first, error = probe_usb(Path(root), serial='ABC')
                self.assertEqual(error, '')
                self.assertIsNone(probe_usb(Path(root), serial='other')[0])
                (p / 'devnum').write_text('26')
                self.assertNotEqual(first, probe_usb(Path(root), serial='ABC')[0])

    def test_failures_are_bounded_and_replug_releases_budget(self):
        budget = RetryBudget(limit=2, delay=3)
        self.assertTrue(budget.allowed('A', 0))
        budget.failed(0)
        self.assertFalse(budget.allowed('A', 2))
        self.assertTrue(budget.allowed('A', 3))
        budget.failed(3)
        self.assertFalse(budget.allowed('A', 300))
        self.assertTrue(budget.allowed('B', 301))

    def test_backend_failure_removes_old_frame_and_alarm_but_keeps_mode(self):
        with tempfile.TemporaryDirectory() as root:
            monitor = ThermalMonitor(ThermalConfigStore(Path(root) / 'config.json'),
                                     hold_seconds=0, wall_clock=lambda: 100)
            monitor.set_mode('inspection', 'task-1')
            monitor.set_backend_state(running=True)
            monitor.ingest(sample(80))
            monitor.set_backend_state(running=False, error='worker exited -6')
            status = monitor.status()
            self.assertFalse(status['ready'])
            self.assertFalse(status['alarm_active'])
            self.assertIsNone(status['max_temperature_c'])
            self.assertEqual(status['task_id'], 'task-1')
            with self.assertRaises(RuntimeError):
                monitor.snapshot()


if __name__ == '__main__':
    unittest.main()
