#!/usr/bin/env python3
"""Safety and scheduling contracts for host power control."""

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.system_power import (  # noqa: E402
    POWER_COMMANDS,
    SystemPowerController,
)


class FakeRunner:
    def __init__(self, permission_returncode=0, execution_returncode=0):
        self.calls = []
        self.permission_returncode = permission_returncode
        self.execution_returncode = execution_returncode

    def __call__(self, argv, **kwargs):
        self.calls.append((tuple(argv), kwargs))
        returncode = (
            self.permission_returncode
            if '-l' in argv else self.execution_returncode
        )
        return SimpleNamespace(returncode=returncode, stdout='', stderr='denied')


class SystemPowerControllerTests(unittest.TestCase):
    def make_controller(self, runner=None, enabled=True):
        scheduled = []
        controller = SystemPowerController(
            enabled=enabled,
            delay_seconds=3.0,
            run_command=runner or FakeRunner(),
            schedule=lambda delay, callback: scheduled.append((delay, callback)),
            path_exists=lambda path: path in {'/usr/bin/sudo', '/usr/bin/systemctl'},
        )
        return controller, scheduled

    def test_only_fixed_absolute_commands_are_exposed(self):
        self.assertEqual(
            POWER_COMMANDS,
            {
                'reboot': (
                    '/usr/bin/sudo', '-n', '/usr/bin/systemctl', 'reboot'),
                'poweroff': (
                    '/usr/bin/sudo', '-n', '/usr/bin/systemctl', 'poweroff'),
            },
        )

    def test_disabled_and_unknown_actions_never_execute(self):
        runner = FakeRunner()
        disabled, scheduled = self.make_controller(runner, enabled=False)
        success, _message = disabled.request('reboot')
        self.assertFalse(success)
        self.assertEqual(runner.calls, [])
        self.assertEqual(scheduled, [])

        enabled, scheduled = self.make_controller(runner)
        success, _message = enabled.request('reboot; rm -rf /')
        self.assertFalse(success)
        self.assertEqual(runner.calls, [])
        self.assertEqual(scheduled, [])

    def test_permission_denial_is_reported_before_scheduling(self):
        runner = FakeRunner(permission_returncode=1)
        controller, scheduled = self.make_controller(runner)
        success, message = controller.request('poweroff')
        self.assertFalse(success)
        self.assertIn('权限', message)
        self.assertEqual(scheduled, [])
        self.assertEqual(
            runner.calls[0][0],
            ('/usr/bin/sudo', '-n', '-l', '/usr/bin/systemctl', 'poweroff'),
        )

    def test_accepts_once_then_executes_after_delay_without_shell(self):
        runner = FakeRunner()
        controller, scheduled = self.make_controller(runner)

        success, message = controller.request('reboot')
        self.assertTrue(success)
        self.assertIn('3', message)
        self.assertEqual(len(scheduled), 1)
        self.assertEqual(scheduled[0][0], 3.0)

        duplicate_success, _message = controller.request('poweroff')
        self.assertFalse(duplicate_success)
        self.assertEqual(len(scheduled), 1)

        scheduled[0][1]()
        self.assertEqual(runner.calls[-1][0], POWER_COMMANDS['reboot'])
        self.assertNotIn('shell', runner.calls[-1][1])
        self.assertFalse(controller.status()['pending'])


if __name__ == '__main__':
    unittest.main()
