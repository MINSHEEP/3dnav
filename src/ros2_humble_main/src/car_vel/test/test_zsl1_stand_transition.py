"""Actual ZSL-1 callbacks with deterministic time/ROS and native I/O doubles."""
import json
from types import SimpleNamespace
import unittest

import test_robot_body_selection as support


class StandTransitionTests(unittest.TestCase):
    def setUp(self):
        self.fixture = support.Zsl1RosCallbacksTests()
        self.fixture.setUp()
        self.node, self.sdk = self.fixture.node, self.fixture.sdk

    def advance(self, seconds):
        for _ in range(round(seconds / 0.05)):
            self.fixture.now += 0.05
            self.node.heartbeat(None)
            self.node.tick()

    def test_wait_does_not_authorize_or_send_move(self):
        self.node.stand()
        self.assertFalse(self.node.motion_authorized)
        self.advance(3.9)
        self.assertFalse(self.node.motion_authorized)
        self.advance(0.2)
        self.assertTrue(self.node.motion_authorized)
        self.assertEqual([name for name, _ in self.sdk.calls], ['init', 'standUp'])

    def test_frontend_zero_burst_and_early_motion_are_not_replayed(self):
        self.node.enable(SimpleNamespace(data=True))
        self.node.stand()
        zero = self.fixture.twist(); zero.linear.x = zero.angular.z = 0
        for _ in range(3):
            self.node.remote_velocity(zero)
            self.node.remote_velocity(self.fixture.twist())
            self.advance(0.05)
        self.advance(4)
        self.assertEqual([name for name, _ in self.sdk.calls], ['init', 'standUp'])
        self.assertEqual(self.node.command_at, 0)
        self.node.remote_velocity(self.fixture.twist())
        self.advance(0.1)
        self.assertTrue(any(name == 'move' for name, _ in self.sdk.calls))

    def test_cached_mode_does_not_skip_explicit_request(self):
        self.sdk.mode = 1
        self.node.stand()
        self.assertIn(('standUp', ()), self.sdk.calls)
        self.assertFalse(self.node.motion_authorized)

    def test_busy_move_is_not_called_by_stand_only(self):
        self.sdk.move = lambda *args: self.fail('Stand-only must not enter Move')
        self.node.stand(); self.advance(5)
        self.assertFalse(self.node.adapter.emergency)
        self.assertTrue(self.node.motion_authorized)

    def test_unstable_feedback_extends_wait(self):
        self.node.stand(); self.sdk.velocity = [0.1, 0, 0]
        self.advance(4.1)
        self.assertFalse(self.node.motion_authorized)
        self.sdk.velocity = [0, 0, 0]
        self.advance(0.3)
        self.assertFalse(self.node.motion_authorized)
        self.advance(0.3)
        self.assertTrue(self.node.motion_authorized)

    def test_emergency_cancels_wait_no_later_authorization(self):
        future = self.node.stand()
        self.node.emergency(); self.advance(5)
        self.assertFalse(self.node.motion_authorized)
        self.assertTrue(future.done())
        self.assertFalse(future.result()[0])

    def test_service_does_not_return_success_before_wait(self):
        coroutine = self.node.ensure_standing(None, SimpleNamespace())
        coroutine.send(None)
        self.assertFalse(self.node.motion_authorized)
        self.advance(4.1)
        with self.assertRaises(StopIteration) as result:
            coroutine.send(None)
        self.assertTrue(result.exception.value.success)

    def test_timeout_latches_and_requests_only_zero_move_without_passive(self):
        future = self.node.stand(); self.sdk.mode = 51
        self.advance(8.1)
        self.assertFalse(future.result()[0])
        self.assertTrue(self.node.adapter.emergency)
        moves = [args for name, args in self.sdk.calls if name == 'move']
        self.assertEqual(moves, [(0.0, 0.0, 0.0)])
        self.assertFalse(any(name == 'passive' for name, _ in self.sdk.calls))

    def test_duplicate_stand_does_not_restart_timer(self):
        first = self.node.stand(); self.advance(2)
        self.assertIs(first, self.node.stand())
        self.advance(2.1)
        self.assertTrue(self.node.motion_authorized)
        self.assertEqual(sum(name == 'standUp' for name, _ in self.sdk.calls), 1)

    def test_bad_or_rotating_feedback_does_not_authorize(self):
        for gyro in ([float('nan'), 0, 0], [0, 0, 0.2], []):
            with self.subTest(gyro=gyro):
                self.setUp()
                self.node.stand(); self.sdk.gyro = gyro
                self.advance(4.1)
                self.assertFalse(self.node.motion_authorized)

    def test_remote_lease_loss_cancels_pending_service(self):
        self.node.enable(SimpleNamespace(data=True))
        future = self.node.stand()
        self.fixture.now += 0.5; self.node.tick()
        self.assertFalse(future.result()[0])
        self.advance(5)
        self.assertFalse(self.node.motion_authorized)

    def test_disconnect_cancels_and_reconnect_never_resumes(self):
        future = self.node.stand(); self.sdk.connected = False
        self.advance(0.1)
        self.assertFalse(future.result()[0])
        self.sdk.connected = True; self.advance(5)
        self.assertFalse(self.node.motion_authorized)

    def test_nonzero_move_failure_is_latched_with_one_zero_attempt(self):
        self.node.stand(); self.advance(4.1)
        def busy(*args):
            self.sdk.calls.append(('move', args)); return 0x3007
        self.sdk.move = busy
        self.node.nav_velocity(self.fixture.twist()); self.advance(0.1)
        self.assertTrue(self.node.adapter.emergency)
        self.assertFalse(self.node.motion_authorized)
        self.assertEqual(sum(name == 'move' for name, _ in self.sdk.calls), 2)
        self.assertEqual(self.sdk.calls[-1], ('move', (0.0, 0.0, 0.0)))
        self.assertFalse(any(name == 'passive' for name, _ in self.sdk.calls))
        with self.assertRaisesRegex(RuntimeError, '0x3007'):
            self.node.stand()

    def test_explicit_stop_cancels_pending_stand(self):
        future = self.node.stand()
        self.node.command(SimpleNamespace(data=json.dumps({'command': 'stop'})))
        self.advance(5)
        self.assertFalse(future.result()[0])
        self.assertFalse(self.node.motion_authorized)
        self.assertFalse(any(name == 'move' for name, _ in self.sdk.calls))

    def test_initial_move_mode_is_not_reported_as_standing(self):
        self.sdk.mode = 18
        with self.assertRaisesRegex(RuntimeError, '不是可站立模式'):
            self.node.stand()
        self.assertFalse(self.node.motion_authorized)

    def test_mode_loss_requires_new_stand(self):
        self.node.stand(); self.advance(4.1)
        self.sdk.mode = 0; self.advance(0.1)
        self.sdk.mode = 1; self.advance(1)
        self.assertFalse(self.node.motion_authorized)

    def test_low_battery_cancels_wait(self):
        future = self.node.stand()
        self.sdk.getBatteryPower = lambda: 10
        self.advance(0.1)
        self.assertFalse(future.result()[0])
        self.assertFalse(self.node.motion_authorized)

    def test_page_exit_cancels_pending_stand(self):
        self.node.enable(SimpleNamespace(data=True))
        future = self.node.stand()
        self.node.enable(SimpleNamespace(data=False)); self.advance(5)
        self.assertFalse(future.result()[0])
        self.assertFalse(self.node.motion_authorized)

    def test_scheduler_gap_cannot_count_as_stable_feedback(self):
        self.node.stand(); self.advance(0.1)
        self.fixture.now += 4; self.node.tick()
        self.assertFalse(self.node.motion_authorized)
        self.advance(0.6)
        self.assertTrue(self.node.motion_authorized)


if __name__ == '__main__':
    unittest.main()
