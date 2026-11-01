"""Run real bridge callbacks through a fake clock and UDP socket."""

import json
import math
import types
import unittest
from unittest.mock import Mock

from test_velocity_arbiter_callbacks import load_module, Twist


class TurnContinuityTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.module = load_module('cmdvel_bridge_ros2.py')
        self.module.time = types.SimpleNamespace(
            monotonic=lambda: self.now, sleep=lambda _: None)
        self.socket = Mock()
        self.module.socket = types.SimpleNamespace(
            AF_INET=2, SOCK_DGRAM=2, socket=lambda *_: self.socket)
        self.node = self.module.CmdVelBridgeNode()
        self.socket.reset_mock()

    def tick(self, x=0.0, w=0.08, *, receive=True, dt=0.05):
        self.now += dt
        if receive:
            message = Twist()
            message.linear.x = x
            message.angular.z = w
            self.node.cmdvel_callback(message)
        self.node.timer_callback()
        payload = json.loads(self.socket.sendto.call_args.args[0])
        return payload['vel_move_frontback'], payload['vel_turn_leftright']

    def warm(self):
        for _ in range(12):
            output = self.tick()
        self.assertAlmostEqual(output[1], 0.15)

    def test_turn_floor_is_reached_gradually_at_udp_boundary(self):
        values = [self.tick()[1] for _ in range(12)]
        self.assertLessEqual(values[0], 0.020001)
        self.assertAlmostEqual(values[-1], 0.15)
        self.assertTrue(all(0 <= b-a <= 0.020001 for a,b in zip(values, values[1:])))

    def test_small_nonzero_jitter_does_not_alternate_stop_and_turn(self):
        values = [self.tick(w=w)[1] for w in [0.021, 0.019, 0.022, 0.018]*5]
        self.assertTrue(all(w > 0 for w in values))
        self.assertTrue(all(abs(b-a) <= 0.020001 for a,b in zip(values,values[1:])))

    def test_translation_boundary_does_not_jump_turn_speed(self):
        self.warm()
        values = [0.15] + [self.tick(x=x, w=0.04)[1] for x in [0.012,0.014]*10]
        self.assertTrue(all(abs(b-a) <= 0.020001 for a,b in zip(values,values[1:])))
        for _ in range(15):
            vx, yaw = self.tick(x=0.05, w=0.04)
        self.assertAlmostEqual(vx, 0.075)
        self.assertAlmostEqual(yaw, 0.04)

    def test_reversing_turn_does_not_flip_in_one_tick(self):
        self.warm()
        values = [0.15] + [self.tick(w=-0.08)[1] for _ in range(20)]
        self.assertTrue(all(abs(b-a) <= 0.020001 for a,b in zip(values,values[1:])))
        self.assertAlmostEqual(values[-1], -0.15)

    def test_explicit_zero_stops_and_clears_turn_memory(self):
        self.warm()
        self.assertEqual(self.tick(w=0), (0, 0))
        self.assertEqual(self.tick(w=0.018), (0, 0))
        self.assertLessEqual(self.tick()[1], 0.020001)

    def test_timeout_stops_and_restart_ramps_from_zero(self):
        self.warm()
        self.assertEqual(self.tick(receive=False, dt=0.31), (0, 0))
        self.assertLessEqual(self.tick()[1], 0.020001)

    def test_emergency_and_release_never_replay_previous_turn(self):
        self.warm()
        self.node.emergency_stop_callback(None)
        self.assertEqual(self.tick(), (0, 0))
        self.node.clear_emergency_stop_callback(None)
        self.assertEqual(self.tick(receive=False), (0, 0))
        self.assertLessEqual(self.tick()[1], 0.020001)

    def test_invalid_command_stops_instead_of_saturating(self):
        for invalid in (math.nan, math.inf, -math.inf):
            self.warm()
            self.assertEqual(self.tick(w=invalid), (0, 0))

    def test_subthreshold_noise_cannot_start_a_turn_and_release_stops(self):
        for _ in range(10):
            self.assertEqual(self.tick(w=0.019), (0, 0))
        self.warm()
        self.assertEqual(self.tick(w=0.009), (0, 0))
        self.assertEqual(self.tick(w=0.019), (0, 0))

    def test_callback_bursts_do_not_accelerate_the_output_ramp(self):
        for _ in range(30):
            message = Twist()
            message.angular.z = 0.08
            self.node.cmdvel_callback(message)
        self.assertLessEqual(self.tick(receive=False)[1], 0.020001)

    def test_scheduler_stall_and_fresh_command_do_not_reuse_old_turn(self):
        self.warm()
        # The next command can arrive before the watchdog timer runs again.
        self.assertLessEqual(self.tick(dt=1.0)[1], 0.020001)

    def test_zero_between_output_ticks_resets_ramp(self):
        self.warm()
        self.node.cmdvel_callback(Twist())
        self.assertLessEqual(self.tick()[1], 0.020001)


if __name__ == '__main__':
    unittest.main()
