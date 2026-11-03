"""Exercise real arbiter callbacks, with robot identity and command direction."""
import json
from pathlib import Path
import runpy
import unittest
from types import SimpleNamespace as NS

runtime = runpy.run_path(str(Path(__file__).with_name('test_velocity_arbiter_callbacks.py')))
Base = runtime['VelocityArbiterCallbacksTest']
Message = runtime['Message']
Twist = runtime['Twist']


class DirectionGuardTests(Base):
    def confirm_localization(self):
        self.node._global_localization_callback(Message(json.dumps({
            'ready': True, 'scope': 'xingji_3d', 'body_generation': 1,
            'stamp_ns': int(self.now * 1e9),
        })))

    def body(self, name='xingji'):
        self.node._body_callback(Message(json.dumps({
            'connected': True, 'active_body': name, 'generation': 1,
            'config': {'selected': name, 'profiles': {name: {'navigation_verified': True}}},
            'status': {},
        })))

    def test_front_floor_never_authorizes_xingji_reverse_or_sideways(self):
        self.body()
        self.confirm_localization()
        self.terrain(True)
        for x, y in ((-.2, 0.), (0., .2)):
            command = Twist()
            command.linear.x = x; command.linear.y = y
            self.node._nav_callback(command)
            self.node._tick()
            value = self.node.publisher.publish.call_args.args[0]
            self.assertEqual((value.linear.x, value.linear.y, value.angular.z), (0., 0., 0.))
            self.assertIn('禁止自动倒退或横移', self.node._guard_reason)

    def test_2d_and_confirmed_gangben_keep_reverse_behavior(self):
        for name, mode in (('xingji', 'map_2d'), ('gangben_zsl1', 'terrain_3d')):
            self.body(name); self.mode(mode); self.terrain(True)
            command = Twist(); command.linear.x = -.2
            self.node._nav_callback(command); self.node._tick()
            self.assertEqual(self.node.publisher.publish.call_args.args[0].linear.x, -.2)

    def test_stale_body_and_unanchored_evidence_do_not_enable_xingji_motion(self):
        self.body();self.terrain(True)
        self.confirm_localization()
        self.node._terrain_evidence='current'
        self.command();self.node._tick()
        self.assertEqual(self.last_velocity(),(0.,0.))
        self.node._terrain_evidence='current_anchored'
        self.now+=2.1
        self.confirm_localization()
        self.command();self.node._tick()
        self.assertEqual(self.last_velocity(),(0.,0.))
        self.assertIn('身份',self.node._guard_reason)

    def test_xingji_forward_and_slow_turn_remain_continuous(self):
        self.body()
        for _ in range(30):
            self.now+=.05;self.body();self.confirm_localization()
            self.terrain(True);self.command();self.node._tick()
            self.assertEqual(self.last_velocity(),(.2,.1))


if __name__ == '__main__':
    unittest.main()
