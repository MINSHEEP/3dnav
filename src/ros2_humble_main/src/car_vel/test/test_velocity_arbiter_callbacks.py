"""Exercise production callbacks with ROS transport and time replaced locally."""

import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class Message:
    def __init__(self, data=None):
        self.data = data


class Node:
    def __init__(self, name):
        self.log = Mock()
        self.parameters = {}

    def declare_parameter(self, name, default):
        self.parameters[name] = default
        return types.SimpleNamespace(value=default)

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, *args):
        return Mock()

    def create_subscription(self, *args):
        return Mock()

    def create_timer(self, *args):
        return Mock()

    def get_logger(self):
        return self.log


def load_module(source='velocity_arbiter.py'):
    def module(name, **values):
        result = types.ModuleType(name)
        result.__dict__.update(values)
        return result

    policy = types.SimpleNamespace(
        KEEP_LAST=1, RELIABLE=2, VOLATILE=3, TRANSIENT_LOCAL=4, BEST_EFFORT=5)
    stubs = {
        'rclpy': module('rclpy'),
        'rclpy.node': module('rclpy.node', Node=Node),
        'rclpy.qos': module(
            'rclpy.qos', DurabilityPolicy=policy, HistoryPolicy=policy,
            ReliabilityPolicy=policy, QoSProfile=lambda **kwargs: kwargs),
        'geometry_msgs': module('geometry_msgs'),
        'geometry_msgs.msg': module('geometry_msgs.msg', Twist=Twist),
        'nav_msgs': module('nav_msgs'),
        'nav_msgs.msg': module('nav_msgs.msg', Odometry=object),
        'std_msgs': module('std_msgs'),
        'std_msgs.msg': module(
            'std_msgs.msg', Bool=Message, Empty=Message,
            Float32=Message, String=Message),
        'lzp_interface': module('lzp_interface'),
        'lzp_interface.msg': module('lzp_interface.msg',RouteExecutionStatus=object),
    }
    spec = importlib.util.spec_from_file_location(
        'car_vel._callback_test', ROOT / 'car_vel' / source)
    result = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, stubs):
        spec.loader.exec_module(result)
    return result


class VelocityArbiterCallbacksTest(unittest.TestCase):
    def setUp(self):
        self.module = load_module()
        self.clock = types.SimpleNamespace(monotonic=lambda: self.now)
        self.module.time = self.clock
        self.now = 100.0
        self.node = self.module.VelocityArbiter()
        self.node.get_clock = lambda: types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=int(self.now * 1e9)))
        self.node._lio_pause_callback(Message(False))
        self.node._navigation_pause_callback(Message(False))
        self.body('gangben_zsl1')
        self.mode('terrain_3d')
        self.terrain(True)
        self.node.publisher.reset_mock()

    def mode(self, value):
        self.node._navigation_mode_callback(Message(json.dumps({'mode': value})))

    def terrain(self, safe):
        self.node._terrain_status_callback(Message(json.dumps({
            'safe': safe, 'speed_limit_mps': 0.3,
            'stamp_ns': int(self.now * 1e9), 'navigation_active': True,
            'navigation_mode': 'terrain_3d', 'evidence_source': 'current_anchored',
        })))

    def body(self, name):
        self.node._body_callback(Message(json.dumps({
            'connected': True, 'active_body': name, 'generation': 1,
            'config': {'selected': name, 'profiles': {name: {'navigation_verified': True}}},
            'status': {},
        })))

    def command(self):
        value = Twist()
        value.linear.x = 0.2
        value.angular.z = 0.1
        self.node._nav_callback(value)

    def last_velocity(self):
        value = self.node.publisher.publish.call_args.args[0]
        return value.linear.x, value.angular.z

    def test_repeated_mode_heartbeat_keeps_continuous_output_in_both_modes(self):
        for mode in ('terrain_3d', 'map_2d'):
            self.mode(mode)
            self.node.publisher.reset_mock()
            for tick in range(80):
                self.now += 0.05
                self.node._lio_pause_callback(Message(False))
                self.node._navigation_pause_callback(Message(False))
                self.body(self.node._body_id)
                self.terrain(True)
                self.command()
                if tick % 4 == 0:
                    self.mode(mode)
                self.node._tick()
            commands = self.node.publisher.publish.call_args_list
            self.assertEqual(len(commands), 80)
            self.assertTrue(all(call.args[0].linear.x == 0.2 for call in commands))

    def test_mode_change_stops_and_requires_new_navigation_command(self):
        self.command()
        self.mode('map_2d')
        self.assertEqual(self.last_velocity(), (0.0, 0.0))
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0.0, 0.0))
        self.command()
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0.2, 0.1))
        self.mode('terrain_3d')
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0.0, 0.0))

    def test_duplicate_connector_state_does_not_interrupt_navigation(self):
        self.command()
        self.node._active_callback(Message(False))
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0.2, 0.1))
        self.assertEqual(self.node.publisher.publish.call_count, 1)
        self.node._active_callback(Message(True))
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0.0, 0.0))

    def test_repeated_mode_cannot_refresh_an_expired_command(self):
        self.command()
        self.now += 0.31
        self.mode('terrain_3d')
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0.0, 0.0))

    def test_unsafe_or_stale_terrain_still_stops_navigation(self):
        self.command()
        self.terrain(False)
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0.0, 0.0))
        self.terrain(True)
        self.now += 0.6
        self.command()
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0.0, 0.0))

    def test_emergency_is_not_cleared_by_mode_heartbeat(self):
        self.command()
        self.node._emergency_callback(Message())
        self.mode('terrain_3d')
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0.0, 0.0))

    def test_invalid_mode_discards_cached_command(self):
        self.command()
        self.node._navigation_mode_callback(Message('not-json'))
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0.0, 0.0))

    def test_xingji_3d_loses_global_anchor_stops_but_teleop_still_works(self):
        self.body('xingji')
        self.terrain(True)
        self.command()
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0., 0.))
        status = dict(ready=True, scope='xingji_3d', body_generation=1,
                      stamp_ns=int(self.now*1e9))
        self.node._global_localization_callback(Message(json.dumps(status)))
        self.node._tick()
        self.assertEqual(self.last_velocity(), (.2, .1))
        self.assertEqual(json.loads(self.node._guard_publisher.publish.call_args.args[0].data)['reason_code'], 'forwarded')
        status['ready'] = False
        self.node._global_localization_callback(Message(json.dumps(status)))
        self.node._tick()
        self.assertEqual(self.last_velocity(), (0., 0.))
        self.assertEqual(json.loads(self.node._guard_publisher.publish.call_args.args[0].data)['reason_code'], 'global_localization_untrusted')
        value = Twist(); value.linear.x = .3
        self.node._teleop_callback(value)
        self.node._tick()
        self.assertEqual(self.last_velocity(), (.3, 0.))

    def test_localization_heartbeat_delay_stops_and_requires_new_command_on_return(self):
        self.body('xingji');self.terrain(True)
        status=dict(ready=True,scope='xingji_3d',body_generation=1,stamp_ns=int(self.now*1e9))
        self.node._global_localization_callback(Message(json.dumps(status)))
        self.command();self.node._tick()
        self.assertEqual(self.last_velocity(),(.2,.1))
        self.now+=.4
        self.terrain(True);self.command();self.node._tick()
        self.assertEqual(self.last_velocity(),(0.,0.))
        status['stamp_ns']=int(self.now*1e9)
        self.node._global_localization_callback(Message(json.dumps(status)))
        self.node._tick()
        self.assertEqual(self.last_velocity(),(0.,0.))
        self.command();self.node._tick()
        self.assertEqual(self.last_velocity(),(.2,.1))

    def test_trace_has_raw_and_final_velocity_and_route_point_without_changing_output(self):
        self.node._navigation_active_callback(Message(True))
        self.node._route_diagnostic_callback(types.SimpleNamespace(
            route_id=7,route_name='route',map_name='room',point_index=0,
            point_count=21,current_loop=1,state='navigating',mode='route'))
        self.command();self.node._tick()
        self.assertEqual(self.last_velocity(),(.2,.1))
        record=self.node._motion_trace.last_record
        self.assertEqual(record['input'],[.2,0,.1])
        self.assertEqual(record['output'],[.2,0,.1])
        self.assertEqual(record['route']['point_number'],1)
        self.assertEqual(record['reason_code'],'forwarded')
        self.node._motion_trace.emit=Mock(side_effect=RuntimeError('sink failed'))
        self.command();self.node._tick()
        self.assertEqual(self.last_velocity(),(.2,.1),'diagnostic failure must not affect control')

    def test_trace_explains_zero_input_guard_and_observer_error_without_changing_gates(self):
        self.node._navigation_active_callback(Message(True))
        self.node._nav_callback(Twist());self.node._tick()
        self.assertEqual(self.node._motion_trace.last_record['reason_code'],'upstream_zero')
        self.now+=.6
        self.node._lio_pause_callback(Message(False))
        self.node._navigation_pause_callback(Message(False))
        self.node._observer_diagnostic_callback(Message(json.dumps({
            'state':'error','message':'transform timed out'})))
        self.terrain(False);self.command();self.node._tick()
        record=self.node._motion_trace.last_record
        self.assertEqual(record['reason_code'],'terrain_unsafe')
        self.assertEqual(record['observer']['message'],'transform timed out')
        self.assertEqual(self.last_velocity(),(0.,0.))
        previous=dict(self.node._diag_observer)
        self.node._observer_diagnostic_callback(Message('not json'))
        self.assertEqual(self.node._diag_observer,previous)
        self.assertFalse(self.node._terrain_safe)

    def test_trace_records_return_coverage_and_does_not_replay_discarded_input(self):
        self.node._observer_diagnostic_callback(Message(json.dumps({
            'state':'ready','corridor_return_count':20,'near_field_return_count':0,
            'nearest_forward_return_m':1.2})))
        self.command();self.mode('map_2d');self.node._tick()
        record=self.node._motion_trace.last_record
        self.assertEqual(record['reason_code'],'upstream_discarded')
        self.assertEqual(record['observer']['near_field_return_count'],0)
        self.assertEqual(self.last_velocity(),(0.,0.))


if __name__ == '__main__':
    unittest.main()
