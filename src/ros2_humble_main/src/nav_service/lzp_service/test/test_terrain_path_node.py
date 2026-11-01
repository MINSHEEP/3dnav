#!/usr/bin/env python3
"""Boundary tests for the display-only terrain path ROS adapter."""

import importlib.util
import json
import math
from pathlib import Path as FilePath
import sys
import types
import unittest


PACKAGE_ROOT = FilePath(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
NODE_SOURCE = PACKAGE_ROOT / 'lzp_service' / 'terrain_path_node.py'


class _Stamp:
    sec = 1
    nanosec = 0


class _Header:
    def __init__(self):
        self.frame_id = ''
        self.stamp = _Stamp()


class _Position:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class _Pose:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.position = _Position(x, y, z)


class _PoseStamped:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.header = _Header()
        self.pose = _Pose(x, y, z)


class _Path:
    def __init__(self):
        self.header = _Header()
        self.poses = []


class _String:
    def __init__(self, data=''):
        self.data = data


class _Bool:
    def __init__(self, data=False):
        self.data = data


class _CurrentMap:
    def __init__(self, success=False, map_name=''):
        self.success = success
        self.message = ''
        self.map_name = map_name


class _TerrainGrid:
    def __init__(self, map_name=''):
        self.header = _Header()
        self.map_name = map_name
        self.resolution = 1.0
        self.width = 0
        self.height = 0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.elevation = []
        self.traversal_cost = []


class _Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class _Now:
    def to_msg(self):
        return _Stamp()


class _Clock:
    def now(self):
        return _Now()


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Node:
    def __init__(self, _name):
        self.publishers = {}

    def declare_parameter(self, _name, default):
        return _Parameter(default)

    def create_publisher(self, _message_type, topic, _qos):
        publisher = _Publisher(topic)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, *_args, **_kwargs):
        return object()

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Logger()


class _QoSProfile:
    def __init__(self, **_kwargs):
        pass


class _Policy:
    KEEP_LAST = object()
    RELIABLE = object()
    VOLATILE = object()
    TRANSIENT_LOCAL = object()


def _fake_module(name, **values):
    module = types.ModuleType(name)
    for key, value in values.items():
        setattr(module, key, value)
    return module


def _load_node_module():
    rclpy = _fake_module(
        'rclpy', init=lambda args=None: None, spin=lambda node: None,
        shutdown=lambda: None)
    stubs = {
        'rclpy': rclpy,
        'rclpy.node': _fake_module('rclpy.node', Node=_Node),
        'rclpy.qos': _fake_module(
            'rclpy.qos',
            DurabilityPolicy=_Policy,
            HistoryPolicy=_Policy,
            QoSProfile=_QoSProfile,
            ReliabilityPolicy=_Policy,
        ),
        'nav_msgs': _fake_module('nav_msgs'),
        'nav_msgs.msg': _fake_module('nav_msgs.msg', Path=_Path),
        'std_msgs': _fake_module('std_msgs'),
        'std_msgs.msg': _fake_module(
            'std_msgs.msg', Bool=_Bool, String=_String),
        'lzp_interface': _fake_module('lzp_interface'),
        'lzp_interface.msg': _fake_module(
            'lzp_interface.msg',
            CurrentMap=_CurrentMap,
            TerrainGrid=_TerrainGrid,
        ),
    }
    previous = {name: sys.modules.get(name) for name in stubs}
    sys.modules.update(stubs)
    module_name = 'lzp_service._terrain_path_node_boundary_test'
    try:
        specification = importlib.util.spec_from_file_location(
            module_name, NODE_SOURCE)
        module = importlib.util.module_from_spec(specification)
        sys.modules[module_name] = module
        specification.loader.exec_module(module)
        return module
    finally:
        sys.modules.pop(module_name, None)
        for name, old_module in previous.items():
            if old_module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old_module


def _plan():
    message = _Path()
    message.header.frame_id = 'map'
    message.poses = [
        _PoseStamped(0.2, 0.2, 0.0),
        _PoseStamped(1.2, 0.2, 0.0),
    ]
    return message


def _terrain(map_name='garden'):
    message = _TerrainGrid(map_name)
    message.header.frame_id = 'map'
    message.width = 2
    message.height = 1
    message.elevation = [1.0, 2.0]
    message.traversal_cost = [0, 0]
    return message


class TerrainPathNodeBoundaryTest(unittest.TestCase):

    def test_2d_mode_forwards_raw_plan_without_waiting_for_terrain(self):
        module = _load_node_module()
        node = module.TerrainPathNode()
        node._mode_callback(_String(json.dumps({'mode': 'map_2d'})))
        node._map_callback(_CurrentMap(True, 'garden'))
        node._active_callback(_Bool(True))

        node._plan_callback(_plan())

        displayed = node.path_publisher.messages[-1]
        self.assertEqual(len(displayed.poses), 2)
        self.assertEqual(displayed.header.frame_id, 'map')
        self.assertEqual(displayed.poses[0].pose.position.z, 0.0)
        status = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(status['state'], 'ready')
        self.assertEqual(status['navigation_mode'], 'map_2d')
        self.assertFalse(status['lifted'])
        self.assertFalse(status['terrain_ready'])

    def test_plan_waits_for_terrain_then_republishes_with_elevation(self):
        module = _load_node_module()
        node = module.TerrainPathNode()
        node._map_callback(_CurrentMap(True, 'garden'))
        node._active_callback(_Bool(True))

        source_plan = _plan()
        node._plan_callback(source_plan)
        self.assertEqual(node.path_publisher.messages[-1].poses, [])
        waiting = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(waiting['state'], 'waiting')

        node._terrain_callback(_terrain())
        displayed = node.path_publisher.messages[-1]
        self.assertEqual(len(displayed.poses), 2)
        # TerrainGrid elevation is float32 on the wire; compare at a precision
        # that reflects that representation instead of Python float64.
        self.assertAlmostEqual(displayed.poses[0].pose.position.z, 1.08, places=6)
        self.assertAlmostEqual(displayed.poses[1].pose.position.z, 2.08, places=6)
        ready = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(ready['state'], 'ready')
        self.assertEqual(ready['source_pose_count'], 2)
        self.assertEqual(ready['display_pose_count'], 2)
        self.assertEqual(ready['fallback_count'], 0)
        self.assertAlmostEqual(ready['length_m'], math.sqrt(2.0), places=6)

    def test_invalid_same_map_grid_clears_old_elevation_cache(self):
        module = _load_node_module()
        node = module.TerrainPathNode()
        node._map_callback(_CurrentMap(True, 'garden'))
        node._terrain_callback(_terrain())
        node._active_callback(_Bool(True))
        node._plan_callback(_plan())
        self.assertEqual(len(node.path_publisher.messages[-1].poses), 2)

        invalid = _terrain()
        invalid.elevation = [9.0]
        node._terrain_callback(invalid)
        self.assertIsNone(node._terrain_data_cache)
        self.assertEqual(node.path_publisher.messages[-1].poses, [])
        rejected = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(rejected['state'], 'error')

        node._plan_callback(_plan())
        self.assertEqual(node.path_publisher.messages[-1].poses, [])
        waiting = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(waiting['state'], 'waiting')

        node._terrain_callback(_terrain())
        self.assertEqual(len(node.path_publisher.messages[-1].poses), 2)
        recovered = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(recovered['state'], 'ready')

    def test_invalid_pre_map_grid_clears_same_map_preloaded_cache(self):
        module = _load_node_module()
        node = module.TerrainPathNode()
        node._terrain_callback(_terrain())
        self.assertIsNotNone(node._terrain_data_cache)

        invalid = _terrain()
        invalid.traversal_cost = [0]
        node._terrain_callback(invalid)
        self.assertIsNone(node._terrain_data_cache)

        node._map_callback(_CurrentMap(True, 'garden'))
        node._active_callback(_Bool(True))
        node._plan_callback(_plan())
        self.assertEqual(node.path_publisher.messages[-1].poses, [])
        waiting = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(waiting['state'], 'waiting')

    def test_map_switch_and_inactive_state_clear_the_latched_path(self):
        module = _load_node_module()
        node = module.TerrainPathNode()
        node._map_callback(_CurrentMap(True, 'garden'))
        node._terrain_callback(_terrain())
        node._active_callback(_Bool(True))
        node._plan_callback(_plan())
        self.assertEqual(len(node.path_publisher.messages[-1].poses), 2)

        node._map_callback(_CurrentMap(True, 'courtyard'))
        self.assertEqual(node.path_publisher.messages[-1].poses, [])
        switched = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(switched['state'], 'waiting')
        self.assertEqual(switched['map_name'], 'courtyard')
        node._terrain_callback(_terrain('courtyard'))
        self.assertEqual(node.path_publisher.messages[-1].poses, [])

        node._active_callback(_Bool(False))
        self.assertEqual(node.path_publisher.messages[-1].poses, [])
        inactive = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(inactive['state'], 'inactive')


if __name__ == '__main__':
    unittest.main()
