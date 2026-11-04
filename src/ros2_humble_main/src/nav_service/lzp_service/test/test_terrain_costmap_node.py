#!/usr/bin/env python3
"""Boundary tests for the selectable Nav2 costmap adapter."""

import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
NODE_SOURCE = PACKAGE_ROOT / 'lzp_service' / 'terrain_costmap_node.py'


class _Stamp:
    sec = 4
    nanosec = 2


class _Header:
    def __init__(self):
        self.frame_id = 'map'
        self.stamp = _Stamp()


class _Value:
    def __init__(self, **values):
        self.__dict__.update(values)


class _Info:
    def __init__(self):
        self.resolution = 1.0
        self.width = 2
        self.height = 1
        self.map_load_time = _Stamp()
        self.origin = _Value(
            position=_Value(x=0.0, y=0.0),
            orientation=_Value(x=0.0, y=0.0, z=0.0, w=1.0),
        )


class _OccupancyGrid:
    def __init__(self):
        self.header = _Header()
        self.info = _Info()
        self.data = []


class _TerrainGrid:
    def __init__(self, map_name='garden', map_load_id='load-1'):
        self.header = _Header()
        self.map_name = map_name
        self.map_load_id = map_load_id
        self.resolution = 1.0
        self.width = 2
        self.height = 1
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.traversal_cost = [0, 0]


class _CurrentMap:
    def __init__(self, success=True, map_name='garden', message='',
                 load_id='load-1'):
        self.success = success
        self.map_name = map_name
        self.message = message
        self.load_id = load_id


class _GetMap:
    class Request:
        pass


class _Future:
    def __init__(self):
        self._callbacks = []
        self._response = None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        return self._response

    def complete(self, map_message):
        self._response = _Value(map=map_message)
        for callback in self._callbacks:
            callback(self)


class _Client:
    def __init__(self):
        self.futures = []
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, _request):
        future = _Future()
        self.futures.append(future)
        return future


class _String:
    def __init__(self, data=''):
        self.data = data


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


class _Now:
    def to_msg(self):
        return _Stamp()


class _Node:
    def __init__(self, _name):
        self.map_client = _Client()

    def declare_parameter(self, _name, default):
        return _Parameter(default)

    def create_publisher(self, *_args, **_kwargs):
        return _Publisher()

    def create_subscription(self, *_args, **_kwargs):
        return object()

    def create_client(self, *_args, **_kwargs):
        return self.map_client

    def create_timer(self, *_args, **_kwargs):
        return object()

    def get_clock(self):
        return _Value(now=lambda: _Now())

    def get_logger(self):
        return _Logger()


class _QoSProfile:
    def __init__(self, **_kwargs):
        pass


class _Policy:
    KEEP_LAST = object()
    RELIABLE = object()
    TRANSIENT_LOCAL = object()


def _module(name, **values):
    output = types.ModuleType(name)
    for key, value in values.items():
        setattr(output, key, value)
    return output


def _load_node_module():
    stubs = {
        'rclpy': _module(
            'rclpy', init=lambda args=None: None, spin=lambda node: None,
            shutdown=lambda: None),
        'rclpy.node': _module('rclpy.node', Node=_Node),
        'rclpy.qos': _module(
            'rclpy.qos', DurabilityPolicy=_Policy,
            HistoryPolicy=_Policy, QoSProfile=_QoSProfile,
            ReliabilityPolicy=_Policy),
        'nav_msgs': _module('nav_msgs'),
        'nav_msgs.msg': _module(
            'nav_msgs.msg', OccupancyGrid=_OccupancyGrid),
        'nav_msgs.srv': _module('nav_msgs.srv', GetMap=_GetMap),
        'std_msgs': _module('std_msgs'),
        'std_msgs.msg': _module('std_msgs.msg', String=_String),
        'lzp_interface': _module('lzp_interface'),
        'lzp_interface.msg': _module(
            'lzp_interface.msg', CurrentMap=_CurrentMap,
            TerrainGrid=_TerrainGrid),
    }
    previous = {name: sys.modules.get(name) for name in stubs}
    sys.modules.update(stubs)
    name = 'lzp_service._terrain_costmap_node_boundary_test'
    try:
        spec = importlib.util.spec_from_file_location(name, NODE_SOURCE)
        loaded = importlib.util.module_from_spec(spec)
        sys.modules[name] = loaded
        spec.loader.exec_module(loaded)
        return loaded
    finally:
        sys.modules.pop(name, None)
        for module_name, old in previous.items():
            if old is None:
                sys.modules.pop(module_name, None)
            else:
                sys.modules[module_name] = old


def _base():
    value = _OccupancyGrid()
    value.data = [0, 100]
    return value


class TerrainCostmapNodeTests(unittest.TestCase):

    def test_2d_mode_publishes_base_map_without_terrain(self):
        node = _load_node_module().TerrainCostmapNode()
        node._map_callback(_CurrentMap())
        node._mode_callback(_String(json.dumps({'mode': 'map_2d'})))
        node._base_callback(_base())

        self.assertEqual(node.publisher.messages[-1].data, [0, 100])
        status = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(status['state'], 'ready')
        self.assertEqual(status['navigation_mode'], 'map_2d')
        self.assertEqual(status['source'], 'raw_map')

    def test_3d_mode_waits_for_matching_terrain_then_fuses(self):
        node = _load_node_module().TerrainCostmapNode()
        node._map_callback(_CurrentMap())
        node._base_callback(_base())
        self.assertFalse(node.publisher.messages)
        waiting = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(waiting['state'], 'waiting')

        node._terrain_callback(_TerrainGrid())
        self.assertFalse(node.publisher.messages)
        node.map_client.futures[-1].complete(_base())
        self.assertEqual(node.publisher.messages[-1].data, [0, 100])
        ready = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(ready['navigation_mode'], 'terrain_3d')
        self.assertEqual(ready['source'], 'terrain_fused')
        self.assertEqual(ready['map_load_id'], 'load-1')

    def test_map_change_invalidates_both_old_inputs_before_new_fusion(self):
        node = _load_node_module().TerrainCostmapNode()
        node._map_callback(_CurrentMap(map_name='garden'))
        node._base_callback(_base())
        node._terrain_callback(_TerrainGrid(map_name='garden'))
        node.map_client.futures[-1].complete(_base())
        published_before_switch = len(node.publisher.messages)

        # DDS does not guarantee callback order across /current_map_info,
        # /map and /terrain_grid.  A new terrain grid arriving first must not
        # be fused with the previous map's still-cached OccupancyGrid.
        node._map_callback(_CurrentMap(
            map_name='warehouse', load_id='load-warehouse'))
        self.assertIsNone(node._base_map)
        self.assertIsNone(node._terrain_grid)
        node._terrain_callback(_TerrainGrid(
            map_name='warehouse', map_load_id='load-warehouse'))
        self.assertEqual(published_before_switch, len(node.publisher.messages))

        node._base_callback(_base())
        self.assertEqual(published_before_switch, len(node.publisher.messages))
        node.map_client.futures[-1].complete(_base())
        self.assertEqual(
            published_before_switch + 1, len(node.publisher.messages))
        ready = json.loads(node.status_publisher.messages[-1].data)
        self.assertEqual(ready['state'], 'ready')
        self.assertEqual(ready['map_name'], 'warehouse')

    def test_same_name_confirmed_reload_requires_authoritative_map(self):
        node = _load_node_module().TerrainCostmapNode()
        node._map_callback(_CurrentMap())
        old_map = _base()
        node._base_callback(old_map)
        old_terrain = _TerrainGrid()
        node._terrain_callback(old_terrain)
        node.map_client.futures[-1].complete(_base())
        published_before_reload = len(node.publisher.messages)

        node._map_callback(_CurrentMap(
            message='Successfully loaded and confirmed map: garden',
            load_id='load-2'))
        self.assertIsNone(node._base_map)
        self.assertIsNone(node._terrain_grid)
        self.assertEqual(published_before_reload, len(node.publisher.messages))

        # A delayed transient-local /map sample and old terrain sample cannot
        # establish the new LoadMap generation, even after the event callback.
        node._base_callback(old_map)
        node._terrain_callback(old_terrain)
        self.assertEqual(published_before_reload, len(node.publisher.messages))

        fresh_terrain = _TerrainGrid(map_load_id='load-2')
        node._terrain_callback(fresh_terrain)
        self.assertEqual(published_before_reload, len(node.publisher.messages))

        live_map = _base()
        node.map_client.futures[-1].complete(live_map)
        self.assertEqual(published_before_reload + 1, len(node.publisher.messages))
        self.assertEqual(node._base_map, live_map)

        # The periodic CurrentMap heartbeat is not a new load generation.
        node._map_callback(_CurrentMap(
            message='Current map: garden', load_id='load-2'))
        self.assertEqual(node._base_map, live_map)

    def test_same_name_reload_leaves_2d_topic_behavior_unchanged(self):
        node = _load_node_module().TerrainCostmapNode()
        node._mode_callback(_String(json.dumps({'mode': 'map_2d'})))
        node._map_callback(_CurrentMap())
        node._base_callback(_base())
        original_count = len(node.publisher.messages)

        node._map_callback(_CurrentMap(
            message='Successfully loaded and confirmed map: garden',
            load_id='load-2'))
        self.assertEqual(original_count, len(node.publisher.messages))
        replacement = _base()
        replacement.data = [100, 0]
        node._base_callback(replacement)
        self.assertEqual(node.publisher.messages[-1].data, [100, 0])
        self.assertFalse(node.map_client.futures)

        # Returning to 3D must not treat the 2D topic cache as verified.
        published_in_2d = len(node.publisher.messages)
        node._mode_callback(_String(json.dumps({'mode': 'terrain_3d'})))
        fresh_terrain = _TerrainGrid(map_load_id='load-2')
        node._terrain_callback(fresh_terrain)
        self.assertEqual(published_in_2d, len(node.publisher.messages))
        self.assertEqual(len(node.map_client.futures), 1)

    def test_get_map_unavailable_or_invalid_keeps_3d_waiting(self):
        node = _load_node_module().TerrainCostmapNode()
        node._map_callback(_CurrentMap())
        node._base_callback(_base())
        node._terrain_callback(_TerrainGrid())
        node.map_client.futures[-1].complete(_base())
        published_before_reload = len(node.publisher.messages)
        node.map_client.ready = False

        node._map_callback(_CurrentMap(
            message='Successfully loaded map: garden', load_id='load-2'))
        fresh_terrain = _TerrainGrid(map_load_id='load-2')
        node._terrain_callback(fresh_terrain)
        self.assertEqual(len(node.map_client.futures), 1)
        self.assertEqual(published_before_reload, len(node.publisher.messages))

        node.map_client.ready = True
        node._request_authoritative_map()
        invalid = _base()
        invalid.data = [0]
        node.map_client.futures[-1].complete(invalid)
        self.assertEqual(published_before_reload, len(node.publisher.messages))
        self.assertIsNone(node._base_map)

    def test_new_generation_terrain_may_arrive_before_current_map_event(self):
        node = _load_node_module().TerrainCostmapNode()
        node._map_callback(_CurrentMap())
        node._terrain_callback(_TerrainGrid())
        node.map_client.futures[-1].complete(_base())
        old_published = len(node.publisher.messages)

        early = _TerrainGrid(map_load_id='load-2')
        node._terrain_callback(early)
        self.assertEqual(
            json.loads(node.status_publisher.messages[-1].data)['state'],
            'waiting')
        self.assertIsNone(node._base_map)
        node._map_callback(_CurrentMap(
            load_id='load-2',
            message='Successfully loaded and confirmed map: garden'))
        self.assertIs(node._terrain_grid, early)
        self.assertEqual(old_published, len(node.publisher.messages))
        node.map_client.futures[-1].complete(_base())
        self.assertEqual(old_published + 1, len(node.publisher.messages))

        # A stale latched terrain update must not overwrite the new one.
        node._terrain_callback(_TerrainGrid(map_load_id='load-1'))
        self.assertIs(node._terrain_grid, early)
        self.assertEqual(old_published + 1, len(node.publisher.messages))

    def test_late_response_from_earlier_reload_is_ignored(self):
        node = _load_node_module().TerrainCostmapNode()
        node._map_callback(_CurrentMap())
        node._base_callback(_base())
        node._terrain_callback(_TerrainGrid())
        node.map_client.futures[-1].complete(_base())
        published_before_reload = len(node.publisher.messages)

        node._map_callback(_CurrentMap(
            message='Successfully loaded and confirmed map: garden',
            load_id='load-2'))
        previous_future = node.map_client.futures[-1]
        node._map_callback(_CurrentMap(
            message='Successfully loaded and confirmed map: garden',
            load_id='load-3'))
        current_future = node.map_client.futures[-1]
        self.assertIsNot(previous_future, current_future)

        previous_future.complete(_base())
        self.assertIsNone(node._base_map)
        fresh_terrain = _TerrainGrid(map_load_id='load-3')
        node._terrain_callback(fresh_terrain)
        self.assertEqual(published_before_reload, len(node.publisher.messages))
        current_future.complete(_base())
        self.assertEqual(published_before_reload + 1, len(node.publisher.messages))

    def test_retracted_terrain_generation_cannot_be_resurrected(self):
        node = _load_node_module().TerrainCostmapNode()
        node._map_callback(_CurrentMap())
        node._terrain_callback(_TerrainGrid())
        node.map_client.futures[-1].complete(_base())
        ready_count = len(node.publisher.messages)

        # A late, unbound status from startup cannot revoke an unrelated
        # confirmed map generation.
        node._terrain_status_callback(_String(json.dumps({
            'state': 'unavailable', 'map_name': '', 'map_load_id': ''})))
        self.assertEqual(node._terrain_invalid_load_id, '')
        self.assertEqual(ready_count, len(node.publisher.messages))

        node._terrain_status_callback(_String(json.dumps({
            'state': 'error', 'map_name': 'garden',
            'map_load_id': 'load-1',
            'message': 'saved terrain map rejected'})))
        self.assertEqual(
            json.loads(node.status_publisher.messages[-1].data)['state'],
            'waiting')
        node._terrain_callback(_TerrainGrid(map_load_id='load-1'))
        self.assertEqual(ready_count, len(node.publisher.messages))
        node._mode_callback(_String(json.dumps({'mode': 'map_2d'})))
        node._terrain_status_callback(_String(json.dumps({
            'state': 'ready', 'map_name': 'garden',
            'map_load_id': 'load-1'})))
        self.assertEqual(node._terrain_invalid_load_id, 'load-1')
        node._mode_callback(_String(json.dumps({'mode': 'terrain_3d'})))
        node._terrain_callback(_TerrainGrid(map_load_id='load-1'))
        self.assertEqual(ready_count, len(node.publisher.messages))
        node._map_callback(_CurrentMap(
            map_name='garden', load_id='load-1',
            message='Current map: garden'))
        self.assertEqual(ready_count, len(node.publisher.messages))

        node._map_callback(_CurrentMap(
            map_name='garden', load_id='load-2',
            message='Successfully loaded and confirmed map: garden'))
        node._terrain_callback(_TerrainGrid(map_load_id='load-2'))
        node.map_client.futures[-1].complete(_base())
        self.assertEqual(ready_count + 1, len(node.publisher.messages))

    def test_terrain_status_has_no_effect_on_active_2d_map(self):
        node = _load_node_module().TerrainCostmapNode()
        node._mode_callback(_String(json.dumps({'mode': 'map_2d'})))
        node._map_callback(_CurrentMap())
        node._base_callback(_base())
        ready_count = len(node.publisher.messages)
        node._terrain_status_callback(_String(json.dumps({
            'state': 'unavailable', 'map_name': 'garden',
            'map_load_id': 'load-1'})))
        self.assertEqual(ready_count, len(node.publisher.messages))
        node._base_callback(_base())
        self.assertEqual(ready_count + 1, len(node.publisher.messages))
        self.assertEqual(
            json.loads(node.status_publisher.messages[-1].data)['source'],
            'raw_map')


if __name__ == '__main__':
    unittest.main()
