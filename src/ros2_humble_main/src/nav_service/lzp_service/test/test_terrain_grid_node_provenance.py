#!/usr/bin/env python3
"""Exercise terrain-grid provenance without importing ROS on the test host."""

import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from tempfile import TemporaryDirectory


NODE_SOURCE = (
    Path(__file__).resolve().parents[1] / 'lzp_service' /
    'terrain_grid_node.py'
)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Cost:
    def __init__(self, values):
        self.values = values

    def __ge__(self, value):
        return [item >= value for item in self.values]

    def __eq__(self, value):
        return [item == value for item in self.values]

    def tolist(self):
        return self.values


class _GridMessage:
    def __init__(self):
        self.header = SimpleNamespace(frame_id='', stamp=None)


def _test_node_class():
    """Compile the real node methods with tiny message/publisher stand-ins."""

    module = ast.parse(NODE_SOURCE.read_text(encoding='utf-8'))
    source_class = next(
        item for item in module.body
        if isinstance(item, ast.ClassDef) and item.name == 'TerrainGridNode'
    )
    names = {
        '_map_callback', '_invalidate_map', '_publish_grid',
        '_publish_grid_data', '_publish_error',
    }
    selected = [
        item for item in source_class.body
        if isinstance(item, ast.FunctionDef) and item.name in names
    ]
    assert len(selected) == len(names)
    test_module = ast.Module(body=[ast.ClassDef(
        name='TestTerrainGridNode', bases=[], keywords=[],
        body=selected, decorator_list=[],
    )], type_ignores=[])
    ast.fix_missing_locations(test_module)

    class _String:
        data = ''

    environment = {
        'json': json,
        'Path': Path,
        'np': SimpleNamespace(count_nonzero=lambda values: sum(values)),
        'String': _String,
        'TerrainGrid': _GridMessage,
        'yaml': SimpleNamespace(safe_load=json.load),
        'terrain_artifact_signature': lambda path: (
            path.stat().st_mtime_ns, path.stat().st_size),
    }
    exec(compile(test_module, str(NODE_SOURCE), 'exec'), environment)
    return environment['TestTerrainGridNode'], environment


def _grid():
    layer = SimpleNamespace(tolist=lambda: [0.0])
    return SimpleNamespace(
        resolution=0.25, width=1, height=1, origin_x=0.0,
        origin_y=0.0, elevation=layer, slope_deg=layer,
        roughness_m=layer, step_height_m=layer, confidence=layer,
        traversal_cost=_Cost([0]),
    )


def _node(node_class, directory):
    node = node_class()
    node.saved_pcd_directory = str(directory)
    node.current_map_name = ''
    node.current_map_load_id = ''
    node._rejected_map_generations = set()
    node.loaded_artifact_signature = None
    node._ground_load_id = ''
    node.publisher = _Publisher()
    node.status_publisher = _Publisher()
    node.get_logger = lambda: SimpleNamespace(warning=lambda _: None)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: SimpleNamespace(sec=123, nanosec=456)))
    return node


def _status(node):
    return json.loads(node.status_publisher.messages[-1].data)


class TerrainGridNodeProvenanceTest(unittest.TestCase):
    def test_runtime_grid_defaults_to_unverified(self):
        node_class, _ = _test_node_class()
        node = _node(node_class, '')
        node._publish_grid_data(_grid(), frame_id='map', source='/cloud')
        self.assertEqual(_status(node)['state'], 'ready')
        self.assertFalse(_status(node)['artifact_verified'])
        self.assertEqual(_status(node)['map_load_id'], '')
        self.assertEqual(_status(node)['source_pcd_sha256'], '')
        self.assertEqual(_status(node)['terrain_sha256'], '')

    def test_bound_artifact_status_is_paired_with_exact_grid_stamp(self):
        node_class, environment = _test_node_class()
        with TemporaryDirectory() as directory:
            map_name = 'verified'
            metadata = {'terrain_grid': {
                'source_pcd_sha256': 'a' * 64,
                'sha256': 'b' * 64,
            }}
            root = Path(directory)
            (root / f'{map_name}.localization.yaml').write_text(
                json.dumps(metadata), encoding='utf-8')
            (root / f'{map_name}.terrain.npz').write_bytes(b'test')
            environment['load_bound_terrain_grid'] = lambda *args, **kwargs: (
                SimpleNamespace(grid=_grid(), frame_id='map',
                                generation={'algorithm': 'test'}))
            node = _node(node_class, root)
            current = SimpleNamespace(
                success=True, map_name=map_name, load_id='uuid-a')
            node._map_callback(current)
            status = _status(node)
            self.assertEqual(status['state'], 'ready', status)
            published = node.publisher.messages[-1]
            self.assertTrue(status['artifact_verified'])
            self.assertEqual(status['map_name'], map_name)
            self.assertEqual(status['map_load_id'], 'uuid-a')
            self.assertEqual(published.map_load_id, 'uuid-a')
            self.assertEqual(status['source_pcd_sha256'], 'a' * 64)
            self.assertEqual(status['terrain_sha256'], 'b' * 64)
            self.assertEqual(status['stamp_ns'], 123_000_000_456)
            self.assertEqual(
                status['stamp_ns'],
                published.header.stamp.sec * 1_000_000_000
                + published.header.stamp.nanosec)
            status_count = len(node.status_publisher.messages)
            node._map_callback(current)
            self.assertEqual(len(node.status_publisher.messages), status_count)
            current.load_id = 'uuid-b'
            node._map_callback(current)
            self.assertEqual(len(node.publisher.messages), 2)
            self.assertEqual(node.publisher.messages[-1].map_load_id, 'uuid-b')
            self.assertEqual(_status(node)['map_load_id'], 'uuid-b')

    def test_map_switch_failed_reload_and_unload_revoke_provenance(self):
        node_class, environment = _test_node_class()
        with TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'first.localization.yaml').write_text(json.dumps({
                'terrain_grid': {
                    'source_pcd_sha256': 'a' * 64,
                    'sha256': 'b' * 64,
                },
            }), encoding='utf-8')
            (root / 'first.terrain.npz').write_bytes(b'test')
            environment['load_bound_terrain_grid'] = lambda *args, **kwargs: (
                SimpleNamespace(grid=_grid(), frame_id='map', generation={}))
            node = _node(node_class, root)
            node._map_callback(SimpleNamespace(
                success=True, map_name='first', load_id='uuid-a'))
            self.assertTrue(_status(node)['artifact_verified'], _status(node))
            (root / 'first.terrain.npz').write_bytes(b'changed')

            unexpected_loads = []

            def reject_artifact(*args, **kwargs):
                unexpected_loads.append(True)
                raise ValueError('digest mismatch')

            environment['load_bound_terrain_grid'] = reject_artifact
            node._map_callback(SimpleNamespace(
                success=True, map_name='first', load_id='uuid-a'))
            self.assertEqual(_status(node)['state'], 'unavailable')
            self.assertEqual(_status(node)['map_name'], 'first')
            self.assertEqual(_status(node)['map_load_id'], 'uuid-a')
            self.assertIn('new LoadMap', _status(node)['message'])
            self.assertFalse(_status(node)['artifact_verified'])
            self.assertEqual(node.current_map_name, '')
            self.assertEqual(unexpected_loads, [])
            status_count = len(node.status_publisher.messages)
            node._map_callback(SimpleNamespace(
                success=True, map_name='first', load_id='uuid-a'))
            self.assertEqual(len(node.status_publisher.messages), status_count)
            self.assertEqual(unexpected_loads, [])
            environment['load_bound_terrain_grid'] = lambda *args, **kwargs: (
                SimpleNamespace(grid=_grid(), frame_id='map', generation={}))
            node._map_callback(SimpleNamespace(
                success=True, map_name='first', load_id='uuid-b'))
            self.assertTrue(_status(node)['artifact_verified'])
            node._map_callback(SimpleNamespace(
                success=True, map_name='missing', load_id='uuid-c'))
            self.assertEqual(_status(node)['state'], 'error')
            self.assertEqual(_status(node)['map_name'], 'missing')
            self.assertEqual(_status(node)['map_load_id'], 'uuid-c')
            self.assertFalse(_status(node)['artifact_verified'])
            self.assertEqual(node.current_map_name, '')
            self.assertIsNone(node.loaded_artifact_signature)
            node._map_callback(SimpleNamespace(success=False, map_name='first'))
            self.assertEqual(_status(node)['state'], 'unavailable')
            self.assertFalse(_status(node)['artifact_verified'])

    def test_legacy_pcd_reconstruction_is_not_verified(self):
        node_class, environment = _test_node_class()
        with TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'legacy.localization.yaml').write_text('{}',
                                                          encoding='utf-8')
            (root / 'legacy.pcd').write_bytes(b'test')
            environment['load_pcd_xyz'] = lambda _: [[0.0, 0.0, 0.0]]
            environment['build_terrain_grid'] = lambda *args, **kwargs: _grid()
            node = _node(node_class, root)
            node.resolution = 0.25
            node.maximum_cells = 10
            node.caution_slope_deg = 12
            node.special_slope_deg = 18
            node.blocked_slope_deg = 28
            node.caution_step_m = 0.08
            node.special_step_m = 0.12
            node.blocked_step_m = 0.18
            node.roughness_caution_m = 0.05
            node.minimum_confidence = 0.35
            node._map_callback(SimpleNamespace(
                success=True, map_name='legacy', load_id='uuid-a'))
            self.assertEqual(_status(node)['state'], 'ready', _status(node))
            self.assertEqual(node.publisher.messages[-1].map_load_id, 'uuid-a')
            self.assertFalse(_status(node)['artifact_verified'])
            self.assertEqual(_status(node)['source_pcd_sha256'], '')

    def test_metadata_mutation_requires_fresh_load_id(self):
        node_class, environment = _test_node_class()
        with TemporaryDirectory() as directory:
            root = Path(directory)
            metadata_path = root / 'first.localization.yaml'
            metadata = {'terrain_grid': {
                'source_pcd_sha256': 'a' * 64,
                'sha256': 'b' * 64,
            }}
            metadata_path.write_text(json.dumps(metadata), encoding='utf-8')
            (root / 'first.terrain.npz').write_bytes(b'test')
            loads = []

            def load_bound(*args, **kwargs):
                loads.append(True)
                return SimpleNamespace(
                    grid=_grid(), frame_id='map', generation={})

            environment['load_bound_terrain_grid'] = load_bound
            node = _node(node_class, root)
            current = SimpleNamespace(
                success=True, map_name='first', load_id='uuid-a')
            node._map_callback(current)
            self.assertEqual(len(loads), 1)
            self.assertEqual(len(node.publisher.messages), 1)

            metadata['edited_after_load'] = 'different file signature'
            metadata_path.write_text(json.dumps(metadata), encoding='utf-8')
            node._map_callback(current)
            self.assertEqual(_status(node)['state'], 'unavailable')
            self.assertEqual(_status(node)['map_name'], 'first')
            self.assertEqual(_status(node)['map_load_id'], 'uuid-a')
            self.assertEqual(len(loads), 1)
            self.assertEqual(len(node.publisher.messages), 1)
            node._map_callback(current)
            self.assertEqual(len(loads), 1)
            self.assertEqual(len(node.publisher.messages), 1)

            current.load_id = 'uuid-b'
            node._map_callback(current)
            self.assertEqual(len(loads), 2)
            self.assertEqual(len(node.publisher.messages), 2)
            self.assertEqual(node.publisher.messages[-1].map_load_id, 'uuid-b')
            current.load_id = 'uuid-a'
            node._map_callback(current)
            self.assertEqual(len(loads), 2)
            self.assertEqual(node.current_map_load_id, 'uuid-b')

    def test_old_latched_current_map_without_load_id_stays_unavailable(self):
        node_class, _ = _test_node_class()
        node = _node(node_class, '/maps')
        node._map_callback(SimpleNamespace(success=True, map_name='old'))
        self.assertEqual(_status(node)['state'], 'unavailable')
        self.assertEqual(len(node.publisher.messages), 0)
        self.assertEqual(node.current_map_name, '')
        self.assertEqual(node.current_map_load_id, '')


if __name__ == '__main__':
    unittest.main()
