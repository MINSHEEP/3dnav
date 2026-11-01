#!/usr/bin/env python3
"""Check saved 3-D ground candidates without importing ROS or NumPy."""

import ast
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest


SOURCE = (
    Path(__file__).resolve().parents[1] / 'lzp_service' /
    'terrain_grid_node.py'
)


def _source_functions():
    module = ast.parse(SOURCE.read_text(encoding='utf-8'))
    function = next(item for item in module.body if (
        isinstance(item, ast.FunctionDef)
        and item.name == 'check_ground_candidate'))
    node_class = next(item for item in module.body if (
        isinstance(item, ast.ClassDef) and item.name == 'TerrainGridNode'))
    callback = next(item for item in node_class.body if (
        isinstance(item, ast.FunctionDef)
        and item.name == '_check_3d_ground'))
    map_callback = next(item for item in node_class.body if (
        isinstance(item, ast.FunctionDef)
        and item.name == '_map_callback'))
    invalidate_map = next(item for item in node_class.body if (
        isinstance(item, ast.FunctionDef)
        and item.name == '_invalidate_map'))
    selected = ast.Module(body=[function, ast.ClassDef(
        name='GroundNode', bases=[], keywords=[],
        body=[callback, map_callback, invalidate_map],
        decorator_list=[])], type_ignores=[])
    ast.fix_missing_locations(selected)
    namespace = {'math': math}
    exec(compile(selected, str(SOURCE), 'exec'), namespace)
    return namespace['check_ground_candidate'], namespace['GroundNode']


def _grid():
    width = height = 7
    count = width * height
    return SimpleNamespace(
        resolution=0.25, width=width, height=height,
        origin_x=-0.875, origin_y=-0.875,
        elevation=[0.0] * count,
        confidence=[1.0] * count,
        roughness_m=[0.01] * count,
        traversal_cost=[0] * count,
    )


def _node(node_type, grid):
    node = node_type()
    node.current_map_name = 'verified_map'
    node.current_map_load_id = 'load-a'
    node._rejected_map_generations = set()
    node.saved_pcd_directory = '/maps'
    node._ground_grid = grid
    node._ground_map_name = 'verified_map'
    node._ground_load_id = 'load-a'
    node._ground_frame_id = 'map'
    node._ground_artifact_verified = True
    node._ground_algorithm = 'free_space_lower_envelope_v2'
    node._ground_roughness_marker_m = 0.05
    return node


class GroundCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        check, cls.node_type = _source_functions()
        cls.check = staticmethod(check)

    def test_valid_candidate_is_only_a_map_point_not_motion_permission(self):
        grid = _grid()
        grid.elevation[3 * 7 + 3] = 0.12
        valid, code, message, z = self.check(
            grid, 0.0, 0.0, inferred_roughness_marker_m=0.05)
        self.assertTrue(valid)
        self.assertEqual(code, 'candidate_ground')
        self.assertAlmostEqual(z, 0.12)
        self.assertIn('不保证实时无坑', message)

    def test_footprint_unknown_special_and_lethal_cells_are_rejected(self):
        for cost, expected in ((-1, 'unknown_footprint'),
                               (90, 'blocked_footprint'),
                               (100, 'blocked_footprint')):
            with self.subTest(cost=cost):
                grid = _grid()
                grid.traversal_cost[3 * 7 + 4] = cost
                result = self.check(
                    grid, 0.0, 0.0, inferred_roughness_marker_m=0.05)
                self.assertFalse(result[0])
                self.assertEqual(result[1], expected)
        grid = _grid()
        grid.traversal_cost[0] = 100  # Outside the 0.45 m footprint.
        self.assertTrue(self.check(
            grid, 0.0, 0.0, inferred_roughness_marker_m=0.05)[0])

    def test_candidate_center_must_be_ordinary_free_ground(self):
        for cost, expected in ((-1, 'unknown_center'),
                               (60, 'non_free_center'),
                               (90, 'non_free_center'),
                               (100, 'non_free_center')):
            with self.subTest(cost=cost):
                grid = _grid()
                grid.traversal_cost[3 * 7 + 3] = cost
                valid, code, _, _ = self.check(
                    grid, 0.0, 0.0, inferred_roughness_marker_m=0.05)
                self.assertFalse(valid)
                self.assertEqual(code, expected)

    def test_missing_low_confidence_or_reconstructed_ground_fails_closed(self):
        index = 3 * 7 + 4
        for layer, value, expected in (
                ('elevation', float('nan'), 'unknown_ground'),
                ('confidence', 0.74, 'low_confidence'),
                ('roughness_m', 0.05, 'possible_reconstruction')):
            with self.subTest(layer=layer):
                grid = _grid()
                getattr(grid, layer)[index] = value
                result = self.check(
                    grid, 0.0, 0.0, inferred_roughness_marker_m=0.05)
                self.assertFalse(result[0])
                self.assertEqual(result[1], expected)

    def test_invalid_coordinates_and_footprint_edge_fail_closed(self):
        grid = _grid()
        for x, y in ((float('nan'), 0.0), (float('inf'), 0.0)):
            with self.subTest(x=x):
                self.assertFalse(self.check(
                    grid, x, y, inferred_roughness_marker_m=0.05)[0])
        result = self.check(
            grid, 0.70, 0.0, inferred_roughness_marker_m=0.05)
        self.assertFalse(result[0])
        self.assertEqual(result[1], 'footprint_outside_map')

    def test_service_requires_exact_active_verified_map_and_known_algorithm(self):
        grid = _grid()
        node = _node(self.node_type, grid)
        request = SimpleNamespace(
            map_name='verified_map', map_load_id='load-a', x=0.0, y=0.0)

        def query():
            response = SimpleNamespace(valid=None, code='', message='', ground_z=0.0)
            return node._check_3d_ground(request, response)

        self.assertTrue(query().valid)
        request.map_load_id = ''
        self.assertEqual(query().code, 'load_id_required')
        request.map_load_id = 'old-load'
        self.assertEqual(query().code, 'stale_map_generation')
        request.map_load_id = 'load-a'
        node._ground_load_id = 'old-load'
        self.assertEqual(query().code, 'stale_map_generation')
        node._ground_load_id = 'load-a'
        request.map_name = 'other_map'
        self.assertEqual(query().code, 'map_mismatch')
        request.map_name = 'verified_map'
        node._ground_artifact_verified = False
        self.assertEqual(query().code, 'unverified_ground')
        node._ground_artifact_verified = True
        node._ground_frame_id = 'odom'
        self.assertEqual(query().code, 'frame_mismatch')
        node._ground_frame_id = 'map'
        node._ground_algorithm = 'unknown_algorithm'
        self.assertEqual(query().code, 'unsupported_artifact')
        node._ground_algorithm = 'free_space_lower_envelope_v2'
        node._ground_roughness_marker_m = None
        self.assertEqual(query().code, 'invalid_grid')
        node._ground_roughness_marker_m = 0.05
        node.current_map_load_id = ''
        self.assertEqual(query().code, 'map_generation_unavailable')
        node.current_map_load_id = 'load-a'
        node.current_map_name = ''
        self.assertEqual(query().code, 'map_unavailable')

    def test_load_id_gates_old_latch_and_same_name_reload(self):
        with TemporaryDirectory() as directory:
            metadata = Path(directory) / 'verified_map.localization.yaml'
            metadata.write_text('terrain_grid: {}\n', encoding='utf-8')
            node = self.node_type()
            node.saved_pcd_directory = directory
            node.current_map_name = ''
            node.current_map_load_id = ''
            node._rejected_map_generations = set()
            node.loaded_artifact_signature = None
            node._ground_grid = None
            node._ground_map_name = ''
            node._ground_load_id = ''
            node._ground_frame_id = ''
            node._ground_artifact_verified = False
            node._ground_algorithm = ''
            node._ground_roughness_marker_m = None
            statuses = []
            node.status_publisher = SimpleNamespace(
                publish=lambda status: statuses.append(json.loads(status.data)))
            loads = []
            publishes = []
            grids = [_grid(), _grid()]

            def load_bound(directory_path, *, map_name, localization_metadata):
                self.assertEqual(map_name, 'verified_map')
                self.assertIsNone(node._ground_grid)
                grid = grids[len(loads)]
                loads.append(grid)
                return SimpleNamespace(
                    grid=grid, frame_id='map', generation={
                        'algorithm': 'free_space_lower_envelope_v2',
                        'parameters': {'roughness_caution_m': 0.05}})

            def publish_grid(grid, **kwargs):
                publishes.append((grid, kwargs))
                node._ground_grid = grid
                node._ground_map_name = node.current_map_name
                node._ground_load_id = node.current_map_load_id
                node._ground_artifact_verified = kwargs['artifact_verified']

            globals_ = node._map_callback.__func__.__globals__
            globals_.update({
                'Path': Path,
                'String': lambda: SimpleNamespace(data=''),
                'json': json,
                'yaml': SimpleNamespace(safe_load=lambda stream: {
                    'terrain_grid': {
                        'source_pcd_sha256': 'a' * 64,
                        'sha256': 'b' * 64}}),
                'terrain_artifact_signature': lambda path: (1, 1),
                'load_bound_terrain_grid': load_bound,
            })
            node._publish_grid_data = publish_grid
            old_latch = SimpleNamespace(
                success=True, map_name='verified_map',
                message='Successfully loaded map: verified_map')
            first = SimpleNamespace(
                success=True, map_name='verified_map', load_id='load-a',
                message='Successfully loaded map: verified_map')
            heartbeat = SimpleNamespace(
                success=True, map_name='verified_map', load_id='load-a',
                message='Current map: verified_map')
            next_load = SimpleNamespace(
                success=True, map_name='verified_map', load_id='load-b',
                message='Successfully loaded and confirmed map: verified_map')

            node._map_callback(old_latch)
            self.assertEqual(len(loads), 0)
            self.assertEqual(len(publishes), 0)
            self.assertEqual(statuses[-1]['state'], 'unavailable')
            node._map_callback(first)
            self.assertEqual(loads, grids[:1])
            self.assertEqual(len(publishes), 1)
            self.assertIs(node._ground_grid, grids[0])
            node._map_callback(heartbeat)
            self.assertEqual(len(loads), 1)
            self.assertEqual(len(publishes), 1)
            node._map_callback(next_load)
            self.assertEqual(loads, grids)
            self.assertEqual(len(publishes), 2)
            self.assertIs(node._ground_grid, grids[1])
            self.assertEqual(statuses[-1]['message'],
                             'terrain map generation changed')
            self.assertEqual(publishes[-1][1]['source'],
                             'persisted terrain verified_map.terrain.npz')
            node._map_callback(next_load)
            self.assertEqual(len(loads), 2)
            self.assertEqual(len(publishes), 2)
            self.assertEqual(node._ground_load_id, 'load-b')


if __name__ == '__main__':
    unittest.main()
