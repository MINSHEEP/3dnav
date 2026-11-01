"""Measured-floor provenance, overhead separation and persisted compatibility."""
import ast
from dataclasses import replace
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from lzp_service.terrain_grid import build_terrain_grid, GroundSupportGrid
from lzp_service.terrain_grid_io import write_terrain_grid, load_terrain_grid


def checker():
    source = (ROOT/'lzp_service/terrain_grid_node.py').read_text('utf-8')
    fn = next(n for n in ast.parse(source).body
              if isinstance(n, ast.FunctionDef) and n.name == 'check_ground_candidate')
    scope = {'math': math}
    exec(ast.get_source_segment(source, fn), scope)
    return scope['check_ground_candidate']


def floor():
    x, y = np.meshgrid(np.arange(-1, 1.01, .04), np.arange(-1, 1.01, .04))
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -.58)))


class MeasuredGroundTests(unittest.TestCase):
    def test_ceiling_does_not_roughen_or_strengthen_observed_floor(self):
        ground = floor()
        roof = ground.copy(); roof[:, 2] += 2.5
        a = build_terrain_grid(ground)
        b = build_terrain_grid(np.vstack((ground, roof)))
        np.testing.assert_allclose(a.roughness_m, b.roughness_m, equal_nan=True)
        np.testing.assert_allclose(a.confidence, b.confidence)
        self.assertTrue(checker()(b, 0, 0, inferred_roughness_marker_m=.05)[0])

    def test_inferred_overhead_and_unseen_cells_never_authorize_selection(self):
        points = floor()
        central = (abs(points[:, 0]) < .3) & (abs(points[:, 1]) < .3)
        points[central, 2] += 2.0
        support = GroundSupportGrid(.25, 12, 12, -1.5, -1.5, np.ones(144, bool))
        grid = build_terrain_grid(points, ground_support=support)
        self.assertTrue(np.any(grid.ground_measured == 0))
        self.assertFalse(checker()(grid, 0, 0, inferred_roughness_marker_m=.05)[0])

    def test_body_height_obstacle_is_not_removed_with_ceiling_points(self):
        ground=floor()
        cluster=ground[(abs(ground[:,0])<.15)&(abs(ground[:,1])<.15)].copy()
        cluster[:,2]+=.4
        grid=build_terrain_grid(np.vstack((ground,cluster)))
        self.assertFalse(checker()(grid,0,0,inferred_roughness_marker_m=.05)[0])
        self.assertTrue(np.any(grid.traversal_cost==100))

    def test_measured_roughness_is_not_confused_with_inference_marker(self):
        grid = build_terrain_grid(floor())
        grid = replace(grid, roughness_m=np.full(grid.width*grid.height, .055, np.float32))
        self.assertTrue(checker()(grid, 0, 0, inferred_roughness_marker_m=.05)[0])
        legacy = replace(grid, ground_measured=None)
        self.assertFalse(checker()(legacy, 0, 0, inferred_roughness_marker_m=.05)[0])

    def test_v2_round_trip_and_v1_compatibility_preserve_evidence(self):
        grid = build_terrain_grid(floor())
        with TemporaryDirectory() as directory:
            path = Path(directory)/'test.terrain.npz'
            summary = write_terrain_grid(path, map_name='test', frame_id='map', grid=grid)
            self.assertEqual(summary['version'], 2)
            loaded = load_terrain_grid(path)
            self.assertEqual(loaded.version, 2)
            np.testing.assert_array_equal(loaded.grid.ground_measured, grid.ground_measured)
            summary = write_terrain_grid(path, map_name='test', frame_id='map',
                                         grid=replace(grid, ground_measured=None))
            self.assertEqual(summary['version'], 1)
            self.assertIsNone(load_terrain_grid(path).grid.ground_measured)


if __name__ == '__main__':
    unittest.main()
