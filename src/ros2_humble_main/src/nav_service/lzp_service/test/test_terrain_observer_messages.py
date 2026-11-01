"""Exercise real message/error boundaries with ROS-like strict field types."""
import ast
import json
import math
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import test_terrain_observer_runtime as runtime
from lzp_service.terrain_navigation import TerrainObservation as Observation

FLOAT_FIELDS = ('slope_deg', 'cross_slope_deg', 'step_height_m', 'gap_width_m',
                'roughness_m', 'clearance_width_m', 'confidence')


class StrictMessage:
    """Matches the generated ROS float assertions shown in the captured log."""
    def __init__(self):
        self.header = NS(frame_id='', stamp=None)

    def __setattr__(self, field, value):
        if field in FLOAT_FIELDS:
            assert isinstance(value, float), f"The '{field}' field must be of type 'float'"
        if field in ('negative_obstacle', 'slippery'):
            assert isinstance(value, bool), f'{field} must be bool'
        object.__setattr__(self, field, value)


def methods():
    source = ROOT / 'lzp_service/terrain_observer_node.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef)
               and item.name == 'TerrainObserverNode')
    selected = [item for item in cls.body if isinstance(item, ast.FunctionDef)
                and item.name in ('_message', '_publish_error')]
    scope = {'__package__': 'lzp_service', 'TerrainObservation': StrictMessage,
             'String': lambda **kw: NS(**kw), 'json': json, 'math': math}
    program = ast.Module(body=[ast.ClassDef(name='Harness', bases=[], keywords=[],
        body=selected, decorator_list=[])], type_ignores=[])
    exec(compile(ast.fix_missing_locations(program), str(source), 'exec'), scope)
    return scope['Harness']


class TerrainObserverMessagesTests(unittest.TestCase):
    def node(self):
        node = runtime.harness()
        real = methods()
        node._message = real._message
        node._publish_error = MethodType(real._publish_error, node)
        node.status_messages = []
        node.status_publisher = NS(publish=node.status_messages.append)
        node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=2_000_000_000,
            to_msg=lambda: NS(sec=2, nanosec=0)))
        return node

    def test_error_path_publishes_unknown_instead_of_crashing(self):
        node = self.node()
        node._memory.clear = Mock()
        node._publish_error('LIO is initializing')
        output = node.outputs[-1]
        self.assertEqual(output.terrain_type, 'unknown')
        self.assertEqual(output.confidence, 0.0)
        self.assertEqual(output.clearance_width_m, 0.0)
        node._memory.clear.assert_called_once()
        self.assertEqual(json.loads(node.status_messages[-1].data)['state'], 'error')

    def test_integer_and_numpy_scalars_are_normalized_at_message_boundary(self):
        values = {field: 0 for field in FLOAT_FIELDS}
        values.update(confidence=np.float32(0.8), clearance_width_m=np.int64(1),
                      negative_obstacle=np.bool_(False), slippery=np.bool_(False))
        output = methods()._message(Observation(**values))
        self.assertTrue(all(type(getattr(output, field)) is float for field in FLOAT_FIELDS))
        self.assertIs(type(output.negative_obstacle), bool)

    def test_non_finite_data_cannot_be_published_as_a_valid_observation(self):
        for field in FLOAT_FIELDS:
            with self.subTest(field=field), self.assertRaises(ValueError):
                methods()._message(Observation(**{field: float('nan')}))

    def test_lio_startup_error_then_healthy_cloud_keeps_node_alive(self):
        node = self.node()
        node._lio_healthy = False
        message = NS(header=NS(frame_id='odom', stamp=NS(sec=2, nanosec=0)))
        node._cloud_callback(message)
        self.assertEqual(node.outputs[-1].terrain_type, 'unknown')
        node._lio_healthy = True
        node._last_update_at = 0
        # The rejected scan has already consumed its timestamp; recovery must
        # use a genuinely newer scan, not replay the old one.
        node._cloud_callback(NS(header=NS(frame_id='odom',
            stamp=NS(sec=2, nanosec=10_000_000))))
        self.assertEqual(node.outputs[-1].terrain_type, 'flat')
        self.assertGreater(node.outputs[-1].confidence, 0)


if __name__ == '__main__':
    unittest.main()
