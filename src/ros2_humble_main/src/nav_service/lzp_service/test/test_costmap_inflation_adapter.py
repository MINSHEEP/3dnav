#!/usr/bin/env python3
"""ROS-free tests for the route manager's costmap parameter adapter."""

import ast
from pathlib import Path
from types import SimpleNamespace
import sys
import textwrap
import time
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ROUTE_MANAGER = PACKAGE_ROOT / 'lzp_service' / 'route_navigation_manager.py'
sys.path.insert(0, str(PACKAGE_ROOT))

from lzp_service.costmap_inflation import (  # noqa: E402
    INFLATION_TARGETS,
    InflationSyncState,
)


class ParameterTypeStub:
    PARAMETER_DOUBLE = 3


class ParameterValueStub:
    def __init__(self, **values):
        self.__dict__.update(values)


class ParameterStub:
    def __init__(self, **values):
        self.__dict__.update(values)


class SetParametersStub:
    class Request:
        def __init__(self):
            self.parameters = []


class FakeFuture:
    def __init__(self, response=None, *, done=False):
        self._response = response
        self._done = done
        self.cancelled = False

    def done(self):
        return self._done

    def result(self):
        return self._response

    def cancel(self):
        self.cancelled = True


class FakeClient:
    def __init__(self, *, ready=True):
        self.ready = ready
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = FakeFuture()
        self.futures.append(future)
        return future


def load_adapter_harness():
    source = ROUTE_MANAGER.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'RouteNavigationManager'
    )
    names = ('_collect_inflation_results', '_drive_inflation_radius_sync')
    methods = [
        next(
            node for node in manager.body
            if isinstance(node, ast.FunctionDef) and node.name == name)
        for name in names
    ]
    namespace = {
        'INFLATION_TARGETS': INFLATION_TARGETS,
        'Parameter': ParameterStub,
        'ParameterType': ParameterTypeStub,
        'ParameterValue': ParameterValueStub,
        'SetParameters': SetParametersStub,
        'time': time,
    }
    body = 'class AdapterHarness:\n' + ''.join(
        textwrap.indent(ast.get_source_segment(source, method), '    ') + '\n'
        for method in methods
    )
    exec(body, namespace)
    return namespace['AdapterHarness']


def make_harness(radius_m=0.75, version=4):
    harness = load_adapter_harness()()
    configured = {'radius_m': radius_m, 'version': version}
    harness._nav2_ready = True
    harness._inflation_sync = InflationSyncState(configured)
    harness._inflation_radius_store = SimpleNamespace(
        snapshot=lambda: dict(configured))
    harness._inflation_parameter_clients = {
        target: FakeClient() for target in INFLATION_TARGETS}
    harness._inflation_futures = {}
    harness._inflation_retry_due = 0.0
    harness._inflation_retry_delay = 1.0
    harness._inflation_sync_timeout = 3.0
    return harness


class CostmapInflationAdapterTests(unittest.TestCase):

    def test_drive_sets_the_same_double_parameter_on_both_costmaps(self):
        harness = make_harness()
        harness._drive_inflation_radius_sync()

        self.assertEqual(set(harness._inflation_futures), set(INFLATION_TARGETS))
        for client in harness._inflation_parameter_clients.values():
            self.assertEqual(len(client.requests), 1)
            parameter = client.requests[0].parameters[0]
            self.assertEqual(parameter.name, 'inflation_layer.inflation_radius')
            self.assertEqual(
                parameter.value.type, ParameterTypeStub.PARAMETER_DOUBLE)
            self.assertEqual(parameter.value.double_value, 0.75)

        for pending in harness._inflation_futures.values():
            pending['future']._done = True
            pending['future']._response = SimpleNamespace(results=[
                SimpleNamespace(successful=True, reason='')])
        harness._collect_inflation_results(time.monotonic())
        self.assertTrue(harness._inflation_sync.ready)

    def test_unavailable_or_timed_out_target_never_reports_applied(self):
        harness = make_harness()
        harness._inflation_parameter_clients['local'].ready = False
        harness._drive_inflation_radius_sync()
        self.assertFalse(harness._inflation_sync.ready)
        self.assertIn(
            'unavailable',
            harness._inflation_sync.snapshot()['targets']['local']['error'])

        global_pending = harness._inflation_futures['global']
        global_pending['started_at'] = 0.0
        harness._collect_inflation_results(4.0)
        self.assertTrue(global_pending['future'].cancelled)
        self.assertFalse(harness._inflation_sync.ready)
        self.assertIn(
            'timeout',
            harness._inflation_sync.snapshot()['targets']['global']['error'])


if __name__ == '__main__':
    unittest.main()
