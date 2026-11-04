#!/usr/bin/env python3
"""Offline contract tests for CurrentMap's per-LoadMap UUID identity."""

import ast
from pathlib import Path
import threading
from types import SimpleNamespace
import uuid


SOURCE = (Path(__file__).resolve().parents[1] / 'lzp_service'
          / 'map_switcher_service.py').read_text(encoding='utf-8')
TREE = ast.parse(SOURCE)
CLASS = next(item for item in TREE.body if (
    isinstance(item, ast.ClassDef) and item.name == 'MapSwitcherService'))


class CurrentMap:
    def __init__(self):
        self.success = False
        self.map_name = ''
        self.load_id = ''
        self.message = ''


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, value):
        self.messages.append(value)


class Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class LoadMap:
    class Request:
        def __init__(self):
            self.map_url = ''

    class Response:
        RESULT_SUCCESS = 0


def _load_methods():
    names = ('publish_current_map_info', 'publish_map_load_result',
             'load_map_callback', 'load_map_blocking',
             '_invalidate_uncertain_map')
    methods = [next(item for item in CLASS.body if (
        isinstance(item, ast.FunctionDef) and item.name == name))
        for name in names]
    harness = ast.ClassDef(
        name='Harness', bases=[], keywords=[], body=methods,
        decorator_list=[])
    namespace = {
        'CurrentMap': CurrentMap, 'LoadMap': LoadMap, 'uuid': uuid,
        'threading': threading}
    exec(compile(ast.fix_missing_locations(ast.Module(
        body=[harness], type_ignores=[])), '<map_switcher_methods>', 'exec'),
        namespace)
    return namespace['Harness']


def _harness():
    node = _load_methods()()
    node.map_load_lock = threading.RLock()
    node.pending_map_name = None
    node.active_map_name = None
    node.active_map_load_id = ''
    node.current_map_publisher = Publisher()
    node.load_map_cli = SimpleNamespace(service_is_ready=lambda: True)
    node.get_logger = lambda: Logger()
    return node


def test_each_successful_load_gets_new_uuid_and_heartbeat_keeps_it():
    node = _harness()
    for _ in range(2):
        node.load_map_callback(SimpleNamespace(result=lambda: SimpleNamespace(
            result=LoadMap.Response.RESULT_SUCCESS)), 'garden')
        event = node.current_map_publisher.messages[-1]
        assert event.success and event.map_name == 'garden'
        assert len(event.load_id) == 32
        node.publish_current_map_info()
        heartbeat = node.current_map_publisher.messages[-1]
        assert heartbeat.load_id == event.load_id
        if _ == 0:
            first_load_id = event.load_id
    assert event.load_id != first_load_id


def test_inactive_server_clears_identity_and_does_not_claim_map():
    node = _harness()
    node.active_map_name = 'garden'
    node.active_map_load_id = uuid.uuid4().hex
    node.load_map_cli = SimpleNamespace(service_is_ready=lambda: False)
    node.publish_current_map_info()
    message = node.current_map_publisher.messages[-1]
    assert not message.success
    assert not message.load_id
    assert node.active_map_name is None
    assert node.active_map_load_id == ''


def test_load_map_timeout_revokes_previous_identity():
    node = _harness()
    node.active_map_name = 'garden'
    node.active_map_load_id = uuid.uuid4().hex
    node.map_server_active_timeout_sec = 1.0
    node.wait_for_map_server_active = lambda **_kwargs: (True, 'active')

    class PendingFuture:
        cancelled = False

        def add_done_callback(self, _callback):
            pass

        def cancel(self):
            self.cancelled = True

    pending = PendingFuture()
    node.load_map_cli = SimpleNamespace(
        wait_for_service=lambda **_kwargs: True,
        call_async=lambda _request: pending,
        service_is_ready=lambda: True)
    original_event = threading.Event
    threading.Event = lambda: SimpleNamespace(wait=lambda **_kwargs: False)
    try:
        success, _ = node.load_map_blocking('garden.yaml', 'garden')
    finally:
        threading.Event = original_event
    assert not success and pending.cancelled
    assert node.active_map_name is None
    assert node.active_map_load_id == ''
    revoked = node.current_map_publisher.messages[-1]
    assert not revoked.success and not revoked.map_name and not revoked.load_id


def test_load_map_transport_error_after_request_revokes_previous_identity():
    node = _harness()
    node.active_map_name = 'garden'
    node.active_map_load_id = uuid.uuid4().hex
    node.map_server_active_timeout_sec = 1.0
    node.wait_for_map_server_active = lambda **_kwargs: (True, 'active')

    class FailedFuture:
        def add_done_callback(self, callback):
            callback(self)

        def result(self):
            raise RuntimeError('connection lost after request')

    node.load_map_cli = SimpleNamespace(
        wait_for_service=lambda **_kwargs: True,
        call_async=lambda _request: FailedFuture(),
        service_is_ready=lambda: True)
    success, _ = node.load_map_blocking('garden.yaml', 'garden')
    assert not success
    assert node.active_map_name is None
    assert node.active_map_load_id == ''
    revoked = node.current_map_publisher.messages[-1]
    assert not revoked.success and not revoked.map_name and not revoked.load_id


def test_async_load_result_error_revokes_previous_identity():
    node = _harness()
    node.active_map_name = 'garden'
    node.active_map_load_id = uuid.uuid4().hex

    class FailedFuture:
        def result(self):
            raise RuntimeError('response lost')

    node.load_map_callback(FailedFuture(), 'garden')
    assert node.active_map_name is None
    assert node.active_map_load_id == ''
    revoked = node.current_map_publisher.messages[-1]
    assert not revoked.success and not revoked.map_name and not revoked.load_id


if __name__ == '__main__':
    test_each_successful_load_gets_new_uuid_and_heartbeat_keeps_it()
    test_inactive_server_clears_identity_and_does_not_claim_map()
    test_load_map_timeout_revokes_previous_identity()
    test_load_map_transport_error_after_request_revokes_previous_identity()
    test_async_load_result_error_revokes_previous_identity()
