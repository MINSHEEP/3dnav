import ast
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


SOURCE = Path(__file__).resolve().parents[2] / 'nav_service/lzp_service/lzp_service/robot_body_manager.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'RobotBodyManager')
method = next(item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == 'stop_for_switch')


def run_cleanup(initial, stop_result=(True, 'stopped'), service_success=True):
    mode = dict(initial)
    events = []
    class Service:
        class Request:
            command = ''
    class Client:
        def __init__(self, name): self.name = name
        def wait_for_service(self, timeout_sec): return True
        def call_async(self, request):
            events.append((self.name, request.command))
            if self.name == '/control_campus_mission': mode['campus_mission_active'] = False
            if self.name == '/inspection/control': mode['inspection_active'] = False
            return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=service_success))
    class Node:
        def create_subscription(self, _typ, _name, callback, _qos): self.callback = callback
        def create_publisher(self, *_args):
            return SimpleNamespace(get_subscription_count=lambda: 1,
                                   publish=lambda _msg: events.append(('navigation', 'cancel')))
        def create_client(self, _typ, name): return Client(name)
        def destroy_node(self): events.append(('cleanup', 'destroy'))
    temporary = Node()
    def spin_once(node, timeout_sec):
        node.callback(SimpleNamespace(data=json.dumps(mode)))
    def stop_modes():
        events.append(('launch', 'stop'))
        mode.update(locked=False, execution_mode='idle')
        return stop_result
    rclpy = SimpleNamespace(create_node=lambda _name: temporary, spin_once=spin_once,
                            spin_until_future_complete=lambda *args, **kwargs: None)
    namespace = dict(rclpy=rclpy, json=json, time=__import__('time'), String=object,
                     Empty=type('Empty', (), {}), ControlCampusMission=Service,
                     ControlInspectionTask=Service, QoSProfile=lambda **kw: None,
                     DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(
        name='Manager', bases=[], keywords=[], body=[method], decorator_list=[])],
        type_ignores=[])), str(SOURCE), 'exec'), namespace)
    manager = namespace['Manager']()
    manager.node = SimpleNamespace(_stop_all_modes=stop_modes, publish_status=Mock())
    manager.stop_for_switch()
    return events


def test_confirmed_switch_cancels_tasks_and_stops_launches():
    events = run_cleanup(dict(locked=True, execution_mode='route',
                              inspection_active=True, campus_mission_active=True))
    assert events == [('/control_campus_mission', 'stop'), ('/inspection/control', 'cancel'),
                      ('navigation', 'cancel'), ('launch', 'stop'), ('cleanup', 'destroy')]


def test_failed_task_stop_aborts_before_launch_shutdown():
    with pytest.raises(RuntimeError, match='无法结束'):
        run_cleanup(dict(locked=True, execution_mode='route', inspection_active=True),
                    service_success=False)


def test_failed_launch_stop_aborts_switch():
    with pytest.raises(RuntimeError, match='停止建图或导航失败'):
        run_cleanup(dict(locked=False, execution_mode='idle'), stop_result=(False, 'still running'))
