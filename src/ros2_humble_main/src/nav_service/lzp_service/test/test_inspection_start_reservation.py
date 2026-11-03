"""Real start/control callbacks under executor contention; no ROS or motion."""
import ast
import copy
from concurrent.futures import Future,ThreadPoolExecutor
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock
import uuid

SOURCE=Path(__file__).resolve().parents[1]/'lzp_service/inspection_task_manager.py'


def harness():
    tree=ast.parse(SOURCE.read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='InspectionTaskManager')
    cls=copy.deepcopy(cls)
    cls.bases=[]
    cls.body=[n for n in cls.body if not (isinstance(n,ast.FunctionDef) and n.name=='__init__')]
    class ShortEvent:
        def __init__(self):self.event=threading.Event()
        def set(self):self.event.set()
        def wait(self,seconds):return self.event.wait(.25)
    ns=dict(threading=NS(Event=ShortEvent),uuid=uuid,time=time,json=json,copy=copy,
        Reserve3DInspectionRoute=NS(Request=NS),Revoke3DInspectionRoute=NS(Request=NS),
        TERRAIN_3D='terrain_3d',utc_now=lambda:'now',navigation_mode_label=lambda _:'3D',
        _json=lambda v:json.dumps(v,ensure_ascii=False))
    exec('from __future__ import annotations\n'+ast.unparse(cls),ns)
    h=ns['InspectionTaskManager']()
    h._lock=threading.RLock()
    h._pending_task_start=None
    h._is_active=lambda:False
    h._route_request_future=None
    h._nav2_ready=h._navigation_mode_seen=h._localization_ready=True
    h._selected_navigation_mode='terrain_3d';h._selected_robot_body='xingji'
    h._localized_map_name='map'
    route=dict(id=8,name='route',map_name='map',points=[dict(id=1)])
    h._routes=NS(get_route=lambda _:copy.deepcopy(route))
    h._database=NS(get_template=lambda _:None,create_task=Mock(),add_event=Mock())
    h._validate_template=lambda *args:dict(checkpoints=[])
    h._route_to_dict=lambda route:copy.deepcopy(route)
    h._reset_runtime_events=Mock();h._camera=NS(set_context=Mock())
    h._state=dict(state='idle',active=False)
    h._log=Mock();h._publish=Mock()
    h.get_logger=lambda:Mock()
    h._revoke_3d_route_client=Mock()
    h._revoke_3d_route_client.service_is_ready.return_value=True
    request=NS(name='test',route_id=8,loop_count=1,loop_interval_minutes=0.,preview_id='preview',bidirectional=False)
    return h,request


def test_ready_response_is_processed_with_six_workers_contending_for_state_lock():
    h,request=harness();future=Future()
    with ThreadPoolExecutor(max_workers=6) as executor:
        def request_backend(_):
            entered=[threading.Event() for _ in range(5)]
            def callback(event):
                event.set()
                with h._lock:pass
            for event in entered:executor.submit(callback,event)
            assert all(event.wait(1.) for event in entered)
            executor.submit(future.set_result,NS(success=False,message='backend returned promptly'))
            return future
        h._reserve_3d_route_client=NS(service_is_ready=lambda:True,call_async=request_backend)
        response=executor.submit(h._start_task_callback,request,NS()).result(timeout=2.)
    assert not response.success
    assert 'backend returned promptly' in response.message,response.message
    h._database.create_task.assert_not_called()


def test_round_trip_start_reserves_the_preview_with_direction_enabled():
    h, request = harness()
    request.bidirectional, request.loop_count = True, 2
    future = Future()
    future.set_result(NS(success=True, message='reserved'))
    h._reserve_3d_route_client = NS(
        service_is_ready=lambda: True, call_async=Mock(return_value=future))
    response = h._start_task_callback(request, NS())
    assert response.success, response.message
    reserved = h._reserve_3d_route_client.call_async.call_args.args[0]
    assert reserved.bidirectional and reserved.loop_count == 2
    assert h._state['bidirectional']

    h._is_active = lambda: True
    h._interval_scale = 60.0
    h._start_route = Mock()
    h._persist = Mock()
    h._begin_patrol_loop()
    h._enter_interval_wait()
    h._drive_interval_wait()
    assert h._state['current_loop'] == 2
    assert [call.kwargs['reverse'] for call in h._start_route.call_args_list] == [
        False, True]
    assert all(call.kwargs['hold'] for call in h._start_route.call_args_list)


def test_cancel_during_reservation_cannot_create_task_when_response_arrives_late():
    h,request=harness();future=Future();sent=threading.Event()
    h._reserve_3d_route_client=NS(service_is_ready=lambda:True,
        call_async=lambda _: (sent.set(),future)[1])
    with ThreadPoolExecutor(max_workers=2) as executor:
        pending=executor.submit(h._start_task_callback,request,NS())
        assert sent.wait(1.)
        control=executor.submit(h._control_task_callback,NS(command='cancel'),NS()).result(timeout=1.)
        future.set_result(NS(success=True,message='reserved'))
        response=pending.result(timeout=1.)
    assert control.success,control.message
    assert not response.success
    h._database.create_task.assert_not_called()
    h._revoke_3d_route_client.call_async.assert_called()


def test_changed_route_while_waiting_cannot_commit_frozen_start():
    h,request=harness();future=Future()
    def reply(_):
        route=h._routes.get_route(8);route['points'].append(dict(id=2))
        h._routes.get_route=lambda _:route
        future.set_result(NS(success=True,message='reserved'))
        return future
    h._reserve_3d_route_client=NS(service_is_ready=lambda:True,call_async=reply)
    response=h._start_task_callback(request,NS())
    assert not response.success
    h._database.create_task.assert_not_called()


def test_successful_reservation_commits_exactly_one_task():
    h,request=harness();future=Future();future.set_result(NS(success=True,message='reserved'))
    h._reserve_3d_route_client=NS(service_is_ready=lambda:True,call_async=lambda _:future)
    response=h._start_task_callback(request,NS())
    assert response.success,response.message
    assert response.task_id
    h._database.create_task.assert_called_once()


def test_timeout_then_late_success_only_requests_cleanup():
    h,request=harness();future=Future()
    h._reserve_3d_route_client=NS(service_is_ready=lambda:True,call_async=lambda _:future)
    response=h._start_task_callback(request,NS())
    assert not response.success and '不是预览过期' in response.message
    assert h._pending_task_start is None
    future.set_result(NS(success=True,message='late reservation'))
    h._database.create_task.assert_not_called()
    h._revoke_3d_route_client.call_async.assert_called_once()


def test_concurrent_start_cannot_replace_pending_owner():
    h,request=harness();future=Future();sent=threading.Event()
    h._reserve_3d_route_client=NS(service_is_ready=lambda:True,
        call_async=lambda _: (sent.set(),future)[1])
    with ThreadPoolExecutor(max_workers=2) as executor:
        first=executor.submit(h._start_task_callback,request,NS())
        assert sent.wait(1.)
        second=h._start_task_callback(request,NS())
        assert not second.success and '启动请求正在确认' in second.message
        future.set_result(NS(success=True,message='reserved'))
        result=first.result(timeout=1.)
    assert result.success
    h._database.create_task.assert_called_once()


def test_template_change_during_wait_is_not_committed():
    h,request=harness();future=Future()
    def reply(_):
        h._validate_template=lambda *args:dict(checkpoints=[dict(route_point_id=1)])
        future.set_result(NS(success=True,message='reserved'))
        return future
    h._reserve_3d_route_client=NS(service_is_ready=lambda:True,call_async=reply)
    response=h._start_task_callback(request,NS())
    assert not response.success and '检测点配置变化' in response.message
    h._database.create_task.assert_not_called()


def test_original_2d_and_unpreviewed_gangben_paths_do_not_wait_for_reservation():
    for mode,body in (('map_2d','xingji'),('terrain_3d','gangben_zsl1')):
        h,request=harness()
        h._selected_navigation_mode=mode;h._selected_robot_body=body
        request.preview_id=''
        h._reserve_3d_route_client=Mock()
        response=h._start_task_callback(request,NS())
        assert response.success,response.message
        h._reserve_3d_route_client.call_async.assert_not_called()
        h._database.create_task.assert_called_once()
