"""Replay the actual ROS adapter methods against owned fake service futures."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_vel.nav2_timing_profile import TimingProfilePolicy


SOURCE = Path(__file__).resolve().parents[1] / 'car_vel/nav2_timing_profile_node.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
node = next(n for n in tree.body if isinstance(n, ast.ClassDef))
names = {'_tick', '_finish', '_observe_controller_instance', '_mode_callback'}
methods = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in names]
clock = SimpleNamespace(value=5.0)
namespace = dict(time=SimpleNamespace(monotonic=lambda: clock.value), json=__import__('json'),
    PARAMETER='FollowPath.batch_size', String=SimpleNamespace,
    GetParameters=SimpleNamespace(Request=SimpleNamespace),
    SetParametersAtomically=SimpleNamespace(Request=SimpleNamespace),
    Parameter=SimpleNamespace, ParameterValue=SimpleNamespace,
    ParameterType=SimpleNamespace(PARAMETER_INTEGER=2))
exec(compile(ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='Harness', bases=[], keywords=[],
    body=methods, decorator_list=[])], type_ignores=[])), str(SOURCE), 'exec'), namespace)
Harness = namespace['Harness']


def harness(*, mode='terrain_3d', active=False):
    h = Harness()
    h._policy = TimingProfilePolicy(configured_baseline=800)
    h._policy.observe_actual(800)
    h._policy.set_context(mode, 'xingji', active=active)
    h._mode, h._body, h._active = mode, 'xingji', active
    h._mode_at = h._body_at = h._active_at = clock.value
    h._future = h._operation = None
    h._next_retry = h._last_read = 0.0
    h._store = Mock()
    h._write_gid = h._controller_gid = '01'
    h._observed_controller_gid = '01'
    h._record_invalid = False
    h._owned_write_pending = False
    h._record_error = ''
    h._last_log = None
    h._read_client = Mock()
    h._write_client = Mock()
    h._read_client.service_is_ready.return_value = True
    h._write_client.service_is_ready.return_value = True
    h.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(
        node_name='controller_server', node_namespace='/', endpoint_gid=[1])]
    h.get_parameter = lambda key: SimpleNamespace(value=True)
    h.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=5_000_000_000))
    h.get_logger = Mock()
    h._publisher = Mock()
    return h


def test_adapter_keeps_uncertain_set_future_after_timeout():
    h = harness()
    h._policy.begin('set', 400, now=1.0)
    h._operation = 'set'
    future = h._future = Mock()
    future.done.return_value = False
    h._tick()
    assert h._future is future
    future.cancel.assert_not_called()
    h._read_client.call_async.assert_not_called()
    assert h._policy.status()['state'] == 'awaiting_late_ack'


def test_set_future_exception_never_releases_unknown_remote_write():
    h = harness()
    h._owned_write_pending = True
    h._policy.begin('set', 400, now=4.5)
    h._operation = 'set'
    h._future = Mock()
    h._future.done.return_value = True
    h._future.result.side_effect = OSError('transport failed after send')
    h._tick()
    assert h._policy.waiting_late_write
    assert not h._policy.status()['ready']
    h._read_client.call_async.assert_not_called()
    h._tick()
    h._read_client.call_async.assert_not_called()


def test_restarted_helper_with_pending_same_controller_does_not_read_or_write():
    h = harness(mode='map_2d')
    h._owned_write_pending = True
    h._policy.uncertain_write('pending ownership recovered')
    h._tick()
    h._read_client.call_async.assert_not_called()
    h._write_client.call_async.assert_not_called()
    assert not h._policy.status()['ready']


def test_new_controller_endpoint_allows_readback_after_unresolved_write():
    h = harness(mode='map_2d')
    h._owned_write_pending = True
    h._policy.uncertain_write('pending ownership recovered')
    h.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(
        node_name='controller_server', node_namespace='/', endpoint_gid=[2])]
    h._tick()
    assert not h._policy.waiting_late_write
    h._read_client.call_async.assert_called_once()
    h._write_client.call_async.assert_not_called()
    assert not h._policy.status()['ready']


def test_active_navigation_never_writes_pending_profile():
    h = harness(active=True)
    h._last_read = 4.5
    h._tick()
    h._write_client.call_async.assert_not_called()


def test_late_ack_and_scope_change_restores_exact_baseline_before_ready():
    h = harness(mode='map_2d')
    h._owned_write_pending = True
    h._policy.begin('set', 400, now=1.0)
    h._policy.expire(4.0)
    h._operation = 'set'
    h._future = Mock()
    h._future.done.return_value = True
    h._future.result.return_value = SimpleNamespace(result=SimpleNamespace(successful=True, reason=''))
    h._tick()
    request = h._read_client.call_async.call_args.args[0]
    assert request.names == ['FollowPath.batch_size']
    h._future.done.return_value = True
    h._future.result.return_value = SimpleNamespace(values=[SimpleNamespace(type=2, integer_value=400)])
    h._tick()
    request = h._write_client.call_async.call_args.args[0]
    assert request.parameters[0].value.integer_value == 800
    assert not h._policy.status()['ready']


def test_periodic_read_does_not_revoke_confirmed_profile():
    h = harness()
    h._policy.observe_actual(400)
    h._last_read = 1.0
    h._tick()
    assert h._policy.status()['ready']
    assert not h._policy.status()['configuring']


def test_call_async_exception_keeps_persisted_unknown_write_blocked():
    h = harness()
    h._write_client.call_async.side_effect = RuntimeError('transport failed during send')
    h._tick()
    assert h._owned_write_pending
    assert h._store.save.call_args.kwargs['pending_write']
    assert h._policy.waiting_late_write
    h._tick()
    h._read_client.call_async.assert_not_called()


def test_new_controller_instance_captures_its_custom_400_baseline():
    h = harness(mode='map_2d')
    h._policy.baseline = 620
    h.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(
        node_name='controller_server', node_namespace='/', endpoint_gid=[2])]
    h._tick()
    h._future.done.return_value = True
    h._future.result.return_value = SimpleNamespace(values=[SimpleNamespace(type=2, integer_value=400)])
    h._tick()
    assert h._policy.baseline == 400
    assert h._policy.status()['ready']
    h._write_client.call_async.assert_not_called()


def test_corrupt_record_never_guesses_ownership_from_an_actual_value():
    h = harness(mode='map_2d')
    h._record_invalid = True
    h._policy.uncertain_write('invalid ownership record')
    h.get_publishers_info_by_topic = lambda topic: [SimpleNamespace(
        node_name='controller_server', node_namespace='/', endpoint_gid=[2])]
    h._tick()
    assert not h._policy.status()['ready']
    h._read_client.call_async.assert_not_called()
    h._write_client.call_async.assert_not_called()
