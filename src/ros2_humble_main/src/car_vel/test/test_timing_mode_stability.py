"""Configuration silence cannot turn a confirmed controller into unknown."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import test_nav2_timing_profile_node as fixture

path = Path(__file__).resolve().parents[2] / 'nav_service/lzp_service/test/test_controller_timing_preflight.py'
spec = importlib.util.spec_from_file_location('mode_stability_preflight', path)
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


def initialized():
    fixture.clock.value = 100.
    h = fixture.harness()
    h._policy.observe_actual(400)
    return h


def tick(h, at, *, busy):
    fixture.clock.value = at
    h._body_at = h._active_at = at
    h._active = busy
    h._last_read = at - .75
    h._tick()
    return json.loads(h._publisher.publish.call_args.args[0].data)


def test_recorded_stale_mode_event_preserves_ready_and_new_route_preflight():
    h = initialized()
    status = tick(h, 102.25, busy=True)
    assert status['navigation_mode'] == 'terrain_3d'
    assert status['profile'] == 'xingji_3d' and status['ready']
    assert status['mode_context_source'] == 'latched_configuration'
    assert status['mode_message_age_sec'] > 2.
    assert preflight.check(status=status) is None
    h._write_client.call_async.assert_not_called()


def test_mode_silence_never_resets_fallback_or_attempts_a_new_write():
    h = initialized()
    h._policy.observe_actual(800)
    h._policy.fallback = True
    assert tick(h, 102.25, busy=False)['ready']
    assert tick(h, 105., busy=False)['ready']
    h._write_client.call_async.assert_not_called()


def test_explicit_new_mode_still_requires_exact_baseline_restoration():
    h = initialized()
    h._mode_callback(NS(data=json.dumps({'mode': 'map_2d'})))
    status = tick(h, 100.5, busy=True)
    assert not status['ready'] and status['profile'] == 'baseline'
    h._write_client.call_async.assert_not_called()


def test_invalid_mode_and_expired_body_are_never_confirmed_by_cached_mode():
    h = initialized()
    h._mode_callback(NS(data='{"mode":"invalid"}'))
    assert not tick(h, 100.5, busy=True)['ready']
    h = initialized()
    fixture.clock.value = 102.25
    h._tick()
    status = json.loads(h._publisher.publish.call_args.args[0].data)
    assert status['body_id'] == '' and not status['ready']
    h._write_client.call_async.assert_not_called()


def test_legacy_body_keeps_verified_original_parameters_after_mode_silence():
    h = initialized()
    h._body = 'gangben_zsl1'
    h._mode_callback(NS(data='{"mode":"map_2d"}'))
    h._policy.observe_actual(800)
    status = tick(h, 105., busy=True)
    assert status['profile'] == 'baseline' and status['ready']
    assert status['actual_batch_size'] == status['baseline_batch_size'] == 800
    h._write_client.call_async.assert_not_called()
