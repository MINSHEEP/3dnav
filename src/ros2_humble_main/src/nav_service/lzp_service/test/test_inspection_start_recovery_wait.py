"""Keep an explicit startup pending through a still-valid authority recovery."""
from types import MethodType, SimpleNamespace as NS

import pytest

import test_3d_route_localization_refresh as fixtures


def pending(*, recovery_at=3.45, renew=False, inspection=True):
    if inspection:
        node, request, clock = fixtures.RouteRefreshTests().patrol()
    else:
        node, request, clock = fixtures.harness()
        preview = node._preview_3d_route_callback(request, NS())
        assert preview.valid, preview.message
        request.preview_id = preview.preview_id
    started = clock.now
    implementation = fixtures.fixtures._load_manager_methods(
        '_global_localization_required', '_global_localization_check')
    for name in ('_global_localization_required', '_global_localization_check'):
        function = getattr(implementation, name)
        function.__globals__['time'] = clock
        setattr(node, name, MethodType(function, node))
    real_check = node._global_localization_check
    sleep = clock.sleep
    def stream_sleep(seconds):
        sleep(seconds)
        # Other live subscriptions continue while the startup callback waits.
        node._robot_body_received_at = clock.now
        node._latest_amcl_received_at = clock.now
    clock.sleep = stream_sleep
    def guard():
        elapsed = clock.now-started
        node._global_localization_at = clock.now
        node._global_localization_status = dict(ready=False, recovering=True,
            scope='xingji_3d', map_name=node._current_map_name,
            map_load_id=node._current_map_load_id, body_generation=1,
            sensor_fresh=True, reason='matching_recovery',
            recovery_remaining_sec=(3. if renew else
                                    (3. if elapsed < 1. else 4.)-elapsed))
        if elapsed >= recovery_at:
            node._global_localization_status.update(ready=True, recovering=False)
        return real_check()
    node._global_localization_check = guard
    return node, request, clock, started


@pytest.mark.parametrize('inspection', [False, True])
def test_recorded_recovery_finishes_after_old_callers_3_1s_limit(inspection):
    node, request, clock, started = pending(inspection=inspection)
    result, ticket = node._prepare_3d_route_start(request)
    assert result.allowed, result.message
    assert ticket is not None and 3.4 <= clock.now-started < 5.
    node._action_client.send_goal_async.assert_not_called()


def test_continuously_renewed_status_cannot_make_start_wait_unbounded():
    node, request, clock, started = pending(recovery_at=100., renew=True)
    result,ticket = node._prepare_3d_route_start(request)
    assert not result.allowed and ticket is None
    assert clock.now-started <= 6.2
    assert '尚未启动' in result.message and '原任务保留' not in result.message
    node._action_client.send_goal_async.assert_not_called()


def test_expired_authority_is_rejected_without_using_caller_tail():
    node, request, clock, started = pending(recovery_at=100.)
    result, ticket = node._prepare_3d_route_start(request)
    assert not result.allowed and ticket is None
    assert result.code == 'global_localization_untrusted'
    assert 3.9 <= clock.now-started < 4.2
    node._action_client.send_goal_async.assert_not_called()


def test_pause_during_pending_start_blocks_later_recovery():
    node, request, clock, started = pending()
    def pause():
        if clock.now-started >= 2.:
            node._inspection_task_paused = True
    clock.on_sleep = pause
    result,ticket = node._prepare_3d_route_start(request)
    assert not result.allowed and ticket is None
    assert result.code == 'inspection_authorization_expired'
    assert clock.now-started < 3.
    node._action_client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('change', ['cancel', 'replacement'])
def test_manual_cancel_when_last_wait_becomes_ready_cannot_mint_new_ticket(change):
    node, request, clock, started = pending(inspection=False, recovery_at=.3)
    def cancel_at_ready():
        if clock.now-started >= .3:
            if change == 'cancel':
                node._preview_request_generation += 1
            else:
                node._preview_token = {'id': 'replacement'}
    clock.on_sleep = cancel_at_ready
    result, ticket = node._prepare_3d_route_start(request)
    assert not result.allowed and ticket is None
    assert result.code == 'preview_canceled'
    node._action_client.send_goal_async.assert_not_called()
