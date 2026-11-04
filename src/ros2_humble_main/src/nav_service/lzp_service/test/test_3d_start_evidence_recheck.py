"""Start from the actual trusted pose; ordinary receipt updates are not identities."""
import json
from types import SimpleNamespace as NS

import test_3d_goal_preview as f
import test_3d_route_localization_refresh as route_fixture
from test_global_localization_preflight import manager

harness=route_fixture.harness


def test_identity_hold_preserves_both_navigation_and_inspection_tasks():
    for inspection in (False, True):
        h=manager()
        h._inspection_task_active=inspection
        status=dict(ready=False,recovering=True,reason='identity_recovery',
            scope='xingji_3d',map_name='map',map_load_id='load',body_generation=1,
            sensor_fresh=True,recovery_remaining_sec=2.9)
        h._global_localization_callback(NS(data=json.dumps(status)))
        h._request_stop.assert_not_called()
        assert h._global_localization_check().code=='global_localization_recovering'
        h._send_zero_velocity.assert_called_once()
        status['recovery_remaining_sec']=0.
        h._global_localization_callback(NS(data=json.dumps(status)))
        h._request_stop.assert_called_once_with('global_localization_untrusted')


def test_manual_route_small_correction_rechecks_actual_start_without_motion():
    h,request,_=harness()
    preview=h._preview_3d_route_callback(request,NS())
    assert preview.valid
    request.preview_id=preview.preview_id
    h._latest_amcl_pose.pose.pose.position.x+=.20
    result,ticket=h._prepare_3d_route_start(request)
    assert result.allowed,result.message
    assert ticket['snapshot']['start_x']==1.4
    h._action_client.send_goal_async.assert_not_called()


def test_inspection_replanning_small_correction_uses_current_start():
    h,request,_=route_fixture.RouteRefreshTests().patrol()
    real=h._plan_3d_route_segments
    def plan(*args):
        value=real(*args)
        h._latest_amcl_pose.pose.pose.position.x+=.20
        return value
    h._plan_3d_route_segments=plan
    result,ticket=h._prepare_3d_route_start(request)
    assert result.allowed,result.message
    assert ticket['snapshot']['start_x']==1.4
    h._action_client.send_goal_async.assert_not_called()


def test_changed_map_is_still_rejected_with_specific_reason():
    h,request,_=route_fixture.RouteRefreshTests().patrol()
    real=h._plan_3d_route_segments
    def plan(*args):
        value=real(*args)
        h._terrain_costmap_ready_stamp_ns+=1
        return value
    h._plan_3d_route_segments=plan
    result,ticket=h._prepare_3d_route_start(request)
    assert not result.allowed and ticket is None
    assert '地形地图版本' in result.message


def test_connection_from_corrected_start_checks_every_crossed_cell():
    # The original path is free; only the short connector from the new pose
    # enters an occupied cell. Distance-only revalidation would miss it.
    grid=f._Grid([[100,0,0]])
    checked=f.CHECK_PLAN(f._path((1.1,.2),(2.2,.2)),grid,f._pose(2.2,.2),start_xy=(.9,.2))
    assert not checked.allowed


def test_large_start_displacement_remains_rejected():
    h,request,_=harness()
    preview=h._preview_3d_route_callback(request,NS())
    request.preview_id=preview.preview_id
    h._latest_amcl_pose.pose.pose.position.x+=.5
    result,ticket=h._prepare_3d_route_start(request)
    assert not result.allowed and ticket is None
    h._action_client.send_goal_async.assert_not_called()
