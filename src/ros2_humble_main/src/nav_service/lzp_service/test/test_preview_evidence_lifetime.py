"""Human waiting does not invalidate unchanged server-side planning evidence."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import math
import test_3d_goal_preview as fixtures

from test_3d_route_localization_refresh import harness
from test_3d_localization_refresh import harness as single_harness


def preview_then_wait():
    h,request,clock=harness()
    preview=h._preview_3d_route_callback(request,NS())
    assert preview.valid,preview.message
    request.preview_id=preview.preview_id
    clock.now+=600.
    h._robot_body_received_at=h._latest_amcl_received_at=clock.now
    return h,request,clock


def test_unchanged_preview_can_start_after_ten_minutes():
    h,request,_=preview_then_wait()
    result,ticket=h._prepare_3d_route_start(request)
    assert result.allowed,result.message
    assert ticket is not None
    h._action_client.send_goal_async.assert_not_called()


def test_single_goal_preview_can_be_confirmed_after_ten_minutes():
    h,request,clock=single_harness()
    preview=h._preview_3d_goal_callback(request,NS())
    assert preview.valid
    clock.now+=600.
    h._robot_body_received_at=h._latest_amcl_received_at=clock.now
    h._single_goal_callback.side_effect=lambda _:setattr(h,'_mode','single')
    response=h._start_previewed_3d_goal_callback(NS(preview_id=preview.preview_id),NS())
    assert response.success,response.message
    h._single_goal_callback.assert_called_once()


def test_inspection_can_reserve_unchanged_preview_after_ten_minutes():
    h,request,_=preview_then_wait()
    request.owner_token='inspection-1'
    result=h._reserve_3d_inspection_route_callback(request,NS())
    assert result.success,result.message
    assert h._inspection_route_authorization['owner_token']=='inspection-1'


def test_same_named_map_reload_still_invalidates_old_preview():
    h,request,_=preview_then_wait()
    h._current_map_load_id='new-load'
    result,ticket=h._prepare_3d_route_start(request)
    assert not result.allowed and ticket is None


def test_reservation_reuses_static_path_proof_but_checks_new_start_connector():
    h,request,_=harness()
    preview=h._preview_3d_route_callback(request,NS())
    request.preview_id=preview.preview_id
    request.owner_token='inspection-1'
    h._validate_3d_route_segments=Mock(side_effect=AssertionError('full route recomputed'))
    result=h._reserve_3d_inspection_route_callback(request,NS())
    assert result.success,result.message
    h._validate_3d_route_segments.assert_not_called()


def test_long_preview_rechecks_connector_without_repeating_a_million_cell_reads():
    h,_,_=harness()
    class Costs:
        reads=0
        def __getitem__(self,cell):
            self.reads+=1
            return 0
    class Grid:
        resolution=.05;origin_x=origin_y=origin_yaw=0.
        width=1500;height=100;data=Costs()
        def world_to_cell(self,x,y):
            row,col=math.floor(y/self.resolution),math.floor(x/self.resolution)
            return (row,col) if 0<=row<self.height and 0<=col<self.width else None
    grid=Grid();h._traversability_grid=grid
    points=[(1.2+i*.02,1.2) for i in range(3233)]
    segments=[(fixtures._pose(*points[-1]),fixtures._path(*points))]
    proof,_=h._validate_3d_route_segments(segments,points[0])
    assert proof.allowed,proof.message
    full_reads=grid.data.reads;grid.data.reads=0
    result,_=h._recheck_3d_start(h._preview_3d_snapshot(),segments,reuse_preview=True)
    assert result.allowed,result.message
    assert grid.data.reads*100<full_reads
    print(dict(full_route_cell_reads=full_reads,cached_start_cell_reads=grid.data.reads))
