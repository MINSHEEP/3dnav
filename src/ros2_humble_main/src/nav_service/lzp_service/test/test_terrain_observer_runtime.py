"""Run the production observer callbacks against timestamp/identity races."""
import ast,json,math,textwrap
from pathlib import Path
import sys,time,unittest
from types import SimpleNamespace as NS
from collections import deque
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from lzp_service.rolling_terrain import RollingGroundMemory,matched_pose
from lzp_service.restricted_ground import restricted_flat_ground
from lzp_service.terrain_pose import propagate_localization
from lzp_service.terrain_navigation import TerrainCapabilityModel
from lzp_service.terrain_observer import estimate_local_terrain,corridor_return_diagnostics
from lzp_service.terrain_grid import build_terrain_grid


def harness():
    source=(ROOT/'lzp_service/terrain_observer_node.py').read_text('utf-8')
    cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef)
             and n.name=='TerrainObserverNode')
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in (
        '_cloud_callback','_restricted_prior','_stamp','_clear_memory','_map_callback',
        '_scan_callback','_lio_callback','_pose_refresh_tick','_navigation_callback',
        '_ground_anchor','_map_pose_callback')]
    scope={'np':np,'math':math,'json':json,'time':NS(monotonic=lambda:100),
        'matched_pose':matched_pose,'estimate_local_terrain':estimate_local_terrain,
        'corridor_return_diagnostics':corridor_return_diagnostics,
        'restricted_flat_ground':restricted_flat_ground,
        'propagate_localization':propagate_localization,
        'EmptyService':NS(Request=NS),
        'Time':NS(from_msg=lambda value: value),
        'TerrainCapabilityModel':TerrainCapabilityModel,'String':lambda **kw:NS(**kw)}
    yaw_fn=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='_yaw')
    tree=ast.Module(body=[yaw_fn,ast.ClassDef(name='Harness',bases=[],keywords=[],
                                     body=methods,decorator_list=[])],type_ignores=[])
    exec(compile(ast.fix_missing_locations(tree),str(ROOT/'lzp_service/terrain_observer_node.py'),'exec'),scope)
    h=scope['Harness']()
    h._last_update_at=0;h.minimum_update_seconds=.2;h._odom=NS();h._odom_received_at=100
    h._odom_samples=deque([(2,0,0,0)]);h._memory=RollingGroundMemory()
    h._last_cloud_stamp=0;h._body='xingji';h._body_at=100;h._mode_at=100
    h._navigation_mode='terrain_3d';h.forward_min=.1;h.forward_max=1.3;h.half_width=.45
    h._lio_healthy=True;h._lio_at=100
    h._ground_anchor=lambda _: (0.,0.,0.)
    h.get_clock=lambda:NS(now=lambda:NS(nanoseconds=2000000000))
    x,y=np.meshgrid(np.arange(.1,1.3,.03),np.arange(-.44,.45,.03))
    h._points=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
    h._xyz=lambda _:h._points;h._message=lambda v:NS(terrain_type=v.terrain_type,confidence=v.confidence)
    h.outputs=[];h.errors=[]
    h.publisher=NS(publish=h.outputs.append);h.status_publisher=NS(publish=lambda _:None)
    h._publish_error=h.errors.append
    h._restricted_enabled=False;h._grid=None;h._grid_status=None
    h._map_pose=None;h._map_pose_samples=0;h._map_identity=('map','uuid')
    h._scan_clear=True;h._scan_at=100
    h._map_pose_age=None;h._refresh_state='idle'
    h._map_pose_cutoff=0.
    return h


class ObserverRuntimeTests(unittest.TestCase):
    def message(self,sec=2,frame='odom'):
        return NS(header=NS(frame_id=frame,stamp=NS(sec=sec,nanosec=0)))

    def test_current_registered_cloud_uses_capture_time_pose(self):
        h=harness();h._cloud_callback(self.message())
        self.assertEqual(len(h.outputs),1)
        self.assertEqual(h.outputs[0].terrain_type,'flat')
        self.assertFalse(h.errors)

    def test_wrong_frame_old_cloud_and_pose_skew_do_not_publish_safe_observation(self):
        for message in (self.message(frame='map'),self.message(sec=1)):
            h=harness();h._cloud_callback(message)
            self.assertTrue(h.errors);self.assertFalse(h.outputs)
        h=harness();h._odom_samples=deque([(1,0,0,0)])
        h._cloud_callback(self.message());self.assertTrue(h.errors)

    def test_map_load_generation_revokes_recent_ground_and_pose(self):
        h=harness();h._cloud_callback(self.message());self.assertTrue(h._memory.cells)
        h._map_pose=NS();h._map_pose_samples=3
        h._map_callback(NS(success=True,map_name='map',load_id='new_uuid'))
        self.assertEqual(h._memory.cells,{})
        self.assertIsNone(h._map_pose)

    def test_gangben_does_not_enter_xingji_ground_memory(self):
        h=harness();h._body='gangben_zsl1';h._cloud_callback(self.message())
        self.assertEqual(h._memory.cells,{})
        self.assertEqual(h.outputs[0].evidence_source,'current')

    def test_malformed_or_near_obstacle_scan_cannot_authorize_map_prior(self):
        scan = NS(header=NS(frame_id='laser_2d', stamp=NS(sec=2, nanosec=0)),
                  angle_min=-math.pi, angle_increment=2*math.pi/360,
                  range_min=.05, range_max=10., ranges=[math.inf]*361)
        h = harness()
        h._scan_callback(scan)
        self.assertTrue(h._scan_clear)
        for value in (math.nan, -math.inf, .50):
            scan.ranges[180] = value
            h._scan_callback(scan)
            self.assertFalse(h._scan_clear)
        scan.ranges = [math.inf]*16
        h._scan_callback(scan)
        self.assertFalse(h._scan_clear)

    def test_lio_fault_does_not_repopulate_ground_memory_on_next_cloud(self):
        h=harness()
        h._lio_callback(NS(data=json.dumps({'state':'soft_quarantined'})))
        h._odom_samples=deque([(2,0,0,0)])
        h._cloud_callback(self.message())
        self.assertTrue(h.errors)
        self.assertFalse(h.outputs)
        self.assertEqual(h._memory.cells,{})

    def test_restricted_prior_binds_exact_grid_stamp_and_rejects_stale_pose_or_scan(self):
        h=harness();h._restricted_enabled=True
        x,y=np.meshgrid(np.arange(-3,3,.06),np.arange(-3,3,.06))
        g=build_terrain_grid(np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size))))
        grid=NS(**g.__dict__);grid.map_name='map';grid.map_load_id='uuid'
        grid.header=NS(frame_id='map',stamp=NS(sec=1791830000,nanosec=123456789))
        h._grid=grid;h._grid_status={'state':'ready','artifact_verified':True,
            'algorithm':'clustered_measured_ground_v3','map_name':'map','map_load_id':'uuid',
            'stamp_ns':1791830000123456789}
        h._map_pose=NS(header=NS(stamp=NS(sec=2,nanosec=0)),pose=NS(
            covariance=[0.]*36,pose=NS(position=NS(x=0,y=0),orientation=NS(x=0,y=0,z=0,w=1))))
        h._map_pose_samples=2
        self.assertIsNotNone(h._restricted_prior(h._points,2,0,0,0,ground_anchor=(0,0,0)))
        h._grid_status['map_load_id']='old_uuid'
        self.assertIsNone(h._restricted_prior(h._points,2,0,0,0,ground_anchor=(0,0,0)))
        h._grid_status['map_load_id']='uuid';h._scan_clear=False
        self.assertIsNone(h._restricted_prior(h._points,2,0,0,0,ground_anchor=(0,0,0)))
        h._scan_clear=True
        self.assertIsNone(h._restricted_prior(h._points,3,0,0,0,ground_anchor=(0,0,0)))


if __name__=='__main__':unittest.main()
