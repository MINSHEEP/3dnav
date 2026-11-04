import math
from types import SimpleNamespace as NS
import numpy as np
from car_vel.initial_pose_quality import scan_map_quality


def pose(x=0., y=0., yaw=0.):
    return NS(position=NS(x=x,y=y,z=0.), orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))


def scene():
    data=np.zeros((100,100),dtype=int)
    data[:,60]=100
    grid=NS(info=NS(resolution=.1,width=100,height=100,origin=pose(-5,-5)),data=data.ravel())
    angles=np.linspace(-.5,.5,81)
    scan=NS(ranges=1/np.cos(angles),angle_min=-.5,angle_increment=1/80,range_min=.1,range_max=20.)
    transform=NS(translation=NS(x=0.,y=0.,z=0.),rotation=pose().orientation)
    return grid,scan,transform


def test_correct_pose_matches_wall_but_stable_wrong_pose_does_not():
    grid,scan,tf=scene()
    assert scan_map_quality(grid,scan,pose(),tf,.2)['match_ratio'] > .95
    assert scan_map_quality(grid,scan,pose(-1),tf,.2)['match_ratio'] == 0


def test_laser_extrinsics_and_rotated_map_origin():
    grid,scan,tf=scene()
    tf.translation.x=.5
    assert scan_map_quality(grid,scan,pose(-.5),tf,.2)['match_ratio'] > .95
    grid.info.origin=pose(5,-5,math.pi/2)
    tf.translation.x=0
    assert scan_map_quality(grid,scan,pose(yaw=math.pi/2),tf,.2)['match_ratio'] > .95


def test_unknown_and_outside_are_not_evidence_and_invalid_returns_excluded():
    grid,scan,tf=scene()
    grid.data[:]=-1
    assert scan_map_quality(grid,scan,pose(),tf,.2)['match_ratio']==0
    assert scan_map_quality(grid,scan,pose(100),tf,.2)['match_ratio']==0
    scan.ranges=[float('inf'),float('nan'),20.,0.]*50
    assert scan_map_quality(grid,scan,pose(),tf,.2)['valid_beams']==0


def coordinator_class():
    import ast
    import copy
    import time
    from pathlib import Path
    tree=ast.parse((Path(__file__).parents[1]/'car_vel/relocalization_coordinator.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    names={'_begin_amcl_confirmation','_ensure_initial_covariance','_amcl_callback','_yaw','_angle_distance','_stamp_seconds','_request_initial_amcl_update'}
    cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names]
    cls.bases=[]
    ns=dict(math=math,copy=copy,time=time,Duration=lambda **kw:0,EmptyService=NS(Request=lambda:None))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'coordinator','exec'),ns)
    return ns['RelocalizationCoordinator']


def test_coarse_covariance_and_timeout_are_distinct_from_refined():
    c=coordinator_class()()
    class Now:
        def __sub__(self,other):return self
        def to_msg(self):return NS(sec=1,nanosec=0)
    c.get_clock=lambda:NS(now=lambda:Now())
    c.generation=1;c.latest_amcl_stamp=0
    c.search_xy=3.;c.search_yaw=math.pi/2
    c.amcl_timeout=10.;c.rough_amcl_timeout=15.
    c.initial_pose_publisher=NS(publish=lambda p:None)
    c._publish_status=lambda *a,**kw:None
    p=NS(header=NS(frame_id='map'),pose=NS(covariance=[.04]*36,pose=pose()))
    c._begin_amcl_confirmation(p,p,'rough')
    rough=c.amcl_wait
    assert rough['candidate'].pose.covariance[0]==2.25
    assert rough['candidate'].pose.covariance[35]==(math.pi/4)**2
    assert 14.9 < rough['deadline']-rough['started_at'] < 15.1
    c._begin_amcl_confirmation(p,p,'refined')
    assert c.amcl_wait['candidate'].pose.covariance[0]==.04
    assert c.amcl_wait['deadline']-c.amcl_wait['started_at']<11
    assert p.pose.covariance[0]==.04


def test_stable_amcl_requires_scan_quality_and_accepts_coarse_correction():
    c=coordinator_class()()
    c.generation=1;c.latest_amcl_stamp=None
    c.amcl_position_std_limit=.55;c.amcl_yaw_std_limit=.5
    c.search_xy=3.;c.search_yaw=math.pi/2
    c.amcl_max_gap=.75;c.amcl_position_spread_limit=.15;c.amcl_yaw_spread_limit=.3
    c.amcl_sample_count=5;c.amcl_hold=1.5;c.scan_match_ratio=.65
    statuses=[]
    c._publish_status=lambda *a,**kw:statuses.append((a,kw))
    candidate=NS(pose=NS(pose=pose()))
    c.amcl_wait=dict(generation=1,candidate=candidate,source='rough',amcl_stamp_before=0,window=None,metrics={},refinement_error='')
    c._initial_scan_quality=lambda m:({'match_ratio':0.,'valid_beams':81},'poor fit')
    def sample(sec,nsec=0):
        return NS(header=NS(stamp=NS(sec=sec,nanosec=nsec)),pose=NS(pose=pose(x=1.5),covariance=[.01]*36))
    for sec in range(1,7):c._amcl_callback(sample(sec))
    assert c.amcl_wait is not None and c.amcl_wait['window'] is None
    c._initial_scan_quality=lambda m:({'match_ratio':.95,'valid_beams':81},'good fit')
    for i in range(5):c._amcl_callback(sample(10+i//2,(i%2)*500000000))
    assert c.amcl_wait is None
    assert statuses[-1][1]['success'] is True


def test_initial_update_uses_new_scans_without_queuing_requests():
    c=coordinator_class()()
    c.nomotion_update_count=8;c.nomotion_update_interval=.35
    calls=[];future=NS(done=lambda:False)
    c.nomotion_client=NS(service_is_ready=lambda:True,call_async=lambda r:(calls.append(r) or future))
    c.initial_scan=NS(header=NS(stamp=NS(sec=1,nanosec=0)))
    waiting=dict(source='rough',deadline=30.,next_nomotion_at=0.,nomotion_sent=0)
    c._request_initial_amcl_update(waiting,1.)
    assert len(calls)==1 and waiting['next_nomotion_at']==1.2
    c.initial_scan.header.stamp.sec=2
    c._request_initial_amcl_update(waiting,2.)
    assert len(calls)==1
    future.done=lambda:True
    c._request_initial_amcl_update(waiting,2.)
    assert len(calls)==2
    c._request_initial_amcl_update(waiting,3.)
    assert len(calls)==2
    c.initial_scan.header.stamp.sec=3
    c._request_initial_amcl_update(waiting,31.)
    assert len(calls)==2
