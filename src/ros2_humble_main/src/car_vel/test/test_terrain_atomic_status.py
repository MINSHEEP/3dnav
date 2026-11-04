"""Final velocity guard consumes safe/limit atomically and rejects old latches."""
import ast,json,threading,time
from pathlib import Path
from types import SimpleNamespace
import math,textwrap,unittest
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_vel.velocity_ownership import constrain_navigation_velocity

SOURCE=Path(__file__).resolve().parents[1]/'car_vel/velocity_arbiter.py'

class AtomicTerrainTests(unittest.TestCase):
    def setUp(self):
        source=SOURCE.read_text('utf-8')
        cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='VelocityArbiter')
        methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in (
            '_terrain_status_callback','_terrain_safe_callback','_terrain_speed_callback')]
        scope={'json':json,'time':time,'math':math,'Twist':SimpleNamespace,
               'should_stop_for_terrain_update':lambda **kw:not kw['terrain_safe']}
        exec('class Harness:\n'+'\n'.join(textwrap.indent(ast.get_source_segment(source,n),'    ') for n in methods),scope)
        self.h=scope['Harness']();h=self.h
        h._lock=threading.RLock();h._campus_active=False;h._teleop_at=0.;h.timeout=.3
        h.terrain_timeout=.5;h._terrain_guard_enabled=True
        h._require_atomic_terrain_status=True
        h.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10000000000))
        h.publisher=SimpleNamespace(publish=lambda _:None)

    def status(self,**kw):
        data={'safe':True,'speed_limit_mps':.25,'stamp_ns':10000000000,
              'navigation_active':True,
              'navigation_mode':'terrain_3d','evidence_source':'restricted_map'}
        data.update(kw);self.h._terrain_status_callback(SimpleNamespace(data=json.dumps(data)))

    def test_atomic_prior_limit_cannot_be_overwritten_by_legacy_topics(self):
        self.status()
        self.assertTrue(self.h._terrain_safe)
        self.assertEqual(self.h._terrain_speed_limit,.10)
        self.h._terrain_speed_callback(SimpleNamespace(data=.25))
        self.h._terrain_safe_callback(SimpleNamespace(data=True))
        self.assertEqual(self.h._terrain_speed_limit,.10)

    def test_old_or_wrong_mode_status_cannot_enable_3d_velocity(self):
        for kw in ({'stamp_ns':1000000000},{'navigation_mode':'map_2d'},
                   {'navigation_active':False},
                   {'speed_limit_mps':float('nan')},{'safe':'true'}):
            self.status(**kw);self.assertFalse(self.h._terrain_safe)

    def test_restricted_prior_cannot_authorize_reverse_or_sideways_motion(self):
        options = dict(terrain_safe=True, speed_limit_mps=.3, restricted_flat_site=True)
        self.assertEqual(constrain_navigation_velocity(-.1,0,0,**options),(0.,0.,0.))
        self.assertEqual(constrain_navigation_velocity(.1,.05,0,**options),(0.,0.,0.))
        self.assertEqual(constrain_navigation_velocity(.3,0,.6,**options),(.1,0.,.25))
        self.assertEqual(constrain_navigation_velocity(-.1,.05,.6,
            terrain_guard_enabled=False,**options),(-.1,.05,.6))

if __name__=='__main__':unittest.main()
