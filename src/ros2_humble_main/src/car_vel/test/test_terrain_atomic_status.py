"""Final velocity guard consumes safe/limit atomically and rejects old latches."""
import json
from types import SimpleNamespace
import unittest

from test_velocity_arbiter_callbacks import VelocityArbiterCallbacksTest as ArbiterFixture
from car_vel.velocity_ownership import constrain_navigation_velocity

class AtomicTerrainTests(unittest.TestCase):
    def setUp(self):
        fixture = ArbiterFixture()
        fixture.setUp()
        fixture.now = 10.0
        self.h = fixture.node

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
