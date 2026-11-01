"""Controlled-site fallback needs measured map plus live floor agreement."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from lzp_service.terrain_grid import build_terrain_grid
from lzp_service.restricted_ground import restricted_flat_ground


class RestrictedGroundTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        x,y=np.meshgrid(np.arange(-3,3,.06),np.arange(-3,3,.06))
        cls.grid=build_terrain_grid(np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size))))
        x,y=np.meshgrid(np.arange(1.04,1.30,.02),np.arange(-.40,.41,.03))
        cls.live=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))

    def assess(self,grid=None,live=None,**kw):
        return restricted_flat_ground(grid or self.grid,robot_x=0,robot_y=0,yaw=0,
            live_points_map=self.live if live is None else live,**kw)

    def test_known_flat_map_can_supply_only_the_explicit_prior(self):
        self.assertIsNotNone(self.assess())
        self.assertIsNone(self.assess(replace(self.grid,ground_measured=None)))
        self.assertIsNone(self.assess(live=np.empty((0,3))))

    def test_live_new_obstacle_or_drop_overrides_the_map(self):
        for z in (.1,-.2):
            points=np.vstack((self.live,[[.5,0,z]]))
            self.assertIsNone(self.assess(live=points))

    def test_inferred_unknown_and_step_cells_block_prior(self):
        i=int((-self.grid.origin_y)/self.grid.resolution)*self.grid.width+int(
            (-self.grid.origin_x)/self.grid.resolution)
        for layer,value in [('ground_measured',0),('traversal_cost',-1),
                            ('step_height_m',.10),('confidence',.5)]:
            changed=getattr(self.grid,layer).copy();changed[i]=value
            self.assertIsNone(self.assess(replace(self.grid,**{layer:changed})))
        self.assertIsNone(self.assess(uncertainty_radius_m=.7))


if __name__=='__main__':unittest.main()
