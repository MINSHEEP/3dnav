"""Opt-in controlled-flat-site prior; never used as general hole detection."""
import math
import numpy as np
from .terrain_navigation import TerrainObservation


def restricted_flat_ground(grid, *, robot_x, robot_y, yaw,
                           live_points_map, uncertainty_radius_m=0.0):
    measured = getattr(grid, 'ground_measured', None)
    if (measured is None or not 0 <= uncertainty_radius_m <= .6
            or not all(math.isfinite(float(v)) for v in (
                robot_x,robot_y,yaw,grid.resolution,grid.origin_x,grid.origin_y))):
        return None
    n = grid.width * grid.height
    if not 0 < n <= 400000:
        return None
    if any(len(v) != n for v in (measured, grid.elevation, grid.confidence,
                                grid.roughness_m, grid.slope_deg,
                                grid.step_height_m, grid.traversal_cost)):
        return None
    c, s = math.cos(yaw), math.sin(yaw)
    pad = uncertainty_radius_m
    step = min(.12, float(grid.resolution)*.5)
    if step < .01:
        return None
    cells = set()
    # Include a 15 cm margin around the circular footprint for slow turns.
    for x in np.arange(-.60-pad, 1.75+pad+step, step):
        for y in np.arange(-.60-pad, .60+pad+step, step):
            wx = robot_x+c*x-s*y; wy = robot_y+s*x+c*y
            col = math.floor((wx-grid.origin_x)/grid.resolution)
            row = math.floor((wy-grid.origin_y)/grid.resolution)
            if not 0 <= col < grid.width or not 0 <= row < grid.height:
                return None
            cells.add(row*grid.width+col)
    for index in cells:
        values = [grid.elevation[index], grid.confidence[index],
                  grid.roughness_m[index], grid.slope_deg[index], grid.step_height_m[index]]
        if (not all(math.isfinite(float(v)) for v in values)
                or measured[index] != 1 or grid.traversal_cost[index] != 0
                or grid.confidence[index] < .75 or grid.roughness_m[index] > .03
                or abs(grid.slope_deg[index]) > 3 or abs(grid.step_height_m[index]) > .03):
            return None
    live = np.asarray(live_points_map, dtype=np.float32)
    if live.ndim != 2 or live.shape[1] != 3:
        return None
    differences = []
    in_corridor = []
    for point in live[np.isfinite(live).all(axis=1)]:
        dx,dy = point[0]-robot_x,point[1]-robot_y
        x,y = c*dx+s*dy,-s*dx+c*dy
        if not -.60 <= x <= 2.5 or abs(y) > .65:
            continue
        col=math.floor((point[0]-grid.origin_x)/grid.resolution)
        row=math.floor((point[1]-grid.origin_y)/grid.resolution)
        if not 0 <= col < grid.width or not 0 <= row < grid.height:
            continue
        index=row*grid.width+col
        if measured[index] != 1 or not math.isfinite(float(grid.elevation[index])):
            continue
        dz=float(point[2]-grid.elevation[index])
        if abs(dz) <= .05:
            differences.append(dz)
        if x <= 1.75:
            in_corridor.append(dz)
    # Live far-floor agreement is needed even though near-floor may be hidden.
    if len(differences) < 30:
        return None
    offset=float(np.median(differences))
    if any(-1.5 < dz-offset < -.06 or .06 < dz-offset < 1.5 for dz in in_corridor):
        return None
    return TerrainObservation(terrain_type='flat', confidence=min(
        float(grid.confidence[i]) for i in cells), clearance_width_m=.9)
