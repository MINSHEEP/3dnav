"""Read-only scan/map evidence used only during initial AMCL confirmation."""
import math
import numpy as np


def _rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def scan_points_in_base(scan, base_from_scan, max_points=180):
    """Project finite laser returns using the same mounting transform as scoring."""
    ranges = np.asarray(scan.ranges, dtype=float)
    valid = np.flatnonzero(np.isfinite(ranges) & (ranges >= scan.range_min)
                           & (ranges < scan.range_max))
    if len(valid) > max_points:
        valid = valid[np.linspace(0, len(valid)-1, max_points, dtype=int)]
    angles = scan.angle_min + valid * scan.angle_increment
    points = np.column_stack((ranges[valid]*np.cos(angles),
                              ranges[valid]*np.sin(angles), np.zeros(len(valid))))
    t = base_from_scan.translation
    return points @ _rotation(base_from_scan.rotation).T + [t.x, t.y, t.z]


def scan_map_quality(grid, scan, map_from_base, base_from_scan, tolerance):
    """Score up to 180 actual returns; unknown/outside cells count as misses."""
    points = scan_points_in_base(scan, base_from_scan)
    result = {'valid_beams': int(len(points)), 'match_ratio': 0.0}
    info = grid.info
    if not len(points) or info.resolution <= 0 or len(grid.data) != info.width*info.height:
        return result
    t = map_from_base.position
    points = points @ _rotation(map_from_base.orientation).T + [t.x, t.y, t.z]
    t = info.origin.position
    points = (points - [t.x, t.y, t.z]) @ _rotation(info.origin.orientation)
    cells = np.floor(points[:, :2] / info.resolution).astype(int)
    occupied = np.asarray(grid.data).reshape(info.height, info.width)
    inside = ((cells[:, 0] >= 0) & (cells[:, 0] < info.width)
              & (cells[:, 1] >= 0) & (cells[:, 1] < info.height))
    known = np.zeros(len(points), dtype=bool)
    known[inside] = occupied[cells[inside, 1], cells[inside, 0]] >= 0
    matched = np.zeros(len(points), dtype=bool)
    radius = math.ceil(tolerance/info.resolution)
    for dy in range(-radius, radius+1):
        for dx in range(-radius, radius+1):
            if math.hypot(dx, dy)*info.resolution > tolerance:
                continue
            x, y = cells[:, 0]+dx, cells[:, 1]+dy
            mask = known & (x >= 0) & (x < info.width) & (y >= 0) & (y < info.height)
            matched[mask] |= occupied[y[mask], x[mask]] >= 65
    result['match_ratio'] = float(matched.mean())
    return result
