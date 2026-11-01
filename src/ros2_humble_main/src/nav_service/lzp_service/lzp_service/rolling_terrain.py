"""Bounded recent *measured* floor samples in the continuous odom frame."""
import math
import numpy as np
from .terrain_observer import estimate_local_terrain


class RollingGroundMemory:
    def __init__(self, *, max_age_seconds=2.0, resolution=.12, maximum_cells=2000):
        self.max_age_seconds = max(.1, min(3., float(max_age_seconds)))
        self.resolution = float(resolution)
        self.maximum_cells = int(maximum_cells)
        self.clear()

    def clear(self):
        self.cells = {}
        self.last_pose = None
        self.last_stamp = None

    def _prune(self, stamp, x, y):
        self.cells = {key: value for key, value in self.cells.items()
                      if 0 <= stamp - value[0] <= self.max_age_seconds
                      and math.hypot(key[0] * self.resolution - x,
                                     key[1] * self.resolution - y) <= 3.0}
        if len(self.cells) > self.maximum_cells:
            newest = sorted(self.cells, key=lambda k: self.cells[k][0], reverse=True)
            self.cells = {k: self.cells[k] for k in newest[:self.maximum_cells]}

    def update(self, points, *, stamp_seconds, robot_x, robot_y, yaw, ground_anchor=None):
        stamp = float(stamp_seconds)
        if self.last_stamp is not None:
            dt = stamp - self.last_stamp
            old_x, old_y, old_yaw = self.last_pose
            if (dt < 0 or dt > self.max_age_seconds
                    or math.hypot(robot_x-old_x, robot_y-old_y) > max(.25, dt)
                    or abs(math.remainder(yaw-old_yaw, 2*math.pi)) > max(.35, 1.5*dt)):
                self.clear()
            elif dt == 0:
                return
        self.last_stamp = stamp
        self.last_pose = (robot_x, robot_y, yaw)
        self._prune(stamp, robot_x, robot_y)
        values = np.asarray(points, dtype=np.float32)
        values = values[np.isfinite(values).all(axis=1)]
        values = values[np.hypot(values[:, 0]-robot_x, values[:, 1]-robot_y) <= 3.0]
        groups = {}
        for point in values:
            key = tuple(np.floor(point[:2] / self.resolution).astype(int))
            groups.setdefault(key, []).append(point)
        samples = []
        for key, rows in groups.items():
            rows = np.asarray(rows)
            floor = rows[rows[:, 2] <= float(np.min(rows[:, 2])) + .04]
            if len(floor) >= 3:
                samples.append((key, floor[:4]))
        if len(samples) < 8:
            return
        centres = np.asarray([np.median(p, axis=0) for _, p in samples])
        design = np.column_stack((centres[:, :2], np.ones(len(centres))))
        coefficients, *_ = np.linalg.lstsq(design, centres[:, 2], rcond=None)
        residual = centres[:, 2] - design @ coefficients
        keep = np.abs(residual-np.median(residual)) <= .06
        if np.count_nonzero(keep) < 8:
            return
        coefficients, *_ = np.linalg.lstsq(design[keep], centres[keep, 2], rcond=None)
        if ground_anchor is not None:
            ax, ay, az = map(float, ground_anchor)
            height = coefficients[0]*ax + coefficients[1]*ay + coefficients[2]
            if (not all(math.isfinite(v) for v in (ax, ay, az, height))
                    or abs(height - az) > .15):
                self.clear()
                return
        if abs(math.degrees(math.atan(coefficients[0]))) > 18 or abs(
                math.degrees(math.atan(coefficients[1]))) > 18:
            return
        keep = np.abs(centres[:, 2] - design @ coefficients) <= .06
        for accepted, (key, floor) in zip(keep, samples):
            if accepted:
                self.cells[key] = (stamp, floor.copy())
        self._prune(stamp, robot_x, robot_y)

    def observe(self, points, *, stamp_seconds, robot_x, robot_y, yaw, **geometry):
        self._prune(float(stamp_seconds), robot_x, robot_y)
        current = estimate_local_terrain(points, robot_x=robot_x,
                                         robot_y=robot_y, yaw=yaw, **geometry)
        # Newly measured hazards always outrank historical floor evidence.
        if current.terrain_type in ('blocked', 'negative_obstacle'):
            return current
        if not self.cells:
            return current
        history = np.concatenate([value[1] for value in self.cells.values()])
        return estimate_local_terrain(np.vstack((points, history)),
            robot_x=robot_x, robot_y=robot_y, yaw=yaw, **geometry)


def matched_pose(samples, stamp_seconds, *, max_skew_seconds=.10):
    """Interpolate time-tagged (stamp,x,y,yaw) odometry; reject distant poses."""
    if not samples:
        return None
    ordered = sorted(samples)
    t = float(stamp_seconds)
    for left, right in zip(ordered[:-1], ordered[1:]):
        if left[0] <= t <= right[0] and right[0] - left[0] <= 2*max_skew_seconds:
            f = (t-left[0]) / max(1e-9, right[0]-left[0])
            return (left[1]+f*(right[1]-left[1]), left[2]+f*(right[2]-left[2]),
                    left[3]+f*math.remainder(right[3]-left[3], 2*math.pi))
    nearest = min(ordered, key=lambda p: abs(p[0]-t))
    return nearest[1:] if abs(nearest[0]-t) <= max_skew_seconds else None
