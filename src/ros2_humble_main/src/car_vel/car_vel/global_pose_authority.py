"""ROS-free acceptance of measured 3D corrections in the planar Nav2 frame.

LIO supplies relative motion. Only verified scan/map matches may change its
global anchor. Never interpolate a rejected correction into an accepted one.
"""
from dataclasses import dataclass
import math


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def compose(a, b):
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2(a.x + c*b.x - s*b.y, a.y + s*b.x + c*b.y, angle(a.yaw+b.yaw))


def inverse(a):
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2(-c*a.x-s*a.y, s*a.x-c*a.y, -a.yaw)


def between(odom, mapped):
    return compose(mapped, inverse(odom))


def lio_step_valid(previous, current, dt):
    """An independent reset/jump check above the configured body speed limits."""
    return (0 < dt <= .5 and all(math.isfinite(v) for v in (
        previous.x, previous.y, previous.yaw, current.x, current.y, current.yaw))
        and math.hypot(current.x-previous.x, current.y-previous.y) <= .15+1.2*dt
        and abs(angle(current.yaw-previous.yaw)) <= .20+1.5*dt)


def usable_lio_health(data):
    # PointLIO explicitly discards this sample; it is not an estimator reset.
    # Fresh accepted TF and independent odometry checks are still required.
    return (data.get('state') == 'healthy' or (
        data.get('state') == 'warning' and data.get('code') == 'odometry_sample_quarantined'
        and data.get('sample_quarantined') is True
        and data.get('temporary_stop') is False and data.get('hard_stop') is False))


class GlobalPoseAuthority:
    def __init__(self, max_xy=.20, max_yaw=math.radians(6), max_age=1.0,
                 coast=2.0):
        self.max_xy, self.max_yaw = max_xy, max_yaw
        self.max_age, self.coast = max_age, coast
        self.identity = None
        self.reset()

    def reset(self):
        self.transform = None
        self.accepted_stamp = 0.
        self.reason = 'initialization_required'
        self.rejected = 0
        self.rmse = self.overlap = None

    def set_identity(self, identity):
        if identity != self.identity:
            self.identity = identity
            self.reset()

    def accept(self, mapped, odom, *, stamp, now, rmse, overlap,
               ambiguous, initial=False):
        values = (mapped.x, mapped.y, mapped.yaw, odom.x, odom.y,
                  odom.yaw, stamp, now, rmse, overlap)
        reason = ''
        if not all(math.isfinite(v) for v in values):
            reason = 'nonfinite'
        elif stamp <= self.accepted_stamp or now-stamp > self.max_age or stamp-now > .1:
            reason = 'timestamp_invalid'
        elif rmse < 0 or rmse > .20+1e-6 or not .55-1e-6 <= overlap <= 1 or ambiguous:
            reason = 'quality_failed'
        elif self.transform is None and not initial:
            reason = 'initialization_required'
        elif self.transform is not None and not initial:
            predicted = compose(self.transform, odom)
            if (math.hypot(mapped.x-predicted.x, mapped.y-predicted.y) > self.max_xy
                    or abs(angle(mapped.yaw-predicted.yaw)) > self.max_yaw):
                reason = 'innovation_exceeded'
        if reason:
            self.reason = reason
            self.rejected += 1
            return False
        self.transform = between(odom, mapped)
        self.accepted_stamp = stamp
        self.rmse, self.overlap = rmse, overlap
        self.reason = 'accepted'
        return True

    def ready(self, now):
        return (self.transform is not None
                and -.1 <= now-self.accepted_stamp <= self.coast)
