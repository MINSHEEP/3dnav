"""Rigid geometry for verified Xingji 3D localization; no ROS dependency."""
from dataclasses import dataclass
import math


TRANSFORM_MODEL = 'se3_map_odom_v1'


def quaternion(pose):
    cr, sr = math.cos(getattr(pose, 'roll', 0.)/2), math.sin(getattr(pose, 'roll', 0.)/2)
    cp, sp = math.cos(getattr(pose, 'pitch', 0.)/2), math.sin(getattr(pose, 'pitch', 0.)/2)
    cy, sy = math.cos(pose.yaw/2), math.sin(pose.yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def multiply(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return (w*u+x*t+y*s-z*v, w*v-x*s+y*t+z*u,
            w*s+x*v-y*u+z*t, w*t-x*u-y*v-z*s)


def rotate(q, xyz):
    x, y, z, w = q
    u, v, s = xyz
    tx, ty, tz = 2*(y*s-z*v), 2*(z*u-x*s), 2*(x*v-y*u)
    return (u+w*tx+y*tz-z*ty, v+w*ty+z*tx-x*tz, s+w*tz+x*ty-y*tx)


@dataclass(frozen=True)
class Pose3:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    @classmethod
    def from_quaternion(cls, x, y, z, qx, qy, qz, qw):
        values = (x, y, z, qx, qy, qz, qw)
        norm = qx*qx+qy*qy+qz*qz+qw*qw
        if not all(math.isfinite(v) for v in values) or abs(norm-1.) > .02:
            raise ValueError('Rigid pose contains invalid coordinates or rotation')
        length = math.sqrt(norm)
        qx, qy, qz, qw = (v/length for v in (qx, qy, qz, qw))
        return cls(x, y, z,
            math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)),
            math.asin(max(-1., min(1., 2*(qw*qy-qz*qx)))),
            math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)))


def compose3(a, b):
    qa, qb = quaternion(a), quaternion(b)
    x, y, z = rotate(qa, (b.x, b.y, getattr(b, 'z', 0.)))
    return Pose3.from_quaternion(a.x+x, a.y+y, getattr(a, 'z', 0.)+z,
                                  *multiply(qa, qb))


def inverse3(a):
    x, y, z, w = quaternion(a)
    q = (-x, -y, -z, w)
    xyz = rotate(q, (-a.x, -a.y, -getattr(a, 'z', 0.)))
    return Pose3.from_quaternion(*xyz, *q)
