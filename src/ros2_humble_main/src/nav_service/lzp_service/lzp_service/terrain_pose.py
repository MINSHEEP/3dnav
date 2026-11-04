"""Time-aligned map pose from a real AMCL sample and bounded LIO evidence."""
from dataclasses import dataclass
import math
from .rolling_terrain import matched_pose


@dataclass(frozen=True)
class MappedPose:
    x: float
    y: float
    yaw: float
    position_sigma: float
    yaw_sigma: float
    source_age: float


def propagate_localization(pose, *, source_stamp, capture_stamp, odometry,
                           position_sigma, yaw_sigma):
    """Propagate at most two seconds; never rewrite the AMCL source stamp."""
    age = float(capture_stamp)-float(source_stamp)
    values = (*pose, age, position_sigma, yaw_sigma)
    if (not all(math.isfinite(float(v)) for v in values)
            or not 0 <= age <= 2. or position_sigma < 0 or yaw_sigma < 0):
        return None
    then = matched_pose(odometry, source_stamp)
    now = matched_pose(odometry, capture_stamp)
    if then is None or now is None or not all(math.isfinite(v) for v in (*then, *now)):
        return None
    dx, dy = now[0]-then[0], now[1]-then[1]
    turn = math.remainder(now[2]-then[2], 2*math.pi)
    distance = math.hypot(dx, dy)
    if distance > max(.10, .5*age) or abs(turn) > max(.35, .5*age):
        return None
    sigma = position_sigma + .02*age + .10*distance
    yaw_error = yaw_sigma + math.radians(2)*age + .10*abs(turn)
    if sigma > .20 or yaw_error > math.radians(10):
        return None
    angle = pose[2]-then[2]
    c, s = math.cos(angle), math.sin(angle)
    return MappedPose(pose[0]+c*dx-s*dy, pose[1]+s*dx+c*dy,
                      pose[2]+turn, sigma, yaw_error, age)
