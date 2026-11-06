"""Pure, hardware-safe shaping for machine-dog velocity commands."""

import math


def shape_velocity_command(
    *,
    linear_x,
    linear_y,
    angular_z,
    linear_scale,
    angular_scale,
    allow_lateral,
    deadband,
    min_in_place_turn_command,
):
    """Scale, limit and stabilize a velocity command before UDP transport.

    The turn floor applies only after translation is inside the command
    deadband. A stop remains a stop, and an arc while walking retains Nav2's
    requested angular speed.
    """
    values = (linear_x, linear_y, angular_z, linear_scale, angular_scale)
    if not all(math.isfinite(float(value)) for value in values):
        return 0.0, 0.0, 0.0
    vx = max(-1.0, min(1.0, float(linear_x) * float(linear_scale)))
    vy = (
        max(-1.0, min(1.0, float(linear_y) * float(linear_scale)))
        if allow_lateral else 0.0
    )
    vtheta = max(-1.0, min(1.0, float(angular_z) * float(angular_scale)))

    command_deadband = max(0.0, float(deadband))
    if abs(vx) < command_deadband:
        vx = 0.0
    if abs(vy) < command_deadband:
        vy = 0.0
    if abs(vtheta) < command_deadband:
        vtheta = 0.0

    turn_floor = max(0.0, min(1.0, float(min_in_place_turn_command)))
    if vx == 0.0 and vy == 0.0 and 0.0 < abs(vtheta) < turn_floor:
        vtheta = turn_floor if vtheta > 0.0 else -turn_floor

    return vx, vy, vtheta


class TurnCommandShaper:
    """Hysteretic command deadbands followed by a timed angular slew limit.

    The floor is a target, never a post-smoothing clamp. Exact zero or release
    below the lower deadband stops that axis immediately. Call reset on every
    safety stop so neither a target nor hysteresis survives it. A nonzero
    turn_start explicitly opts a body into breakaway output for in-place turns.
    """

    def __init__(self, *, linear_scale=1.5, angular_scale=1.0,
                 allow_lateral=False, deadband=0.02, release_deadband=0.01,
                 turn_floor=0.25, angular_accel=0.4, turn_start=0.0):
        if not all(math.isfinite(float(value)) for value in (
                linear_scale, angular_scale, deadband, release_deadband,
                turn_floor, angular_accel, turn_start)):
            raise ValueError('velocity shaping parameters must be finite')
        if not (0 <= release_deadband < deadband <= 1 and
                0 <= turn_start <= turn_floor <= 1 and angular_accel > 0):
            raise ValueError('invalid velocity shaping thresholds or acceleration')
        self.linear_scale = linear_scale
        self.angular_scale = angular_scale
        self.allow_lateral = allow_lateral
        self.deadband = deadband
        self.release_deadband = release_deadband
        self.turn_start = turn_start
        self.turn_floor = turn_floor
        self.angular_accel = angular_accel
        self.reset()

    def reset(self):
        self._active = [False, False, False]
        self._target = (0.0, 0.0, 0.0)
        self._angular = 0.0
        self._in_place = False

    def set_target(self, linear_x, linear_y, angular_z):
        values = shape_velocity_command(
            linear_x=linear_x, linear_y=linear_y, angular_z=angular_z,
            linear_scale=self.linear_scale, angular_scale=self.angular_scale,
            allow_lateral=self.allow_lateral, deadband=0.0,
            min_in_place_turn_command=0.0)
        filtered = []
        for index, value in enumerate(values):
            if value == 0.0 or abs(value) <= self.release_deadband:
                self._active[index] = False
            elif abs(value) >= self.deadband:
                self._active[index] = True
            filtered.append(value if self._active[index] else 0.0)
        vx, vy, yaw = filtered
        self._in_place = vx == 0.0 and vy == 0.0 and yaw != 0.0
        if self._in_place:
            yaw = math.copysign(max(abs(yaw), self.turn_floor), yaw)
        self._target = (vx, vy, yaw)
        if yaw == 0.0:
            # A stop received between output ticks must clear ramp memory too.
            self._angular = 0.0

    def step(self, elapsed):
        vx, vy, target = self._target
        if target == 0.0:
            self._angular = 0.0
        elif self._in_place and self.turn_start > 0.0 and self._angular * target < 0.0:
            # Stop the old turn before applying breakaway in the new direction.
            self._angular = 0.0
        elif (self._in_place and abs(self._angular) < self.turn_start
              and self._angular * target >= 0.0):
            # Short command bursts must reach the body's breakaway threshold.
            self._angular = math.copysign(min(abs(target), self.turn_start), target)
        else:
            # Never compensate a scheduling stall with a large command jump.
            delta = self.angular_accel * max(0.0, min(float(elapsed), 0.1))
            self._angular += max(-delta, min(delta, target - self._angular))
        return vx, vy, self._angular
