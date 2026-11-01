"""Pure velocity ownership policy shared by tests and the ROS arbiter."""

import math


def select_velocity_source(
        *, connector_active, emergency, nav_fresh, campus_fresh,
        teleop_fresh=False):
    """Return the one velocity owner allowed by the current safety state."""
    if emergency:
        return 'zero'
    if connector_active:
        return 'campus' if campus_fresh else 'zero'
    if teleop_fresh:
        return 'teleop'
    return 'nav' if nav_fresh else 'zero'


def should_stop_for_terrain_update(
        *, terrain_safe, connector_active, teleop_fresh,
        terrain_guard_enabled=True):
    """Return whether an unsafe terrain update must immediately stop Nav2."""
    return (
        bool(terrain_guard_enabled) and not terrain_safe and
        not connector_active and not teleop_fresh)


def constrain_navigation_velocity(
        linear_x, linear_y, angular_z, *, terrain_safe, speed_limit_mps,
        terrain_guard_enabled=True, restricted_flat_site=False, forward_only=False):
    """Stop unsafe Nav2 commands or clamp their planar speed."""
    try:
        x_value = float(linear_x)
        y_value = float(linear_y)
        yaw_value = float(angular_z)
    except (TypeError, ValueError, OverflowError):
        return 0.0, 0.0, 0.0
    if not all(math.isfinite(value) for value in (
            x_value, y_value, yaw_value)):
        return 0.0, 0.0, 0.0
    if not terrain_guard_enabled:
        return (
            round(x_value, 12), round(y_value, 12),
            round(yaw_value, 12))
    if not terrain_safe:
        return 0.0, 0.0, 0.0
    if restricted_flat_site or forward_only:
        # The map-prior corridor is forward-facing. Do not authorize sideways
        # or reverse motion using evidence gathered for another direction.
        if x_value < -1e-6 or abs(y_value) > 1e-6:
            return 0.0, 0.0, 0.0
        yaw_value = max(-0.25, min(0.25, yaw_value))
    try:
        limit = float(speed_limit_mps)
    except (TypeError, ValueError, OverflowError):
        return 0.0, 0.0, 0.0
    if not math.isfinite(limit):
        return 0.0, 0.0, 0.0
    limit = max(0.0, limit)
    if restricted_flat_site:
        limit = min(limit, 0.10)
    magnitude = math.hypot(x_value, y_value)
    if magnitude > limit and magnitude > 0.0:
        scale = limit / magnitude
        x_value *= scale
        y_value *= scale
    return (
        round(x_value, 12), round(y_value, 12),
        round(yaw_value, 12))
