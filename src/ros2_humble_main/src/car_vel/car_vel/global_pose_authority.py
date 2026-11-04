"""Measured rigid-3D map anchors and unchanged legacy planar operations.

LIO supplies relative motion. Only verified scan/map matches may change its
global anchor. Never interpolate a rejected correction into an accepted one.
"""
from dataclasses import dataclass
import math
from .rigid_pose import Pose3, compose3, inverse3

RUNTIME_TIMING_MODEL = 'xingji_3d_runtime_timing_v1'


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    yaw: float


def compose(a, b):
    if isinstance(a, Pose3) or isinstance(b, Pose3):
        return compose3(a, b)
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2(a.x + c*b.x - s*b.y, a.y + s*b.x + c*b.y, angle(a.yaw+b.yaw))


def inverse(a):
    if isinstance(a, Pose3):
        return inverse3(a)
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


class LioStreamGuard:
    """One temporal recovery path; never turn a stale TF into fresh evidence."""
    def __init__(self, max_gap=1.5, recovery=3.0, *, freshness_sec=.3):
        self.max_gap, self.recovery = max_gap, recovery
        if not math.isfinite(freshness_sec) or not .3 <= freshness_sec <= .45:
            raise ValueError('LIO freshness must be between .3 and .45 seconds')
        self.freshness_sec = freshness_sec
        self.reset()

    def reset(self):
        self.pose = self.stamp = self.received = None
        self.deadline = None
        self.samples = 0
        self.fresh = self.failed = False
        self.reason = 'waiting_for_tf'
        self.source_age = self.receipt_age = None
        self.dt = self.xy = self.yaw = None
        self.lookup_error = ''
        self.requires_match = False
        self.match_min_stamp = 0.

    @property
    def recovering(self):
        return self.deadline is not None and not self.failed

    @property
    def stream_ready(self):
        return self.fresh and not self.failed and (not self.recovering or self.samples >= 3)

    def fail(self, reason):
        self.failed, self.fresh, self.reason = True, False, reason
        return self

    def _hold(self, wall, anchored, reason):
        if anchored and self.stamp is not None:
            if self.deadline is None:
                self.deadline = wall+self.recovery
                self.samples = 0
            self.reason = reason
            if reason == 'sample_gap':
                self.requires_match = True

    def _age(self, now, wall, anchored):
        self.source_age = now-self.stamp if self.stamp is not None else None
        self.receipt_age = wall-self.received if self.received is not None else None
        if self.deadline is not None and wall > self.deadline:
            self.fail('sensor_recovery_timeout')
        elif anchored and self.stamp is not None and (
                self.source_age > self.max_gap or self.receipt_age > self.max_gap):
            self.fail('source_timeout')

    def missing(self, now, wall, anchored, error=''):
        self.lookup_error = str(error)
        self.fresh = False
        # A brief transport hiccup blocks motion, but does not erase real
        # samples already verified in this same, fixed recovery window.
        self._hold(wall, anchored, 'tf_lookup_missing')
        self._age(now, wall, anchored)
        return self

    def observe(self, pose, stamp, now, wall, anchored):
        if self.failed:
            return self
        if not all(math.isfinite(v) for v in (pose.x,pose.y,pose.yaw,stamp,now,wall)):
            return self.fail('nonfinite_lio')
        new = self.stamp is None or stamp != self.stamp
        if self.pose is not None and new:
            self.dt = stamp-self.stamp
            self.xy = math.hypot(pose.x-self.pose.x,pose.y-self.pose.y)
            self.yaw = abs(angle(pose.yaw-self.pose.yaw))
            if self.dt <= 0:
                return self.fail('lio_time_reversed')
            if self.dt <= .5:
                if not lio_step_valid(self.pose,pose,self.dt):
                    return self.fail('lio_pose_jump')
            else:
                # Movement is bounded by the short pre-stop interval, not by
                # the whole missing period. Do not allow metre-scale resets.
                if self.dt > self.max_gap:
                    return self.fail('source_gap_exceeded')
                if self.xy > .30 or self.yaw > math.radians(20):
                    return self.fail('lio_gap_pose_jump')
                self._hold(wall,anchored,'sample_gap')
                self.match_min_stamp = stamp
        if new:
            self.pose,self.stamp,self.received = pose,stamp,wall
        self._age(now,wall,anchored)
        if self.failed:
            return self
        self.fresh = (-.1 <= self.source_age <= self.freshness_sec
                      and 0 <= self.receipt_age <= self.freshness_sec)
        if self.source_age < -.1 or self.receipt_age < 0:
            return self.fail('lio_clock_discontinuity')
        if not self.fresh:
            self._hold(wall,anchored,'tf_stale')
        elif self.recovering and new:
            self.samples += 1
        if self.fresh:
            self.lookup_error = ''
            if self.recovering and self.samples >= 3:
                # Temporal evidence is complete. The authority separately
                # retains any map-match requirement and its original budget.
                self.deadline = None
            self.reason = 'verifying_stream' if self.recovering else 'ready'
        return self

    def finish(self):
        self.deadline = None
        self.samples = 0
        self.requires_match = False
        self.match_min_stamp = 0.
        self.reason = 'ready'

    def details(self):
        def number(v):
            return round(v,6) if v is not None and math.isfinite(v) else None
        return dict(state='failed' if self.failed else self.reason,
            freshness_limit_sec=self.freshness_sec,
            failure_reason=self.reason if self.failed else '',
            source_age_sec=number(self.source_age),receipt_age_sec=number(self.receipt_age),
            step_dt_sec=number(self.dt),step_xy_m=number(self.xy),
            step_yaw_deg=number(math.degrees(self.yaw) if self.yaw is not None else None),
            fresh_samples=self.samples,lookup_error=self.lookup_error)


class LioAnchorContinuity:
    """Keep an anchor across a bounded, explicitly rolled-back quality scan.

    This authorizes only localization continuity. /lio/motion_paused still
    stops velocity independently, including throughout the healthy-scan check.
    Recovery messages alone never establish continuity after startup/fault.
    """
    def __init__(self, max_pause_sec=1.0):
        self.max_pause_sec = max_pause_sec
        self.since = None
        self.usable = False
        self.paused = False

    def allows_anchor(self, now):
        if self.since is not None and not 0 <= now-self.since <= self.max_pause_sec:
            self.since, self.paused, self.usable = None, False, False
        return self.usable

    def observe(self, data, now):
        if self.since is not None and not self.allows_anchor(now):
            return False
        valid = isinstance(data, dict) and data.get('hard_stop') is not True
        if valid and usable_lio_health(data):
            self.since, self.paused, self.usable = None, False, True
            return True
        rollback = (valid and data.get('state') == 'warning'
            and data.get('code') == 'odometry_sample_quarantined'
            and data.get('reason') == 'scan_matching_quality'
            and data.get('sample_quarantined') is True
            and data.get('temporary_stop') is True
            and data.get('hard_stop') is False
            and data.get('recovery_only') is True
            and data.get('has_planar_or_yaw_corroboration') is False)
        # A fixed per-sample step gate can quarantine ordinary slow motion
        # after a longer scan interval (recorded dt=.499s, speed=.344m/s).
        # Keep the anchor briefly only when the sample was explicitly excluded
        # and all independent speed diagnostics remain in the body envelope.
        rates = (('pose_linear_speed',.6),('estimator_linear_speed',.6),
                 ('pose_angular_speed',.8),('estimator_angular_speed',.8),
                 ('pose_vertical_speed',.3),('estimator_vertical_speed',.3))
        count = data.get('consecutive_samples') if valid else None
        required_count = data.get('required_samples') if valid else None
        slow_step = (valid and data.get('state') == 'warning'
            and data.get('code') == 'odometry_sample_quarantined'
            and data.get('reason') in ('position_step','yaw_step')
            and data.get('sample_quarantined') is True
            and data.get('temporary_stop') is True and data.get('hard_stop') is False
            and type(count) is int and type(required_count) is int
            and 1 <= count < required_count <= 16
            and all(type(data.get(k)) in (int,float) and math.isfinite(data[k])
                    and 0 <= data[k] <= limit for k,limit in rates))
        samples = data.get('healthy_samples') if valid else None
        required = data.get('required_samples') if valid else None
        recovery = (valid and data.get('state') == 'recovering'
            and data.get('code') == 'lio_auto_recovery'
            and type(samples) is int and type(required) is int
            and 1 <= samples <= required <= 100
            and self.since is not None)
        if self.usable and (rollback or slow_step or recovery):
            if self.since is None:
                self.since = now
            if 0 <= now-self.since <= self.max_pause_sec:
                self.paused = True
                return True
        self.since, self.paused, self.usable = None, False, False
        return False


class GlobalPoseAuthority:
    def __init__(self, max_xy=.20, max_yaw=math.radians(6), max_age=1.0,
                 coast=3.0, recovery=2.0, tracking_rmse=.22,
                 max_height=.45, max_tilt=math.radians(8)):
        self.max_xy, self.max_yaw = max_xy, max_yaw
        self.max_height, self.max_tilt = max_height, max_tilt
        self.max_age, self.coast = max_age, coast
        self.recovery, self.tracking_rmse = recovery, tracking_rmse
        self.recovery_matches_required = 1
        self.identity = None
        self.reset()

    def reset(self):
        self.transform = None
        self.accepted_stamp = 0.
        self.reason = 'initialization_required'
        self.rejected = 0
        self.rmse = self.overlap = None
        self.recovery_deadline = None
        self.recovery_min_stamp = 0.
        self.recovery_samples = 0
        self.recovery_requires_matches = True
        self.hard_failure = False

    def set_identity(self, identity):
        if identity != self.identity:
            self.identity = identity
            self.reset()

    def accept(self, mapped, odom, *, stamp, now, rmse, overlap,
               ambiguous, initial=False):
        # Late results cannot overwrite a latched failure with "accepted".
        # Only an explicit reset/initialization can establish a new identity.
        if self.hard_failure:
            return False
        if self.recovery_deadline is not None and now > self.recovery_deadline:
            self.reason = 'recovery_timeout'
            self.hard_failure = True
            return False
        values = (mapped.x, mapped.y, mapped.yaw, odom.x, odom.y,
                  odom.yaw, stamp, now, rmse, overlap)
        values += tuple(getattr(pose, key, 0.) for pose in (mapped, odom)
                        for key in ('z', 'roll', 'pitch'))
        reason = ''
        correction_xy = correction_yaw = correction_height = correction_tilt = 0.
        if not all(math.isfinite(v) for v in values):
            reason = 'nonfinite'
        elif stamp <= self.accepted_stamp or now-stamp > self.max_age or stamp-now > .1:
            reason = 'timestamp_invalid'
        elif (rmse < 0 or rmse > (.20 if initial else self.tracking_rmse)+1e-6
              or not .55-1e-6 <= overlap <= 1 or ambiguous
              or (initial and (abs(getattr(mapped, 'roll', 0.)) > self.max_tilt
                               or abs(getattr(mapped, 'pitch', 0.)) > self.max_tilt))):
            reason = 'quality_failed'
        elif self.transform is None and not initial:
            reason = 'initialization_required'
        elif self.transform is not None and not initial:
            predicted = compose(self.transform, odom)
            correction_xy = math.hypot(mapped.x-predicted.x, mapped.y-predicted.y)
            correction_yaw = abs(angle(mapped.yaw-predicted.yaw))
            correction_height = abs(getattr(mapped, 'z', 0.)-getattr(predicted, 'z', 0.))
            correction_tilt = math.hypot(
                angle(getattr(mapped, 'roll', 0.)-getattr(predicted, 'roll', 0.)),
                angle(getattr(mapped, 'pitch', 0.)-getattr(predicted, 'pitch', 0.)))
            if (correction_xy > self.max_xy or correction_yaw > self.max_yaw
                    or correction_height > self.max_height or correction_tilt > self.max_tilt):
                reason = 'innovation_exceeded'
        if reason:
            self.reason = reason
            self.rejected += 1
            self.recovery_samples = 0
            if (reason == 'nonfinite' or ambiguous or (reason == 'innovation_exceeded'
                    and (correction_xy > 2*self.max_xy or correction_yaw > 2*self.max_yaw
                         or correction_height > 2*self.max_height
                         or correction_tilt > 2*self.max_tilt))):
                self.hard_failure = True
            elif reason == 'innovation_exceeded':
                # Reject this candidate without moving the anchor. A small
                # overshoot gets a new verified recheck, never a clamped pose.
                self.begin_recovery(now, self.recovery, stamp)
            return False
        self.transform = between(odom, mapped)
        self.accepted_stamp = stamp
        self.rmse, self.overlap = rmse, overlap
        self.reason = 'accepted'
        if (self.recovery_deadline is not None and not initial
                and stamp >= self.recovery_min_stamp):
            self.recovery_samples += 1
        return True

    def ready(self, now):
        return (self.transform is not None and not self.hard_failure
                and -.1 <= now-self.accepted_stamp <= self.coast)

    def match_failed(self):
        self.recovery_samples = 0

    def begin_recovery(self, now, duration, min_stamp, *, require_matches=True):
        deadline = min(now+duration,self.accepted_stamp+self.coast+self.recovery)
        if self.recovery_deadline is None:
            self.recovery_min_stamp = min_stamp
            self.recovery_samples = 0
            self.recovery_requires_matches = require_matches
        elif require_matches and not self.recovery_requires_matches:
            self.recovery_requires_matches = True
            self.recovery_min_stamp = min_stamp
            self.recovery_samples = 0
        self.recovery_deadline = (deadline if self.recovery_deadline is None
                                  else min(self.recovery_deadline,deadline))

    def phase(self, now, *, can_resume=True):
        """Short match loss holds motion; it never extends a fixed deadline."""
        if self.transform is None or self.hard_failure or now-self.accepted_stamp < -.1:
            return 'failed'
        if self.recovery_deadline is not None:
            if now > self.recovery_deadline:
                self.reason = 'recovery_timeout'
                self.hard_failure = True
                return 'failed'
            if (can_resume and self.ready(now)
                    and (not self.recovery_requires_matches
                         or self.recovery_samples >= self.recovery_matches_required)):
                self.recovery_deadline = None
                self.recovery_samples = 0
                return 'ready'
            return 'recovering'
        if self.ready(now):
            return 'ready'
        deadline = self.accepted_stamp+self.coast+self.recovery
        if now <= deadline:
            self.begin_recovery(now, deadline-now, now)
            return 'recovering'
        return 'failed'
