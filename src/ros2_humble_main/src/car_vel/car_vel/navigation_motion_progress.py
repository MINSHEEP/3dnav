"""Observe commanded motion versus odometry without changing any command.

This is not execution acknowledgement or a replacement for Nav2's progress
checker. Distances are measured from a reference pose, not accumulated from
noisy per-frame deltas. A pose reset or loss of fresh data restarts evidence.
"""
import math


def _angle_difference(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


class NavigationMotionProgress:
    def __init__(self):
        self.warning_after_sec = 5.0
        self.odom_timeout_sec = 1.0
        self.translation_threshold_m = .03
        self.rotation_threshold_rad = .05
        self._sample = None
        self._epoch = 0
        self._invalid_reason = '尚未收到里程计'
        self.reset()

    def reset(self):
        self._context = None
        self._last_tick = None
        self._active_since = None
        self._baseline = None
        self._baseline_at = None

    def observe(self, *, now, stamp, frame, child_frame, x, y, yaw):
        try:
            values = tuple(float(v) for v in (now, stamp, x, y, yaw))
            now, stamp, x, y, yaw = values
            if not all(math.isfinite(v) for v in values) or stamp <= 0 or not frame or not child_frame:
                raise ValueError('invalid odometry')
        except (TypeError, ValueError, OverflowError):
            self._sample = None
            self._baseline = None
            self._baseline_at = None
            self._invalid_reason = '里程计位姿或时间戳无效'
            return
        previous = self._sample
        if previous:
            if (frame, child_frame) != (previous['frame'], previous['child_frame']):
                self._epoch += 1
            elif stamp == previous['stamp']:
                # Replayed messages cannot renew receipt freshness.
                self._invalid_reason = '里程计时间戳未推进'
                return
            elif stamp < previous['stamp'] or now < previous['received_at']:
                self._epoch += 1
            else:
                dt = stamp-previous['stamp']
                distance = math.hypot(x-previous['x'], y-previous['y'])
                rotation = _angle_difference(yaw, previous['yaw'])
                # Larger-than-physical jumps are resets, never proof of motion.
                if distance > max(.5, 2.*dt) or rotation > max(1., 3.*dt):
                    self._epoch += 1
        self._sample = dict(received_at=now, stamp=stamp, frame=str(frame),
            child_frame=str(child_frame), x=x, y=y, yaw=yaw)
        self._invalid_reason = ''

    def evaluate(self, *, now, ros_now, enabled, command, context):
        try:
            values = tuple(float(v) for v in command)
            motion_requested = len(values)==3 and all(math.isfinite(v) for v in values) and any(abs(v)>1e-6 for v in values)
        except (TypeError, ValueError, OverflowError):
            motion_requested = False
        if not enabled or not motion_requested:
            self.reset()
            return self._result('inactive')
        identity = (context, self._epoch)
        if (self._context != identity or self._last_tick is None
                or not 0. <= now-self._last_tick <= .3):
            self.reset()
            self._context = identity
            self._active_since = now
        self._last_tick = now
        sample = self._sample
        receipt_age = now-sample['received_at'] if sample else None
        source_age = ros_now-sample['stamp'] if sample else None
        fresh = (sample is not None and 0. <= receipt_age <= self.odom_timeout_sec
                 and -.05 <= source_age <= self.odom_timeout_sec)
        if not fresh:
            self._baseline = None
            self._baseline_at = None
            elapsed = max(0., now-self._active_since)
            detail = self._invalid_reason or '里程计缺失、时间戳异常或数据过期'
            return self._result('odometry_unavailable', duration=elapsed,
                warning=elapsed >= self.warning_after_sec,
                reason=f'已输出非零导航速度，但{detail}，无法判断机器人是否移动',
                receipt_age=receipt_age, source_age=source_age)
        if self._baseline is None:
            self._baseline = sample
            self._baseline_at = now
        translation = math.hypot(sample['x']-self._baseline['x'], sample['y']-self._baseline['y'])
        rotation = _angle_difference(sample['yaw'], self._baseline['yaw'])
        elapsed = max(0., now-self._baseline_at)
        if translation >= self.translation_threshold_m or rotation >= self.rotation_threshold_rad:
            self._baseline = sample
            self._baseline_at = now
            return self._result('progress_observed', duration=elapsed,
                reason='里程计已检测到位姿变化；不代表已到达目标',
                translation=translation, rotation=rotation,
                receipt_age=receipt_age, source_age=source_age)
        warning = elapsed >= self.warning_after_sec
        return self._result('no_motion_observed' if warning else 'observing',
            duration=elapsed, warning=warning,
            reason=(f'已持续输出非零导航速度 {elapsed:.1f} 秒，但里程计未检测到明显移动；'
                    '请检查本体控制和低速响应，系统不会因此自动增速') if warning else '',
            translation=translation, rotation=rotation,
            receipt_age=receipt_age, source_age=source_age)

    def _result(self, state, *, duration=0., warning=False, reason='',
                translation=None, rotation=None, receipt_age=None, source_age=None):
        def rounded(value):
            return round(value, 3) if value is not None and math.isfinite(value) else None
        return dict(state=state, warning=warning, warning_code=state if warning else '',
            reason=reason, read_only=True, duration_sec=rounded(duration),
            translation_m=rounded(translation), rotation_rad=rounded(rotation),
            odom_age_sec=rounded(receipt_age), odom_source_age_sec=rounded(source_age),
            warning_after_sec=self.warning_after_sec,
            translation_threshold_m=self.translation_threshold_m,
            rotation_threshold_rad=self.rotation_threshold_rad)
