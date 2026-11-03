"""Read-only, bounded diagnostics; these functions never authorize motion."""
import json
import math
from collections import deque


REASONS = {
    'global_localization_recovering': '定位数据流或三维匹配正在限时复核，零速等待，原任务保留',
    'global_localization_untrusted': '统一三维定位未确认或过期，已零速停车；请重新确认初始点并预览',
    'idle':'无活动导航任务',
    'stop_latched':'停车锁已锁定',
    'lio_paused_or_stale':'LIO暂停或状态过期',
    'navigation_paused_or_stale':'导航恢复暂停或状态过期',
    'other_source_owns_output':'速度由遥控或园区连接段占用',
    'upstream_missing':'尚未收到导航控制器速度',
    'upstream_stale':'导航控制器速度已过期',
    'upstream_discarded':'上一次导航速度已被状态切换清除，需要新的控制器速度',
    'upstream_zero':'导航控制器当前给出零速，不能仅据此判断TF或障碍根因',
    'input_invalid':'输入速度无效',
    'body_identity_invalid':'3D本体身份或状态校验未通过',
    'terrain_status_stale':'地形保护状态已过期',
    'terrain_unsafe':'地形保护未放行，详见terrain原因',
    'ground_evidence_unanchored':'地面证据未通过高度锚定',
    'direction_not_authorized':'当前地面证据不允许倒退或横移',
    'speed_limit_zero':'地形限速为零',
    'limited':'速度已限幅并发布给本体层，不代表本体已运动',
    'forwarded':'速度已发布给本体层，不代表本体已运动',
    'output_zero_unexplained':'最终速度为零，请对照全部门禁字段',
    'body_switching':'本体切换锁定中',
    'mapping_lio_paused':'建图LIO保护暂停',
    'remote_owns_body':'本体由独立遥控会话控制',
    'body_status_stale':'本体反馈过期，运动授权可能已清除',
    'body_not_standing':'本体反馈未确认站立',
    'gait_feedback_invalid':'越障模式或碰撞停止配置反馈未确认',
    'motion_not_armed':'本体没有有效运动授权，需人工确认后重新初始化',
    'safe_command_missing':'本体尚未收到最终速度',
    'safe_command_stale':'本体收到的最终速度已过期',
    'safe_command_discarded':'本体已清除上一条速度的执行资格，需要新的有效速度',
    'received_zero':'本体收到零速指令',
    'udp_send_failed':'UDP发送返回失败',
    'udp_send_not_observed':'本周期尚未观察到UDP速度发送',
    'udp_dispatched_unconfirmed':'UDP速度已发送，协议无逐条执行确认；不等于本体已运动',
}


def age(now, at):
    if at is None or at <= 0:
        return None
    value=now-at
    return round(value, 3) if math.isfinite(value) else None


def velocity(message):
    return [finite(getattr(message.linear, 'x', 0)),
            finite(getattr(message.linear, 'y', 0)),
            finite(getattr(message.angular, 'z', 0))]


def finite(value):
    try:
        value=float(value)
        return round(value, 6) if math.isfinite(value) else None
    except (ValueError, TypeError, OverflowError):
        return None


def moving(values):
    return any(v is not None and abs(v)>1e-6 for v in values)


def arbiter_reason(s):
    if s['emergency']:return 'stop_latched'
    if not s['lio_ready']:return 'lio_paused_or_stale'
    if not s['navigation_ready']:return 'navigation_paused_or_stale'
    if s.get('global_localization_blocked'):
        data=s.get('global_localization') or {}
        status_age=s.get('global_localization_age_sec')
        delayed=(data.get('ready') is True and type(status_age) in (float,int) and .3 < status_age <= 1.5)
        return ('global_localization_recovering' if data.get('recovering') is True or delayed
                else 'global_localization_untrusted')
    if s.get('connector_active') or s['source'] not in ('nav','zero'):return 'other_source_owns_output'
    if s.get('terrain_stop_bypassed') and not s.get('navigation_active'):return 'idle'
    if not s['input_seen']:return 'upstream_missing'
    if s.get('input_discarded'):return 'upstream_discarded'
    if s['input_age_sec'] is None or s['input_age_sec']>s['command_timeout_sec']:return 'upstream_stale'
    if any(v is None for v in s['input']):return 'input_invalid'
    if s['terrain_enabled']:
        if not s['body_known']:return 'body_identity_invalid'
        terrain_stop_bypassed = s.get('terrain_stop_bypassed', False)
        if not terrain_stop_bypassed:
            if not s['terrain_fresh']:return 'terrain_status_stale'
            if not s['terrain_safe']:return 'terrain_unsafe'
        if s['body_id']=='xingji':
            if (not terrain_stop_bypassed
                    and s['evidence'] not in ('current_anchored','recent_measured_anchored','restricted_map')):
                return 'ground_evidence_unanchored'
            if s['input'][0]<-1e-6 or abs(s['input'][1])>1e-6:return 'direction_not_authorized'
        if s['speed_limit_mps']<=0 and moving(s['input'][:2]):return 'speed_limit_zero'
    if not moving(s['input']):return 'upstream_zero'
    if not moving(s['output']):return 'output_zero_unexplained'
    if any(abs(a-b)>1e-6 for a,b in zip(s['input'],s['output'])):return 'limited'
    return 'forwarded'


def body_reason(s):
    if s['frozen']:return 'body_switching'
    if s['mapping_paused']:return 'mapping_lio_paused'
    if s['remote_active']:return 'remote_owns_body'
    if s['emergency']:return 'stop_latched'
    if not s['status_fresh']:return 'body_status_stale'
    if not s['standing']:return 'body_not_standing'
    if not s['gait_ready']:return 'gait_feedback_invalid'
    if not s['armed']:return 'motion_not_armed'
    if not s['input_seen']:return 'safe_command_missing'
    if s.get('command_discarded'):return 'safe_command_discarded'
    if s['input_age_sec'] is None or s['input_age_sec']>.3:return 'safe_command_stale'
    if not moving(s['input']):return 'received_zero'
    if s['send_age_sec'] is None or s['send_age_sec']>.3:return 'udp_send_not_observed'
    if s['send_success'] is False:return 'udp_send_failed'
    return 'udp_dispatched_unconfirmed'


def velocity_guard_status(record):
    """A read-only UI snapshot of the already published arbiter command.

    Requests are hidden after their original receive deadline. Publishing a
    status heartbeat never makes an expired motion request fresh again.
    """
    active = record.get('navigation_active') is True
    code = record.get('reason_code') or arbiter_reason(record)
    if not active and record.get('source') == 'zero':
        code = 'idle'
    fresh = bool(record.get('input_seen') and not record.get('input_discarded')
                 and record.get('input_age_sec') is not None
                 and record['input_age_sec'] <= record['command_timeout_sec'])
    blocked_codes = {
        'stop_latched', 'lio_paused_or_stale', 'navigation_paused_or_stale',
        'upstream_missing', 'upstream_stale', 'upstream_discarded', 'input_invalid',
        'body_identity_invalid', 'terrain_status_stale', 'terrain_unsafe',
        'ground_evidence_unanchored', 'direction_not_authorized', 'speed_limit_zero',
        'output_zero_unexplained',
        'global_localization_untrusted','global_localization_recovering',
    }
    blocked = active and code in blocked_codes
    terrain = record.get('terrain') or {}
    reason = REASONS.get(code, code)
    if code == 'terrain_unsafe':
        terrain_reason = terrain.get('validation_error') or terrain.get('reason')
        if terrain_reason:
            reason = '地形保护停车：' + str(terrain_reason)[:400]
    if code == 'direction_not_authorized' and record.get('terrain_stop_bypassed'):
        reason = '星玑3D导航禁止自动倒退或横移'
    if code == 'upstream_stale':
        reason = ('导航控制器速度消息已过期，保持零速停车；'
                  '正在等待新的有效速度指令')
    controller = record.get('controller_stream') or {}
    stream_code = stream_reason = ''
    if active and controller.get('seen'):
        controller_age = controller.get('age_sec')
        if controller_age is not None and controller_age > record['command_timeout_sec']:
            stream_code = 'controller_stream_stale'
            stream_reason = 'Nav2原始速度消息已中断或过期，请检查控制器计算、TF和代价地图'
        elif (not record.get('input_seen') or record.get('input_age_sec') is None
              or record['input_age_sec'] > record['command_timeout_sec']):
            # Eligibility can be deliberately cleared on a protection-state
            # transition while smoothed messages are still arriving on time.
            # Do not turn that safety action into a false transport diagnosis.
            stream_code = 'smoother_output_stale'
            stream_reason = 'Nav2原始速度仍在更新，但平滑后的速度消息没有及时到达'
    if blocked and stream_reason:
        reason += '；' + stream_reason
    terrain_stop_bypassed = bool(record.get('terrain_stop_bypassed'))
    cap = finite(record.get('speed_limit_mps'))
    if not (record.get('terrain_enabled') and (terrain_stop_bypassed or record.get('terrain_fresh'))
            and record.get('body_known')):
        cap = None
    return dict(schema_version=2, navigation_active=active,
        body_id=record.get('body_id', ''),
        navigation_mode='terrain_3d' if record.get('terrain_enabled') else 'map_2d',
        terrain_guard_enabled=bool(record.get('terrain_enabled')) and not terrain_stop_bypassed,
        source=record.get('source', 'zero'), input_fresh=fresh,
        input_age_sec=record.get('input_age_sec'),
        command_timeout_sec=record.get('command_timeout_sec'),
        request_velocity=record.get('input') if fresh else None,
        commanded_velocity=record.get('output'), speed_limit_mps=cap,
        speed_policy=('nav2_body_limits' if terrain_stop_bypassed
                      else terrain.get('speed_policy', 'terrain_adaptive')),
        terrain_speed_limit_enabled=(not terrain_stop_bypassed
                                     and terrain.get('terrain_speed_limit_enabled', True)),
        speed_limit_source=('terrain_advisory' if terrain_stop_bypassed
                            else terrain.get('speed_limit_source', 'terrain_assessment')),
        terrain_type=terrain.get('observed_terrain_type', ''),
        controller_stream=controller, stream_reason_code=stream_code,
        stream_reason=stream_reason,
        reason_code=code, blocked=blocked, reason=reason if blocked else '')


class CommandStreamTiming:
    """Bounded arrival telemetry; observing this stream never refreshes a command."""
    def __init__(self, topic, window_seconds=5.):
        self.topic = str(topic)
        self.window = float(window_seconds)
        self._arrivals = deque(maxlen=256)
        self._at = None
        self._values = None

    def observe(self, now, values):
        if not math.isfinite(now) or len(values) != 3 or any(v is None for v in values):
            return
        if self._at is not None and now <= self._at:
            self._arrivals.clear()
        self._at = now
        self._values = list(values)
        self._arrivals.append(now)

    def snapshot(self, now, freshness_seconds):
        while self._arrivals and now - self._arrivals[0] > self.window:
            self._arrivals.popleft()
        elapsed = None if self._at is None or now < self._at else now - self._at
        samples = list(self._arrivals)
        span = samples[-1] - samples[0] if len(samples) > 1 else 0.
        gaps = [b-a for a,b in zip(samples, samples[1:])]
        if elapsed is not None:
            gaps.append(elapsed)
        fresh = elapsed is not None and elapsed <= freshness_seconds
        return dict(topic=self.topic, seen=self._at is not None, fresh=fresh,
            age_sec=finite(elapsed), request_velocity=self._values if fresh else None,
            receive_rate_hz=finite((len(samples)-1)/span) if span > 0. else None,
            max_gap_sec=finite(max(gaps)) if gaps else None)


def _clean(value, depth=0):
    if depth>6:return None
    if isinstance(value,float):return finite(value)
    if isinstance(value,(str,int,bool)) or value is None:
        return value[:600] if isinstance(value,str) else value
    if isinstance(value,dict):
        return {str(k)[:80]:_clean(v,depth+1) for k,v in list(value.items())[:64]}
    if isinstance(value,(list,tuple)):return [_clean(v,depth+1) for v in value[:32]]
    return str(value)[:120]


class MotionTrace:
    """Changes <=2Hz, steady active summary every 2s; failed logs don't raise."""
    def __init__(self, logger, prefix):
        self.logger,self.prefix=logger,prefix
        self._last_at=-math.inf
        self._key=None
        self._counts={}
        self._window_at=None
        self.last_record={}

    def emit(self, record, *, now, active):
        try:
            if self._window_at is None:self._window_at=now
            code=str(record.get('reason_code','unknown'))
            if code not in self._counts and len(self._counts)>=32:code='other'
            self._counts[code]=self._counts.get(code,0)+1
            route=record.get('route') or {}
            key=(code,bool(active),record.get('source'),record.get('body_id'),
                 route.get('route_id'),route.get('point_number'))
            elapsed=now-self._last_at
            if elapsed<.5-1e-9 or (key==self._key and (not active or elapsed<2.0-1e-9)):
                return False
            data=_clean(dict(record,reason=REASONS.get(code,code),
                navigation_active=bool(active),window_reason_counts=dict(self._counts),
                window_sample_count=sum(self._counts.values()),
                window_duration_sec=max(0.0,now-self._window_at)))
            self.logger.info(self.prefix+' '+json.dumps(data,ensure_ascii=False,allow_nan=False,separators=(',',':')))
            self.last_record=data
            self._last_at,self._key=now,key
            self._counts={}
            self._window_at=now
            return True
        except Exception:
            # Diagnostics must never change authorization, velocity or fault
            # handling, including when stdout/logger serialization fails.
            return False
