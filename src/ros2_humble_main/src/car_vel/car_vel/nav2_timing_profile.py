"""Idle-only MPPI workload selection; never changes velocities or deadlines."""
from __future__ import annotations

import json
from pathlib import Path
import os


PARAMETER = 'FollowPath.batch_size'


def scoped_profile(mode, body):
    if mode not in ('map_2d', 'terrain_3d') or body not in ('xingji', 'gangben_zsl1'):
        return 'unknown'
    return 'xingji_3d' if mode == 'terrain_3d' and body == 'xingji' else 'baseline'


def valid_batch(value):
    if type(value) is not int or not 1 <= value <= 10000:
        raise ValueError('MPPI batch_size must be an integer from 1 to 10000')
    return value


class BaselineStore:
    """Record ownership before a write, so restarting the node can restore it."""
    def __init__(self, path, configured_baseline, optimized_batch=400):
        self.path = Path(path).expanduser()
        self.configured = valid_batch(configured_baseline)
        self.optimized = valid_batch(optimized_batch)

    def load(self):
        record = self.load_record()
        return record['baseline'] if record else None

    def load_record(self):
        if not self.path.is_file():
            return None
        data = json.loads(self.path.read_text(encoding='utf-8'))
        if (not isinstance(data, dict) or data.get('schema') != 2 or data.get('parameter') != PARAMETER
                or data.get('configured_baseline') != self.configured
                or data.get('optimized_batch') != self.optimized):
            raise ValueError('controller profile ownership record does not match this launch')
        data['baseline'] = valid_batch(data.get('baseline'))
        return data

    def save(self, baseline, *, pending_write=False, controller_gid=''):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        data = dict(schema=2, parameter=PARAMETER, baseline=valid_batch(baseline),
                    configured_baseline=self.configured, optimized_batch=self.optimized,
                    pending_write=bool(pending_write), controller_gid=str(controller_gid))
        temporary.write_text(json.dumps(data, separators=(',', ':')) + '\n', encoding='utf-8')
        os.replace(temporary, self.path)


class TimingProfilePolicy:
    def __init__(self, *, optimized_batch=400, transaction_timeout=3.0,
                 configured_baseline=None, recovered_baseline=None):
        self.optimized = valid_batch(optimized_batch)
        self.configured = valid_batch(configured_baseline) if configured_baseline is not None else None
        self.baseline = valid_batch(recovered_baseline) if recovered_baseline is not None else None
        self.actual = None
        self.profile = 'unknown'
        self.active = True
        self.enabled = True
        self.pending = None
        self.deadline = None
        self._write_deadline = self._expected_readback = None
        self.timeout = max(0.5, float(transaction_timeout))
        self.error = ''
        self.fallback = False
        self._last_context = None
        self._observed_once = False
        self.waiting_late_write = False

    def set_context(self, mode, body, *, active, enabled=True):
        context = (mode, body, bool(enabled))
        if self._last_context is not None and context != self._last_context and not self.waiting_late_write:
            self.error = ''
            self.fallback = False
        self._last_context = context
        self.profile = scoped_profile(mode, body)
        self.active, self.enabled = bool(active), bool(enabled)

    @property
    def target(self):
        if self.baseline is None or self.profile == 'unknown':
            return None
        if self.enabled and self.profile == 'xingji_3d' and not self.fallback:
            return min(self.baseline, self.optimized)
        return self.baseline

    def observe_actual(self, value):
        if self.waiting_late_write:
            raise RuntimeError('Cannot confirm readback while an owned write is unresolved')
        value = valid_batch(value)
        if self.baseline is None:
            # Only a matching ownership record proves that a value is ours.
            # A custom original value of 400 must remain 400 in 2-D as well.
            self.baseline = value
        elif (not self._observed_once and self.configured is not None
              and value not in (self.baseline, min(self.baseline, self.optimized))):
            # A fresh Nav2 process/custom configuration supersedes an old record.
            self.baseline = value
        self.actual = value
        self._observed_once = True
        self.pending = self.deadline = None
        if self._expected_readback is not None and value != self._expected_readback:
            self.error = '控制器参数写入后回读不一致，恢复原配置'
            self.fallback = True
        self._write_deadline = self._expected_readback = None

    def next_operation(self):
        if self.pending is not None or self.profile == 'unknown':
            return None
        if self.actual is None:
            return ('read', None)
        if self.active:
            return None
        if self.target != self.actual:
            return ('set', self.target)
        return None

    def begin(self, operation, value, *, now):
        if self.pending is not None:
            raise RuntimeError('A controller profile transaction is already pending')
        self.pending = (operation, value)
        self.deadline = float(now) + self.timeout
        if operation == 'set':
            self._write_deadline = self.deadline
            self._expected_readback = value
        elif self._write_deadline is not None:
            self.deadline = min(self.deadline, self._write_deadline)

    def finish_set(self, success, message=''):
        self.waiting_late_write = False
        self.pending = self.deadline = None
        self.actual = None  # A successful RPC is not a readback acknowledgement.
        if not success:
            self.error = str(message or 'controller parameter rejected')
            self.fallback = True
            self._write_deadline = self._expected_readback = None

    def fail(self, message):
        passive_read = self.pending is not None and self.pending[0] == 'read' and self.actual is not None
        self.pending = self.deadline = None
        self.error = str(message)
        if not passive_read:
            self.actual = None
            self.fallback = True
        self._write_deadline = self._expected_readback = None

    def uncertain_write(self, message):
        self.pending = ('set', None)
        self.deadline = None
        self.actual = None
        self.error = str(message)
        self.fallback = True
        self.waiting_late_write = True

    def controller_restarted(self):
        """Called only after graph evidence proves a new controller endpoint."""
        interrupted_write = (self.waiting_late_write or
            self.pending is not None and self.pending[0] == 'set')
        self.waiting_late_write = False
        self.pending = self.deadline = self._write_deadline = self._expected_readback = None
        self.actual = None
        self.baseline = None
        self._observed_once = False
        self.fallback = interrupted_write
        self.error = ('参数事务被控制器重启中断，正在回读该实例的原配置'
                      if interrupted_write else '')

    def expire(self, now):
        if self.deadline is None or now < self.deadline:
            return False
        if self.pending is not None and self.pending[0] == 'set':
            # Local future cancellation cannot cancel a remote parameter write.
            # Keep ownership until its response arrives, then read/restore.
            self.error = '控制器参数写入超过 3 秒，等待远端确认后恢复原配置；本次新目标未启动'
            self.fallback = True
            self.actual = None
            self.deadline = None
            self.waiting_late_write = True
            return True
        self.fail('控制器计算配置事务超过 3 秒，正在回读并恢复原配置')
        return True

    def status(self):
        target = self.target
        passive_read = self.pending is not None and self.pending[0] == 'read' and self.actual is not None
        ready = target is not None and self.actual == target and (self.pending is None or passive_read)
        state = ('awaiting_late_ack' if self.waiting_late_write else
                 'error' if self.error and not ready else
                 'configuring' if self.pending is not None and not passive_read else
                 'ready' if ready else 'waiting_idle' if self.active else 'pending')
        return dict(schema=1, profile=self.profile, enabled=self.enabled, ready=ready, state=state,
                    configuring=self.pending is not None and not passive_read, fallback=self.fallback, error=self.error,
                    waiting_late_write=self.waiting_late_write,
                    baseline_batch_size=self.baseline, actual_batch_size=self.actual,
                    requested_batch_size=target, transaction_timeout_sec=self.timeout,
                    controller_node='/controller_server', parameter=PARAMETER,
                    raw_velocity_topic='/cmd_vel_nav', smoothed_velocity_topic='/cmd_vel',
                    changes_velocity_limits=False, changes_command_timeout=False)
