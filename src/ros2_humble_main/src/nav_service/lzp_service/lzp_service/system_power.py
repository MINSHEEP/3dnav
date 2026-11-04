"""Safe, fixed-command host reboot and power-off controller."""

from datetime import datetime, timezone
from pathlib import Path
import subprocess
from threading import Event, RLock, Thread


SUDO_PATH = '/usr/bin/sudo'
SYSTEMCTL_PATH = '/usr/bin/systemctl'
POWER_COMMANDS = {
    'reboot': (SUDO_PATH, '-n', SYSTEMCTL_PATH, 'reboot'),
    'poweroff': (SUDO_PATH, '-n', SYSTEMCTL_PATH, 'poweroff'),
}
POWER_LABELS = {'reboot': '重启', 'poweroff': '关机'}


def _utc_now():
    return datetime.now(timezone.utc).isoformat()


class SystemPowerController:
    """Validate permission, serialize requests and run fixed argv commands."""

    def __init__(
            self, *, enabled=True, delay_seconds=3.0,
            run_command=subprocess.run, schedule=None,
            status_callback=None, path_exists=None):
        self._enabled = bool(enabled)
        self._delay_seconds = min(30.0, max(1.0, float(delay_seconds)))
        self._run_command = run_command
        self._schedule = schedule or self._schedule_in_thread
        self._status_callback = status_callback
        self._path_exists = path_exists or (lambda value: Path(value).is_file())
        self._lock = RLock()
        self._permissions = {action: None for action in POWER_COMMANDS}
        self._status = {
            'enabled': self._enabled,
            'pending': False,
            'action': '',
            'state': 'checking' if self._enabled else 'disabled',
            'message': (
                '正在检查系统电源控制权限'
                if self._enabled else '系统电源控制已由启动参数禁用'
            ),
            'delay_seconds': self._delay_seconds,
            'permissions': dict(self._permissions),
            'updated_at': _utc_now(),
        }

    def status(self):
        """Return an isolated copy of the current observable state."""
        with self._lock:
            result = dict(self._status)
            result['permissions'] = dict(self._permissions)
            return result

    def refresh_permissions(self):
        """Probe both exact sudoers entries without executing either action."""
        if not self._enabled:
            self._update(
                state='disabled', message='系统电源控制已由启动参数禁用')
            return False
        permissions = {
            action: self._has_permission(action) for action in POWER_COMMANDS
        }
        with self._lock:
            self._permissions.update(permissions)
        if all(permissions.values()):
            self._update(state='ready', message='系统电源控制已就绪')
            return True
        self._update(
            state='unavailable',
            message='系统权限未配置，请先运行电源权限安装脚本',
        )
        return False

    def request(self, action):
        """Schedule one whitelisted action after a permission preflight."""
        if action not in POWER_COMMANDS:
            return False, '拒绝未知的系统电源操作'
        with self._lock:
            if not self._enabled:
                return False, '系统电源控制已由启动参数禁用'
            if self._status['pending']:
                active = POWER_LABELS.get(self._status['action'], '电源操作')
                return False, f'{active}已经进入执行队列，请勿重复操作'

            permitted = self._has_permission(action)
            self._permissions[action] = permitted
            if not permitted:
                message = '系统电源权限不可用，请先运行电源权限安装脚本'
                self._update_locked(state='unavailable', message=message)
                self._emit_locked()
                return False, message

            label = POWER_LABELS[action]
            self._update_locked(
                pending=True,
                action=action,
                state='scheduled',
                message=f'已接受{label}请求，将在 '
                        f'{self._delay_seconds:g} 秒后执行',
            )
            self._emit_locked()
            try:
                self._schedule(
                    self._delay_seconds,
                    lambda: self._execute(action),
                )
            except Exception as error:
                message = f'无法创建{label}执行任务：{error}'
                self._update_locked(
                    pending=False, action='', state='failed', message=message)
                self._emit_locked()
                return False, message
            return True, self._status['message']

    def _has_permission(self, action):
        if not self._path_exists(SUDO_PATH) or not self._path_exists(
                SYSTEMCTL_PATH):
            return False
        permission_command = (
            SUDO_PATH, '-n', '-l', SYSTEMCTL_PATH, action)
        try:
            result = self._run_command(
                permission_command,
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            return False
        return result.returncode == 0

    def _execute(self, action):
        label = POWER_LABELS[action]
        self._update(
            state='executing', message=f'正在向系统提交{label}命令')
        try:
            result = self._run_command(
                POWER_COMMANDS[action],
                capture_output=True,
                text=True,
                timeout=15.0,
                check=False,
            )
            if result.returncode != 0:
                detail = (result.stderr or result.stdout or '').strip()
                message = f'{label}命令执行失败'
                if detail:
                    message += f'：{detail[:300]}'
                self._update(
                    pending=False, action='', state='failed', message=message)
                return
            self._update(
                pending=False,
                action='',
                state='submitted',
                message=f'{label}命令已提交，系统连接即将断开',
            )
        except (OSError, subprocess.SubprocessError) as error:
            self._update(
                pending=False,
                action='',
                state='failed',
                message=f'{label}命令执行异常：{error}',
            )

    def _update(self, **values):
        with self._lock:
            self._update_locked(**values)
            self._emit_locked()

    def _update_locked(self, **values):
        self._status.update(values)
        self._status['permissions'] = dict(self._permissions)
        self._status['updated_at'] = _utc_now()

    def _emit_locked(self):
        if self._status_callback is None:
            return
        try:
            self._status_callback(self.status())
        except Exception:
            # Observability must never change whether a fixed power command runs.
            pass

    @staticmethod
    def _schedule_in_thread(delay_seconds, callback):
        def worker():
            Event().wait(delay_seconds)
            callback()

        Thread(target=worker, daemon=True, name='system-power').start()
