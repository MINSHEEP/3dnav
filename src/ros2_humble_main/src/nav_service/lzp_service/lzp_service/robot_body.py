"""Validated, atomic robot-body configuration and single-worker ownership.

No ROS or vendor SDK is imported here. A worker must acknowledge freezing before
it is replaced; a failed replacement never restarts the old robot implicitly.
"""
import copy
import ipaddress
import json
import os
from pathlib import Path
import tempfile
import time


BODY_NAMES = {'xingji': '星玑', 'gangben_zsl1': '钢镚 ZSL-1 点足',
              'xuanji': '璇玑 Hypertron 轮式四足'}


def default_config():
    return {'version': 1, 'selected': 'xingji', 'profiles': {
        'xingji': {'robot_ip': '192.168.96.2', 'local_ip': '', 'port': 8080,
                   'navigation_verified': True, 'max_vx': 0.5, 'max_vy': 0.2,
                   'max_yaw': 0.6},
        'gangben_zsl1': {'robot_ip': '192.168.168.168', 'local_ip': '',
                        'port': 43988, 'navigation_verified': False,
                        'max_vx': 0.5, 'max_vy': 0.2, 'max_yaw': 0.6},
        'xuanji': {'robot_ip': '10.18.10.100', 'local_ip': '10.18.10.201',
                   'port': 3601, 'navigation_verified': False,
                   'max_vx': 0.5, 'max_vy': 0.0, 'max_yaw': 0.6}}}


def validate_profile(body, profile):
    if body not in BODY_NAMES or not isinstance(profile, dict):
        raise ValueError('不支持的机器人本体')
    expected = set(default_config()['profiles'][body])
    if set(profile) != expected:
        raise ValueError('本体配置字段不完整或包含未知字段')
    result = copy.deepcopy(profile)
    for key in ('robot_ip', 'local_ip'):
        value = result[key]
        if not isinstance(value, str):
            raise ValueError('IP 地址必须为文本')
        if key == 'local_ip' and body == 'xingji' and value == '':
            continue
        # An unconfigured ZSL-1 profile can be stored, but cannot be activated.
        if key == 'local_ip' and value == '':
            continue
        address = ipaddress.IPv4Address(value)
        if (address.is_multicast or address.is_unspecified or address.is_loopback
                or str(address) == '255.255.255.255'):
            raise ValueError('请填写机器人专用网络的有效 IPv4 地址')
        result[key] = str(address)
    if result['robot_ip'] == result['local_ip']:
        raise ValueError('小电脑 IP 与机器人 IP 不能相同')
    if type(result['port']) is not int or not 1024 <= result['port'] <= 65535:
        raise ValueError('端口必须是 1024–65535 的整数')
    if type(result['navigation_verified']) is not bool:
        raise ValueError('导航安装参数确认必须为布尔值')
    for key, maximum in (('max_vx', 0.8), ('max_vy', 0.3), ('max_yaw', 1.0)):
        value = result[key]
        if body == 'xuanji' and key == 'max_vy':
            if type(value) not in (int, float) or value != 0:
                raise ValueError('璇玑禁止横移，max_vy 必须为 0')
            result[key] = 0.0
            continue
        if type(value) not in (int, float) or not 0.1 <= value <= maximum:
            raise ValueError(f'{key} 超出本项目首版安全范围 0.1–{maximum}')
        result[key] = float(value)
    if body == 'xuanji':
        if result['robot_ip'] != '10.18.10.100' or result['port'] != 3601:
            raise ValueError('璇玑 SDK 固定机器人地址 10.18.10.100，接收端口 3601')
        if result['local_ip'] not in [f'10.18.10.{n}' for n in range(200, 210)]:
            raise ValueError('璇玑 SDK 要求本机网卡配置在 10.18.10.200–209；网页不会设置网卡')
    return result


def config_path():
    return Path.home() / '.ros' / 'robot_navigation' / 'robot_body.json'


class BodyConfigStore:
    def __init__(self, path=None):
        self.path = Path(path) if path else config_path()

    def load(self):
        if not self.path.exists():
            return default_config()
        raw = json.loads(self.path.read_text(encoding='utf-8'))
        if not isinstance(raw, dict):
            raise ValueError('本体配置必须为 JSON 对象，禁止自动回退')
        # Extend the previous two-body schema without changing saved settings.
        if set(raw.get('profiles', {})) == {'xingji', 'gangben_zsl1'}:
            raw['profiles']['xuanji'] = default_config()['profiles']['xuanji']
        if (raw.get('selected') not in BODY_NAMES or
                type(raw.get('version')) is not int or raw['version'] < 1 or
                set(raw.get('profiles', {})) != set(BODY_NAMES)):
            raise ValueError('本体配置文件损坏；禁止自动回退并控制星玑')
        for body in BODY_NAMES:
            raw['profiles'][body] = validate_profile(body, raw['profiles'][body])
        return raw

    def save(self, config):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(prefix='.robot_body-', dir=self.path.parent)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                json.dump(config, stream, ensure_ascii=False, indent=2,
                          allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, self.path)
        finally:
            if os.path.exists(name):
                os.unlink(name)


def parked_status(status):
    """Only fresh, explicitly parked status permits releasing an online body."""
    return (status.get('connected') is True and status.get('stale') is False
            and status.get('parked') is True and not status.get('remote_active')
            and not status.get('active_action'))


class BodySupervisor:
    def __init__(self, worker_factory, store=None, clock=time.monotonic):
        self.factory, self.store, self.clock = worker_factory, store or BodyConfigStore(), clock
        self.worker = None
        self.config = None
        self.status = {}
        self.status_at = 0.0
        self.error = ''
        self.switching = False
        self.generation = 0
        try:
            self.config = self.store.load()
        except (OSError, ValueError, TypeError) as error:
            self.error = str(error)

    def start(self):
        if self.config is None:
            return
        body = self.config['selected']
        self._start(body, self.config['profiles'][body])

    def _start(self, body, profile, frozen=False):
        self.generation += 1
        self.status, self.status_at = {}, 0.0
        self.started_at = self.clock()
        self.worker = self.factory(body, profile, self.generation, frozen)

    def poll(self):
        if not self.worker:
            return
        for message in self.worker.poll():
            if message.get('kind') == 'status':
                self.status = message['data']
                self.status_at = self.clock()
            elif message.get('kind') == 'error':
                self.error = str(message['message'])
        if (self.worker.alive() and not self.status_at
                and self.clock() - self.started_at > 12.0):
            self.worker.close()
            self.error = '本体初始化超时；已停止无状态的控制进程，请检查 SDK/网卡后重新应用'
        if not self.worker.alive():
            self.error = self.error or '本体控制进程退出；不会自动重启或恢复运动，请检查连接后重新应用'
            self.status = {}

    def apply(self, body, profile, expected_version, idle_check, offline_xingji_powered_off=False,
              switch_confirmed=False):
        if self.config is None:
            raise ValueError(self.error)
        if expected_version != self.config['version']:
            raise ValueError('配置已被其他页面修改，请刷新后重试')
        profile = validate_profile(body, profile)
        if body == 'gangben_zsl1' and not profile['local_ip']:
            raise ValueError('请填写小电脑连接钢镚的网卡 IP，并先配置机器人回传地址')
        idle_check()
        self.switching = True
        replacing = False
        try:
            if self.worker and self.worker.alive():
                # Worker processes the barrier in the same loop as ROS commands.
                confirmed = (offline_xingji_powered_off is True and
                             self.config['selected'] == 'xingji' and body == 'gangben_zsl1')
                operation = ('prepare_confirmed' if switch_confirmed else
                             'prepare_xingji_powered_off' if confirmed else 'prepare')
                reply = self.worker.request(operation, timeout=3.0)
                if not reply.get('success'):
                    raise ValueError(reply.get('message', '旧本体尚未安全停放'))
                replacing = True
                self.worker.close()
                if self.worker.alive():
                    raise RuntimeError('旧本体进程未退出，禁止启动新本体')
            self.worker = None
            replacing = True
            self.error = ''
            self._start(body, profile, frozen=True)
            reply = self.worker.request('ready', timeout=10.0)
            if not reply.get('success'):
                raise RuntimeError(reply.get('message', '新本体连接失败'))
            updated = copy.deepcopy(self.config)
            updated['selected'] = body
            updated['profiles'][body] = profile
            updated['version'] += 1
            self.store.save(updated)
            self.config = updated
            reply = self.worker.request('activate', timeout=3.0)
            if not reply.get('success'):
                raise RuntimeError('配置已保存，但本体启用失败；请重新应用')
            self.error = ''
        except Exception as error:
            if replacing:
                self.error = str(error)
                if self.worker:
                    self.worker.close()
                    if not self.worker.alive():
                        self.worker = None
                self.status, self.status_at = {}, 0.0
            # Rejected parking checks leave the old body unchanged. Failure
            # after freezing never unfreezes/restarts the old body implicitly.
            raise
        finally:
            self.switching = False

    def snapshot(self):
        fresh = self.clock() - self.status_at <= 2.0
        return {'config': self.config, 'generation': self.generation,
                'switching': self.switching or (fresh and self.status.get('switching') is True),
                'error': self.error,
                'connected': fresh and self.status.get('connected') is True,
                'active_body': self.status.get('body_id') if fresh else None,
                'status': self.status if fresh else {},
                'names': BODY_NAMES}

    def close(self):
        if self.worker:
            self.worker.close()
