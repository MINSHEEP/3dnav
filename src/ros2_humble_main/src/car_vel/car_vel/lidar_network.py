"""Persist driver addresses; never change NICs or the sensor's own IP."""
import copy
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import socket
import tempfile


def validate(value):
    if not isinstance(value, dict) or set(value) != {'version', 'lidar_ip', 'host_ip'}:
        raise ValueError('雷达网络配置格式错误')
    if type(value['version']) is not int or not 0 <= value['version'] < 2**31:
        raise ValueError('配置版本错误')
    for key in ('lidar_ip', 'host_ip'):
        if not isinstance(value[key], str):
            raise ValueError('请输入 IPv4 地址')
        try:
            address = ipaddress.IPv4Address(value[key])
        except ipaddress.AddressValueError as error:
            raise ValueError('请输入有效 IPv4 地址，不接受主机名或命令') from error
        if (address.is_multicast or address.is_unspecified or address.is_loopback
                or address.is_reserved or int(address) >> 24 == 0):
            raise ValueError('请使用有效的单播设备地址，不能使用回环、广播或组播地址')
    if value['lidar_ip'] == value['host_ip']:
        raise ValueError('雷达与本机 IP 不能相同')
    return dict(value)


def local_address_warning(host_ip):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as connection:
            connection.bind((host_ip, 0))
    except OSError:
        return '本机当前无法绑定该 IP；请先在系统中配置连接雷达的网卡，再启动雷达'
    return ''


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.lidar-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class NetworkStore:
    def __init__(self, template, directory=None):
        self.template = Path(template)
        self.directory = Path(directory) if directory else Path.home() / '.ros/robot_navigation'
        self.path = self.directory / 'lidar_network.json'

    def load(self):
        if self.path.exists():
            return validate(json.loads(self.path.read_text(encoding='utf-8')))
        template = json.loads(self.template.read_text(encoding='utf-8'))
        return validate({'version': 0, 'lidar_ip': template['lidar_configs'][0]['ip'],
                         'host_ip': template['Mid360s']['host_net_info'][0]['host_ip']})

    def save(self, lidar_ip, host_ip, expected_version):
        # Serialize saves across accidental duplicate service processes too.
        import fcntl
        self.directory.mkdir(parents=True, exist_ok=True)
        with (self.directory / 'lidar_network.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            current = self.load()
            if expected_version != current['version']:
                raise ValueError('配置已更新，请刷新后重试')
            value = validate({'version': current['version'] + 1,
                              'lidar_ip': lidar_ip, 'host_ip': host_ip})
            atomic_json(self.path, value)
        return value

    def snapshot(self):
        value = self.load()  # Corrupt saved settings fail closed, never silently fall back.
        template = json.loads(self.template.read_text(encoding='utf-8'))
        if len(template['lidar_configs']) != 1 or len(template['Mid360s']['host_net_info']) != 1:
            raise ValueError('当前设置只支持单台 MID360s 雷达')
        config = copy.deepcopy(template)
        config['lidar_configs'][0]['ip'] = value['lidar_ip']
        config['Mid360s']['host_net_info'][0]['host_ip'] = value['host_ip']
        digest = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
        path = self.directory / 'lidar_runtime' / (digest + '.json')
        # Content-addressed snapshot: later saves cannot mutate a live launch's file.
        atomic_json(path, config)
        return str(path)


def installed_store():
    from ament_index_python.packages import get_package_share_directory
    return NetworkStore(Path(get_package_share_directory('livox_ros_driver2'))
                        / 'config/MID360s_config.json')


def runtime_config_path():
    return installed_store().snapshot()
