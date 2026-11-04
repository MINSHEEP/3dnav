"""USB serial inventory and fail-closed M701 binding; never opens a serial port."""
import copy
import json
import os
from pathlib import Path
import tempfile


class ConnectionError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def _text(path):
    try:
        return path.read_text().strip()
    except (OSError, UnicodeError):
        return ''


def _driver_name(path):
    if (path / 'driver').exists():
        return (path / 'driver').resolve().name
    return next((line.partition('=')[2] for line in _text(path / 'uevent').splitlines()
                 if line.startswith('DRIVER=')), '')


def inventory(sys_root=Path('/sys'), dev_root=Path('/dev')):
    """Include out-of-tree CH341/CH348 ttys omitted by some list_ports versions."""
    devices = []
    for entry in sorted((sys_root / 'class/tty').glob('*')):
        target = (entry / 'device').resolve()
        ancestors = [target, *target.parents]
        usb = next((p for p in ancestors if (p / 'idVendor').exists()
                    and (p / 'idProduct').exists()), None)
        if usb is None:
            continue
        port = str(dev_root / entry.name)
        if not Path(port).exists():
            continue
        interface = next((_text(p / 'bInterfaceNumber') for p in ancestors
                          if (p / 'bInterfaceNumber').exists()), '')
        channel = next((_text(p / 'port_number') for p in ancestors
                        if (p / 'port_number').exists()), '')
        driver = next(filter(None, (_driver_name(p) for p in ancestors)), '')
        aliases = []
        for folder in ('by-id', 'by-path'):
            for alias in sorted((dev_root / 'serial' / folder).glob('*')):
                if alias.resolve() == Path(port).resolve():
                    aliases.append(str(alias))
        vid, pid = _text(usb / 'idVendor').lower(), _text(usb / 'idProduct').lower()
        location = next((p for p in aliases if '/by-path/' in p), '')
        if not location:
            interface_node = next((p for p in ancestors if (p / 'bInterfaceNumber').exists()), usb)
            location = str(interface_node) + (':' + channel if channel else '')
        devices.append({
            'port': port, 'aliases': aliases, 'vid': vid, 'pid': pid,
            'serial': _text(usb / 'serial'), 'interface': interface, 'channel': channel,
            'location': location, 'driver': driver, 'usb_path': str(usb),
            'product': _text(usb / 'product') or entry.name,
            'candidate': vid == '1a86' or vid == '4348' or driver.lower().startswith(('ch341', 'ch348')),
        })
    for device in devices:
        siblings = [d for d in devices if d['usb_path'] == device['usb_path']
                    and d['interface'] == device['interface']]
        if not device['channel'] and len(siblings) == 1:
            device['channel'] = '0'
    missing = []
    seen = {d['usb_path'] for d in devices}
    for usb in (sys_root / 'bus/usb/devices').glob('*'):
        if _text(usb / 'idVendor').lower() in ('1a86', '4348') and str(usb.resolve()) not in seen:
            interfaces = [{'interface': _text(p / 'bInterfaceNumber'),
                           'driver': _driver_name(p)} for p in sorted(usb.iterdir())
                          if (p / 'bInterfaceNumber').exists()]
            bus, number = _text(usb / 'busnum'), _text(usb / 'devnum')
            usb_node = (str(dev_root / 'bus/usb' / bus.zfill(3) / number.zfill(3))
                        if bus.isdecimal() and number.isdecimal() else '')
            missing.append({'vid': _text(usb / 'idVendor'), 'pid': _text(usb / 'idProduct'),
                            'product': _text(usb / 'product'), 'usb_path': str(usb.resolve()),
                            'interfaces': interfaces, 'usb_node': usb_node})
    return devices, missing


def identity(device):
    """Serial identity when present, physical interface/channel otherwise."""
    result = {key: device.get(key, '') for key in ('vid', 'pid', 'serial', 'interface', 'channel')}
    if not result['serial']:
        result['location'] = device.get('location', '')
    return result


def observation(device):
    # UI selections also carry the physical location to detect tty reuse
    # between the displayed inventory and the confirmed operation.
    return dict(identity(device), location=device.get('location', ''))


def matches_binding(device, binding):
    return all(device.get(key, '') == value for key, value in binding.items())


def validate(config):
    if not isinstance(config, dict) or set(config) != {
            'version', 'mode', 'port', 'baudrate', 'address', 'binding'}:
        raise ValueError('气体传感器配置格式不正确')
    for key, low, high in (('version', 0, 2**31 - 1), ('address', 0, 7)):
        if type(config[key]) is not int or not low <= config[key] <= high:
            raise ValueError(f'{key} 超出范围')
    if type(config['baudrate']) is not int or config['baudrate'] not in (2400, 4800, 9600, 19200, 38400, 57600, 115200):
        raise ValueError('不支持的波特率')
    if config['mode'] not in ('auto', 'manual'):
        raise ValueError('连接模式不正确')
    if not isinstance(config['port'], str) or not config['port'].startswith('/dev/'):
        raise ValueError('必须选择 Linux 串口设备')
    binding = config['binding']
    if config['mode'] == 'auto' and binding is None:
        raise ValueError('自动重连必须先绑定一个已确认设备')
    if binding is not None:
        if not isinstance(binding, dict) or set(binding) not in (
                {'vid', 'pid', 'serial', 'interface', 'channel'},
                {'vid', 'pid', 'serial', 'interface', 'channel', 'location'}):
            raise ValueError('设备身份格式不正确')
        if not all(isinstance(v, str) for v in binding.values()):
            raise ValueError('设备身份格式不正确')
        if not binding['serial'] and not binding.get('location'):
            raise ValueError('缺少设备序列号或 USB 插口身份')
    return copy.deepcopy(config)


def resolve(config, devices):
    binding = config['binding']
    if binding is not None:
        matches = [d for d in devices if matches_binding(d, binding)]
        if len(matches) > 1:
            raise ConnectionError('ambiguous', '多个串口具有相同身份，拒绝猜测；请仅保留目标设备或绑定独立接口')
        if not matches:
            raise ConnectionError('device_missing', '未找到已绑定的传感器转接模块；不会切换到其他串口')
        if config['mode'] == 'auto':
            return matches[0]
    matches = [d for d in devices if config['port'] in [d['port'], *d.get('aliases', [])]]
    if len(matches) != 1:
        raise ConnectionError('device_missing', '指定串口不存在或不唯一，请在设置页重新选择设备')
    device = matches[0]
    if binding is not None and not matches_binding(device, binding):
        raise ConnectionError('identity_changed', '串口编号已被其他设备占用，拒绝自动连接')
    if binding is None and not device['candidate']:
        raise ConnectionError('unconfirmed', '此串口不是已知候选，请在设置页确认连接的是 M701')
    # Even legacy by-id links must not choose between duplicate serial numbers.
    if binding is None and len([d for d in devices if identity(d) == identity(device)]) != 1:
        raise ConnectionError('ambiguous', '设备身份不唯一，不能安全绑定此串口')
    return device


def diagnose(config, devices, missing_usb):
    """Explain discovery failures and recommend, but never authorize, a port."""
    def result(code, message, suggested_port=''):
        return {'code': code, 'message': message, 'suggested_port': suggested_port,
                'confirmation_required': bool(suggested_port)}

    if not devices:
        if missing_usb:
            usb_ids = ', '.join(sorted({f"{d['vid']}:{d['pid']}" for d in missing_usb}))
            if any(i.get('driver') == 'usbfs' for d in missing_usb
                   for i in d.get('interfaces', [])):
                return result('usb_claimed',
                              f'已识别 WCH USB 设备 {usb_ids}，但接口由 usbfs 接管且没有串口；'
                              '请核对占用 USB 的进程和内核日志（例如是否被 BRLTTY 抢占），'
                              '不能仅凭此状态认定驱动未安装，也不要自动卸载辅助服务')
            return result('driver_missing',
                          f'已识别 WCH USB 设备 {usb_ids}，但没有生成 USB 串口；'
                          '请按实际芯片检查驱动和内核日志，不要盲目重装驱动')
        return result('serial_not_found',
                      '未发现任何 USB 串口；请先检查 RS485 转 USB 模块的连接，'
                      '单独运行 lsusb 核对 USB 枚举，再检查驱动；尚未向任何串口发送查询')
    try:
        resolve(config, devices)
    except ConnectionError as error:
        if error.code != 'device_missing':
            return result(error.code, str(error))
        binding = config['binding']
        if binding is not None:
            matches = [d for d in devices if matches_binding(d, binding)]
            if config['mode'] == 'manual' and len(matches) == 1:
                return result('path_changed',
                              f"固定串口 {config['port']} 已失效，但找到了原绑定设备 "
                              f"{matches[0]['port']}；请在设置页确认后改用自动重连并保存",
                              matches[0]['port'])
            return result(error.code,
                          '原绑定的传感器转接模块不在当前设备列表中；'
                          '不会自动切换到其他设备。更换模块或 USB 插口后，请在设置页重新确认绑定')
        candidates = [d for d in devices if d.get('candidate')]
        if len(candidates) == 1:
            return result('selection_required',
                          f"旧串口 {config['port']} 不存在，发现一个 CH341/CH348 候选 "
                          f"{candidates[0]['port']}；请在设置页确认它连接 M701，测试后保存。"
                          '推荐设备不等于已连接，不会自动探测该串口', candidates[0]['port'])
        if len(candidates) > 1:
            return result('ambiguous_selection',
                          f"旧串口 {config['port']} 不存在，发现 {len(candidates)} 个候选串口；"
                          '请在设置页按实际接线选择，程序不会猜测或轮流探测')
        return result(error.code,
                      f"指定串口 {config['port']} 不存在；已发现的其他 USB 串口不能自动认作 M701，"
                      '请在设置页核对实际设备')
    return result('configured', '已找到配置指定的串口；是否在线以通过 CRC 校验的 M701 数据为准')


def draft(current, request, devices):
    if not isinstance(request, dict) or request.get('confirmed') is not True:
        raise ValueError('请先确认所选 USB 串口连接的是 M701，而非底盘或其他设备')
    if set(request) != {'mode', 'port', 'baudrate', 'address', 'identity', 'confirmed'}:
        raise ValueError('连接请求字段不正确')
    selected = [d for d in devices if request['port'] in [d['port'], *d.get('aliases', [])]]
    if len(selected) != 1 or observation(selected[0]) != request['identity']:
        raise ConnectionError('device_changed', '设备列表已经变化，请刷新并重新确认设备')
    binding = identity(selected[0])
    if sum(matches_binding(d, binding) for d in devices) > 1:
        # Explicit selection may pin a duplicated serial number to a unique
        # USB interface/port. Still fail closed if channels cannot be separated.
        binding['location'] = selected[0].get('location', '')
    config = validate({'version': current['version'], 'mode': request['mode'],
                       'port': request['port'], 'baudrate': request['baudrate'],
                       'address': request['address'], 'binding': binding})
    resolve(config, devices)
    return config


class ConfigStore:
    def __init__(self, path=None):
        self.path = Path(path) if path else Path.home() / '.ros/robot_navigation/m701_connection.json'

    def load(self, fallback):
        if not self.path.exists():
            return validate(fallback)
        return validate(json.loads(self.path.read_text(encoding='utf-8')))

    def save(self, config):
        config = validate(config)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix='.m701-', dir=self.path.parent)
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                json.dump(config, stream, ensure_ascii=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)


def main():
    """Read-only CLI inventory, usable without ROS or pyserial."""
    devices, missing_usb = inventory()
    report = {'devices': devices, 'usb_without_serial': missing_usb}
    fallback = {'version': 0, 'mode': 'manual',
                'port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                'baudrate': 9600, 'address': 1, 'binding': None}
    try:
        config = ConfigStore().load(fallback)
        report.update(config=config, diagnostic=diagnose(config, devices, missing_usb))
    except (OSError, ValueError, TypeError) as error:
        report['diagnostic'] = {'code': 'config_error', 'message': str(error),
                                'suggested_port': '', 'confirmation_required': False}
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
