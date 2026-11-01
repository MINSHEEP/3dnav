"""ZSL-1 high-level adapter. Physical velocity units, no motor-level access."""
import importlib
import ctypes
import math
from pathlib import Path
import platform
import sys


def load_sdk_module():
    arch = platform.machine().lower().replace('amd64', 'x86_64').replace('arm64', 'aarch64')
    if sys.platform != 'linux' or sys.version_info[:2] != (3, 10):
        raise RuntimeError('钢镚 SDK 需要 Linux / Python 3.10，请使用 /usr/bin/python3')
    if arch not in ('x86_64', 'aarch64'):
        raise RuntimeError('钢镚 SDK 不支持当前 CPU 架构')
    folder = Path(__file__).parent / 'lib' / 'zsl-1' / arch
    if not folder.is_dir():
        raise RuntimeError(f'缺少钢镚 SDK：{folder}')
    # Resolve the native companion explicitly, independent of the old host's
    # build-time library search path. Import does not construct HighLevel.
    native = ctypes.CDLL(str(folder / f'libmc_sdk_zsl_1_{arch}.so'), mode=ctypes.RTLD_GLOBAL)
    sys.path.insert(0, str(folder))
    try:
        sdk = importlib.import_module('mc_sdk_zsl_1_py')
        sdk._native_companion = native
        for method in ('initRobot', 'checkConnect', 'standUp', 'lieDown', 'passive',
                       'move', 'getBatteryPower', 'getCurrentCtrlmode', 'getBodyVelocity',
                       'getBodyGyro'):
            if not callable(getattr(sdk.HighLevel, method, None)):
                raise RuntimeError(f'钢镚 SDK 版本不匹配，缺少 HighLevel.{method}')
        return sdk
    finally:
        sys.path.remove(str(folder))


def load_sdk():
    return load_sdk_module().HighLevel()


class Zsl1Adapter:
    def __init__(self, profile, sdk=None):
        self.profile = profile
        self.sdk = sdk if sdk is not None else load_sdk()
        self.error = ''
        self.emergency = False
        self.hardware_emergency_latched = False
        self.zero_stop_command_succeeded = None
        self.target = (0.0, 0.0, 0.0)
        self.output = [0.0, 0.0, 0.0]
        self.moving = False
        self.last_mode = None
        if not profile['local_ip']:
            raise ValueError('尚未配置小电脑连接钢镚的本地 IP')
        # initRobot may block in a faulty native library; the supervisor owns
        # this isolated process and enforces a bounded readiness deadline.
        self.sdk.initRobot(profile['local_ip'], profile['port'], profile['robot_ip'])

    def call(self, name, *args):
        try:
            code = getattr(self.sdk, name)(*args)
            if code == 0:
                return
            message = f'{name} 失败，SDK 错误码 0x{int(code):04X}'
            if int(code) == 0x3007:
                message += '（状态机切换失败）'
        except Exception as error:
            message = f'{name} 失败：{error}'
        message += f'；最近模式={self.last_mode}，参数={args}'
        self.fail_safe(message)
        raise RuntimeError(self.error)

    def fail_safe(self, message):
        self.error = message
        self.soft_stop()

    def soft_stop(self):
        """One bounded zero request, no recursive SDK error or passive fallback.

        A successful SDK return is not a physical stop certificate. On rejection
        retain the fault and motion latch until explicit human intervention.
        """
        self.emergency = True
        self.target = (0.0, 0.0, 0.0)
        self.output = [0.0, 0.0, 0.0]
        self.zero_stop_command_succeeded = False
        try:
            result = self.sdk.move(0.0, 0.0, 0.0)
            self.zero_stop_command_succeeded = result == 0
            if not self.zero_stop_command_succeeded:
                self.error += f'；零速请求失败：0x{int(result):04X}，无法确认实际停车；未执行阻尼急停'
        except Exception as error:
            self.error += f'；零速请求失败：{error}，无法确认实际停车；未执行阻尼急停'
        self.moving = False

    def state(self):
        connected = bool(self.sdk.checkConnect())
        if not connected:
            return {'connected': False, 'stale': True, 'parked': False}
        mode = int(self.sdk.getCurrentCtrlmode())
        self.last_mode = mode
        velocity = list(self.sdk.getBodyVelocity())
        velocity_valid = len(velocity) == 3 and all(math.isfinite(float(v)) for v in velocity)
        gyro = list(self.sdk.getBodyGyro())
        gyro_valid = len(gyro) == 3 and all(math.isfinite(float(v)) for v in gyro)
        stationary = velocity_valid and all(abs(v) < 0.03 for v in velocity)
        battery = int(self.sdk.getBatteryPower())
        return {'connected': True, 'stale': False,
                'battery': battery if 0 <= battery <= 100 else None,
                # Vendor mode is a last executed control state, not proof that
                # an action finished. Do not translate it as physical posture.
                'sdk_control_mode': mode,
                'posture': {0: '阻尼', 1: '站立/打招呼模式', 10: '失能模式',
                            18: '行走模式', 21: '动作模式', 51: '趴下模式'}.get(mode, '未知'),
                'parked': mode in (0, 10, 51) and stationary and not self.moving,
                'body_velocity': velocity if velocity_valid else None,
                'body_gyro': gyro if gyro_valid else None,
                'model_label': '钢镚高层行走', 'obstacle_stop_enabled': None,
                'hardware_error': True if self.error else None,
                'hardware_error_value': self.error,
                'hw_failures': [self.error] if self.error else [],
                'emergency_latched': self.emergency,
                'hardware_emergency_latched': self.hardware_emergency_latched,
                'stop_kind': ('manual_emergency' if self.hardware_emergency_latched
                              else 'zero_speed_latched' if self.emergency else ''),
                'zero_stop_command_succeeded': self.zero_stop_command_succeeded}

    def set_velocity(self, values):
        if (self.emergency or self.error or len(values) != 3 or
                not all(math.isfinite(float(value)) for value in values)):
            values = (0.0, 0.0, 0.0)
        limits = [self.profile[key] for key in ('max_vx', 'max_vy', 'max_yaw')]
        self.target = tuple(max(-limit, min(limit, float(value)))
                            for value, limit in zip(values, limits))
        for axis, value in enumerate(self.target):
            if value == 0:
                self.output[axis] = 0.0

    def tick(self, elapsed, permitted):
        if not permitted or self.emergency or self.error:
            self.stop()
            return
        for axis, (target, acceleration) in enumerate(zip(self.target, (0.4, 0.4, 0.6))):
            delta = max(0.0, min(elapsed, 0.05)) * acceleration
            self.output[axis] += max(-delta, min(delta, target - self.output[axis]))
        # Stay inside documented nonzero ranges; never raise a small target
        # into a larger speed that the planner did not request.
        values = [v if abs(v) >= minimum else 0.0
                  for v, minimum in zip(self.output, (0.05, 0.1, 0.02))]
        if any(values) or self.moving:
            self.call('move', *values)
            self.moving = any(values)

    def stop(self):
        self.target = (0.0, 0.0, 0.0)
        self.output = [0.0, 0.0, 0.0]
        if self.moving:
            self.call('move', 0.0, 0.0, 0.0)
            self.moving = False

    def emergency_stop(self):
        self.emergency = True
        self.hardware_emergency_latched = True
        self.target = (0.0, 0.0, 0.0)
        self.output = [0.0, 0.0, 0.0]
        self.call('passive')
        self.moving = False

    def reset(self):
        # Reset clears the software latch, never stands up or replays a target.
        self.stop()
        self.error = ''
        self.emergency = False
        self.hardware_emergency_latched = False
        self.zero_stop_command_succeeded = None


if __name__ == '__main__':
    # Read-only installation check: no HighLevel instance or initRobot call.
    load_sdk_module()
    print('ZSL-1 原生库和 Python 3.10 接口导入检查通过；未连接机器人或发送动作')
