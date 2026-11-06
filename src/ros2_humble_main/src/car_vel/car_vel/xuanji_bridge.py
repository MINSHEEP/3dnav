"""Selected-body adapter for the existing Astrall ROS driver (SDK 1.1.1)."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from diagnostic_msgs.msg import DiagnosticArray
from astrall_interfaces.msg import ConnectionStatus, SportStatus, PowerStatus, SystemStatus
from astrall_interfaces.srv import SetControlAuthority, SetMode
from lzp_interface.srv import SetTraversalMode


class XuanjiBridge(Node):
    def __init__(self, profile, frozen=False):
        super().__init__('dog_remote_bridge')
        self.profile, self.frozen = profile, frozen
        self.body_status = {}
        self.ever_connected = self.ever_controlled = False
        self.remote_active = self.armed = self.emergency_latched = False
        self.mapping_paused = False
        self.mapping_teleop_active = False
        self.connection = None
        self.sport = SportStatus.UNKNOWN
        self.connection_at = self.sport_at = self.heartbeat_at = self.command_at = 0.0
        self.battery = None
        self.hardware_error = False
        self.error = ''
        self.driver_active = False
        self.steps = []
        self.pending = None
        self.action = ''
        self.action_future = None
        self.step_at = 0.0
        self.process = None
        namespace = f'/xuanji_sdk_{os.getpid()}'
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(String, '/dog_status', qos)
        self.events = self.create_publisher(String, '/dog_remote/events', 10)
        self.velocity = self.create_publisher(Twist, namespace + '/cmd_vel', 1)
        self.lifecycle = self.create_client(ChangeState, namespace + '/astrall_driver/change_state')
        self.authority = self.create_client(SetControlAuthority, namespace + '/set_control_authority')
        self.mode = self.create_client(SetMode, namespace + '/set_mode')
        self.create_subscription(ConnectionStatus, namespace + '/connection_status', self.on_connection, qos)
        self.create_subscription(DiagnosticArray, namespace + '/driver_status', self.on_diagnostic, qos)
        self.create_subscription(SportStatus, namespace + '/sport_status', self.on_sport, 1)
        self.create_subscription(PowerStatus, namespace + '/power_status', self.on_power, 1)
        self.create_subscription(SystemStatus, namespace + '/system_status', self.on_system, 1)
        self.create_subscription(Twist, '/cmd_vel_safe', self.nav_velocity, 1)
        self.create_subscription(Bool, '/mapping/teleop_active', self.mapping_teleop, 10)
        self.create_subscription(Twist, '/dog_remote/cmd_vel', self.remote_velocity, 1)
        self.create_subscription(String, '/dog_remote/command', self.command, 10)
        self.create_subscription(Bool, '/dog_remote/enable', self.enable, 10)
        self.create_subscription(Empty, '/dog_remote/heartbeat', self.heartbeat, 1)
        self.create_subscription(Empty, '/motion_stop', self.safety_stop, 10)
        self.create_subscription(Empty, '/emergency_stop', self.safety_stop, 10)
        self.create_subscription(Empty, '/clear_emergency_stop', self.reset, 10)
        self.create_subscription(Empty, '/xuanji/mapping/lio_stop', self.mapping_stop, 10)
        self.create_subscription(Empty, '/xuanji/mapping/lio_resume', self.mapping_resume, 10)
        group = ReentrantCallbackGroup()
        self.create_service(Trigger, '/dog_remote/ensure_standing', self.ensure_standing, callback_group=group)
        self.create_service(Trigger, '/dog_remote/ensure_mapping_motion', self.ensure_mapping_motion, callback_group=group)
        self.create_service(SetTraversalMode, '/dog_remote/set_traversal_mode', self.traversal, callback_group=group)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.25, self.publish_status)
        install_root = Path(get_package_prefix('car_vel')).parent.resolve()
        driver_prefix = Path(get_package_prefix('astrall_driver')).resolve()
        for package in ('astrall_driver', 'astrall_interfaces', 'astrall_sdk_vendor'):
            if Path(get_package_prefix(package)).resolve() != install_root / package:
                raise RuntimeError(f'{package} 必须来自当前 3dnav/install；请重新编译并重启后台清除旧工作空间环境')
        executable = driver_prefix / 'lib/astrall_driver/astrall_driver_node'
        params = dict(use_fake_sdk=False, enable_lidar=False, enable_rgb=False,
                      robot_ip=profile['robot_ip'], local_ip=profile['local_ip'],
                      sdk_port=3600, sdk_client_port=profile['port'],
                      heartbeat_timeout_ms=20, sdk_timeout_ms=20, control_sdk_timeout_ms=20,
                      max_forward_velocity=profile['max_vx'], max_lateral_velocity=0.2,
                      max_yaw_velocity=profile['max_yaw'], cmd_vel_timeout_ms=250,
                      mode_switch_timeout_ms=8000)
        command = [str(executable), '--ros-args', '-r', f'__ns:={namespace}']
        for key, value in params.items():
            command += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']
        # Reuse the launch guard so a killed body worker cannot leave an SDK owner behind.
        self.driver_command = [
            sys.executable, '-m', 'lzp_service.launch_process', '--owner-pid', str(os.getpid()),
            '--shutdown-timeout', '1.0', '--', *command]
        self.process = subprocess.Popen(self.driver_command, start_new_session=True)
        self.begin('connect', [self.lifecycle_step(Transition.TRANSITION_CONFIGURE),
                               self.lifecycle_step(Transition.TRANSITION_ACTIVATE)])

    def event(self, message, level='info'):
        self.events.publish(String(data=json.dumps(dict(message=message, level=level), ensure_ascii=False)))

    def on_connection(self, message):
        self.connection, self.connection_at = message, time.monotonic()

    def on_sport(self, message):
        self.sport, self.sport_at = message.status, time.monotonic()

    def on_power(self, message):
        self.battery = round(message.percentage * 100)

    def on_system(self, message):
        self.hardware_error = bool(message.error_code or message.system_status in (message.ERROR, message.SHUTDOWN))
        if self.hardware_error:
            self.fail('璇玑硬件故障：' + (', '.join(message.error_messages) or str(message.error_code)))

    def on_diagnostic(self, message):
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            self.driver_active = values.get('operational_active') == '1'
            if (status.message.startswith('fault;') and self.action != 'reset'
                    or status.level == status.ERROR and not self.driver_active and self.action != 'reset'):
                self.fail(status.message)

    def connected(self):
        return bool(self.connection and self.connection.connected
                    and time.monotonic() - self.connection_at < 0.5)

    def controlled(self):
        return self.connected() and self.connection.has_control_authority

    def in_mode(self, mode):
        return self.sport == mode and time.monotonic() - self.sport_at < 1.0

    def lifecycle_step(self, transition):
        request = ChangeState.Request()
        request.transition.id = transition
        return self.lifecycle, request, lambda: True

    def mode_step(self, mode, expected):
        return self.mode, SetMode.Request(mode=mode), lambda: (
            self.controlled() and self.in_mode(expected) and self.sport_at >= self.step_at)

    def begin(self, action, steps):
        self.cancel('前一个动作已取消')
        self.stop()
        self.armed = False
        self.action, self.steps = action, steps
        self.action_future = Future()
        self.step_at = time.monotonic()
        return self.action_future

    def cancel(self, reason):
        if self.action_future is not None and not self.action_future.done():
            self.action_future.set_result((False, reason))
        if self.pending is not None and not self.pending.done():
            self.pending.cancel()
        self.pending = self.action_future = None
        self.steps, self.action = [], ''

    def stop(self):
        self.command_at = 0.0
        self.velocity.publish(Twist())

    def fail(self, message):
        if '0x8020' in message:
            message = '遥控器未释放控制权：三档开关回到 1 档，复位后重新申请 SDK 控制权。' + message
        elif '0x8022' in message:
            message = 'SDK 控制权异常，驱动已锁定：手柄三档开关回到 1 档，先按 R 复位，等待 reset 已确认后再按 9，等待 ctrl:1。' + message
        needs_stop = not self.emergency_latched or self.armed or self.command_at > 0
        changed = message != self.error
        self.error = message
        self.emergency_latched = True
        self.armed = False
        self.cancel(message)
        if needs_stop:
            self.stop()
        if changed:
            self.event(message, 'error')

    def stand(self, enter_move=True):
        if self.action == ('stand' if enter_move else 'fixed_stand'):
            return self.action_future
        if self.frozen or self.mapping_paused or self.emergency_latched or self.error or not self.ready():
            raise RuntimeError(self.error or '本体未就绪或停车锁定，请连接后复位')
        if self.battery is None or self.battery <= 15:
            raise RuntimeError('电量未知或不高于 15%，禁止开始运动')
        if not enter_move and not self.controlled():
            raise RuntimeError('请先按 9 申请 SDK 控制权，等待 ctrl:1')
        steps = [] if self.controlled() else [(self.authority, SetControlAuthority.Request(authority=SetControlAuthority.Request.SDK),
                  lambda: self.controlled() and self.connection_at >= self.step_at)]
        if not enter_move:
            steps.append(self.mode_step(SetMode.Request.FIXEDSTAND, SportStatus.FIXEDSTAND))
        elif not self.in_mode(SportStatus.MOVE):
            if not self.in_mode(SportStatus.FIXEDSTAND):
                steps.append(self.mode_step(SetMode.Request.FIXEDSTAND, SportStatus.FIXEDSTAND))
            steps.append(self.mode_step(SetMode.Request.MOVE, SportStatus.MOVE))
        self.ever_controlled = True
        if not steps:
            future = Future()
            self.stop()
            self.armed = enter_move
            future.set_result((True, '璇玑已处于运动模式'))
            return future
        return self.begin('stand' if enter_move else 'fixed_stand', steps)

    def request_control(self):
        if self.frozen or self.emergency_latched or not self.ready():
            raise RuntimeError(self.error or '请先复位，遥控器三档开关回到 1 档')
        self.ever_controlled = True
        return self.begin('authority', [(self.authority,
            SetControlAuthority.Request(authority=SetControlAuthority.Request.SDK),
            lambda: self.controlled() and self.connection_at >= self.step_at)])

    def move_mode(self):
        if (self.frozen or self.mapping_paused or self.emergency_latched or self.error
                or not self.controlled() or not self.in_mode(SportStatus.FIXEDSTAND)):
            raise RuntimeError('需 ctrl:1 且固定站立完成（0xB102），才能进入 MOVE')
        if self.battery is None or self.battery <= 15:
            raise RuntimeError('电量未知或不足，禁止开始运动')
        return self.begin('move', [self.mode_step(SetMode.Request.MOVE, SportStatus.MOVE)])

    def safety_stop(self, _message=None):
        if self.frozen:
            return
        self.cancel('停车已取消动作')
        self.armed = False
        self.emergency_latched = True
        self.stop()

    def reset(self, _message=None):
        if self.frozen or self.action in ('connect', 'reset'):
            return
        # Restart the vendor process after repeated unsolicited-authority faults on re-init.
        self.emergency_latched = True
        self.armed = False
        self.close()
        self.connection = None
        self.connection_at = self.sport_at = 0.0
        self.driver_active = False
        self.process = subprocess.Popen(self.driver_command, start_new_session=True)
        self.heartbeat_at = time.monotonic()
        self.event('璇玑：正在重启 SDK 会话，完成后请按 9 申请控制权')
        self.begin('reset', [self.lifecycle_step(Transition.TRANSITION_CONFIGURE),
                             self.lifecycle_step(Transition.TRANSITION_ACTIVATE)])

    def mapping_stop(self, _message):
        if not self.frozen:
            self.mapping_paused = True
            self.cancel('建图定位暂停')
            self.armed = False
            self.stop()

    def mapping_resume(self, _message):
        self.mapping_paused = False
        # A fresh ensure_standing request is required after LIO recovers.

    def enable(self, message):
        if self.frozen:
            return
        if self.action not in ('connect', 'reset'):
            self.cancel('遥控会话已切换')
        self.stop()
        self.armed = False
        self.remote_active = message.data
        self.heartbeat_at = time.monotonic()

    def heartbeat(self, _message):
        if self.remote_active:
            self.heartbeat_at = time.monotonic()

    def accept_velocity(self, message, allow_reverse=False):
        if (self.frozen or self.mapping_paused or not self.armed or self.emergency_latched or self.error
                or not self.controlled() or not self.in_mode(SportStatus.MOVE)):
            self.stop()
            return
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.stop()
            return
        command = Twist()
        command.angular.z = max(-self.profile['max_yaw'], min(self.profile['max_yaw'], message.angular.z))
        # Navigation is forward-only; teleoperation may reverse. Turning is in place.
        minimum = -self.profile['max_vx'] if allow_reverse else 0.0
        command.linear.x = 0.0 if abs(command.angular.z) > 1e-6 else max(minimum, min(self.profile['max_vx'], message.linear.x))
        # Only independent teleoperation permits lateral motion.
        if allow_reverse and abs(command.angular.z) <= 1e-6:
            command.linear.y = max(-0.2, min(0.2, message.linear.y))
        self.velocity.publish(command)
        self.command_at = time.monotonic()

    def remote_velocity(self, message):
        if self.remote_active:
            self.accept_velocity(message, allow_reverse=True)

    def mapping_teleop(self, message):
        self.mapping_teleop_active = bool(message.data)
        if not self.mapping_teleop_active:
            self.stop()

    async def ensure_mapping_motion(self, _request, response):
        try:
            if self.remote_active:
                raise RuntimeError('请先退出独立遥控，再开启建图遥控')
            response.success, response.message = await self.stand()
            self.mapping_teleop_active = response.success
        except RuntimeError as error:
            response.success, response.message = False, str(error)
        return response

    def nav_velocity(self, message):
        if not self.remote_active and (self.mapping_teleop_active or self.profile['navigation_verified']):
            self.accept_velocity(message, allow_reverse=self.mapping_teleop_active)

    def command(self, message):
        try:
            command = json.loads(message.data)['command']
            if self.frozen:
                raise RuntimeError('本体正在切换')
            if command == 'stop':
                if self.action not in ('connect', 'reset'):
                    self.cancel('停止已取消动作')
                self.stop()
                return
            if command == 'emergency':
                self.safety_stop()
                if self.controlled():
                    self.begin('emergency', [self.mode_step(SetMode.Request.DAMPING, SportStatus.DAMPING)])
                return
            if not self.remote_active:
                raise RuntimeError('遥控会话已停用，请刷新遥控页重新启用；如有停车锁定，再按 R 复位')
            if command == 'reset':
                self.reset()
            elif self.action:
                raise RuntimeError(f'璇玑正在执行 {self.action}，请等待当前动作确认后再切换模式；可按 0 停止或 R 复位')
            elif command == 'request_control':
                self.request_control()
            elif command == 'move_mode':
                self.move_mode()
            elif command == 'stand':
                self.stand(enter_move=False)
            elif command == 'lie_down':
                if self.emergency_latched or not self.controlled():
                    raise RuntimeError('请先复位并取得 SDK 控制权')
                self.begin('lie_down', [self.mode_step(SetMode.Request.FIXEDDOWN, SportStatus.FIXEDDOWN)])
            else:
                raise RuntimeError('璇玑不支持此动作或步态开关')
        except (ValueError, KeyError, TypeError, RuntimeError) as error:
            self.event(str(error), 'error')

    async def ensure_standing(self, _request, response):
        try:
            if not self.profile['navigation_verified']:
                raise RuntimeError('请先在设置页确认本体的雷达安装与导航安全参数')
            response.success, response.message = await self.stand()
        except RuntimeError as error:
            response.success, response.message = False, str(error)
        return response

    async def traversal(self, request, response):
        if request.mode not in ('', 'normal'):
            response.success, response.message = False, '璇玑尚未配置该地形模式'
            return response
        return await self.ensure_standing(request, response)

    def advance_action(self):
        if time.monotonic() - self.step_at > 10.0:
            raise RuntimeError(f'璇玑 {self.action} 请求或状态确认超时')
        client, request, confirmed = self.steps[0]
        if self.pending is None:
            if client.service_is_ready():
                self.step_at = time.monotonic()
                self.pending = client.call_async(request)
                if isinstance(request, SetMode.Request):
                    self.event(f'璇玑 {self.action}：请求模式 0x{request.mode:04X}，当前状态 0x{self.sport:04X}；等待实际完成')
                elif isinstance(request, SetControlAuthority.Request):
                    self.event('璇玑：已申请 SDK 控制权，等待 ctrl:1')
            return
        if not self.pending.done():
            return
        response = self.pending.result()
        if isinstance(request, ChangeState.Request):
            success = response.success
            reason = '生命周期切换失败'
        else:
            # RUNNING is an ACK, never proof of completed physical motion.
            success = response.result.success or response.result.code == 0x8006
            reason = response.result.message
        if not success:
            raise RuntimeError(reason)
        if not confirmed():
            return
        self.steps.pop(0)
        self.pending = None
        self.step_at = time.monotonic()
        if not self.steps:
            action, future = self.action, self.action_future
            self.action, self.action_future = '', None
            self.stop()
            if action in ('stand', 'move'):
                self.armed = True
            elif action == 'reset':
                self.error = ''
                self.emergency_latched = False
            future.set_result((True, f'璇玑 {action} 已确认'))
            self.event(f'璇玑 {action} 已确认')

    def tick(self):
        try:
            if self.process.poll() is not None:
                raise RuntimeError('Astrall 驱动退出，请检查日志并重新应用本体')
            if self.remote_active and time.monotonic() - self.heartbeat_at > 0.45:
                self.remote_active = False
                self.safety_stop()
            if self.action:
                if self.action not in ('connect', 'reset') and not self.connected():
                    raise RuntimeError('璇玑动作期间连接丢失，请检查后复位')
                self.advance_action()
            if self.armed and (not self.controlled() or not self.in_mode(SportStatus.MOVE)
                               or self.battery is None or self.battery <= 15):
                self.fail('控制权、模式、连接或电量不可用；请检查后复位')
            if self.command_at and time.monotonic() - self.command_at > 0.25:
                self.stop()
        except Exception as error:
            self.fail(str(error))

    def publish_status(self):
        connected = self.connected()
        self.status_publisher.publish(String(data=json.dumps(dict(
            connected=connected, stale=not connected, battery=self.battery,
            parked=connected and self.in_mode(SportStatus.FIXEDDOWN),
            posture={SportStatus.FIXEDDOWN: '趴下', SportStatus.FIXEDSTAND: '站立',
                     SportStatus.MOVE: '运动模式', SportStatus.DAMPING: '阻尼'}.get(self.sport, '过渡/未知'),
            sdk_control_authority=self.controlled(), sdk_sport_status=self.sport,
            model_label='璇玑轮式四足', remote_active=self.remote_active, active_action=self.action,
            motion_authorized=self.armed, emergency_latched=self.emergency_latched,
            hardware_error=self.hardware_error, error=self.error,
            navigation_verified=self.profile['navigation_verified'], local_udp_port=self.profile['port'],
            capabilities=dict(special_actions=False, traversal_modes=False, obstacle_stop=False,
                              lateral_motion=True, reverse_motion=True)), ensure_ascii=False)))

    def ready(self):
        # Lifecycle replies carry only a bool; the following diagnostic has the SDK reason.
        if self.error == '生命周期切换失败':
            return False
        if self.error:
            raise RuntimeError(self.error)
        return self.connected() and self.driver_active and self.action not in ('connect', 'reset') and not self.error

    def prepare_switch(self, switch_confirmed=False):
        if not switch_confirmed and (self.remote_active or self.action or not self.in_mode(SportStatus.FIXEDDOWN)):
            return False, '请先停止任务和遥控，并确认璇玑已趴下'
        self.cancel('本体切换')
        self.armed = self.remote_active = False
        self.stop()
        self.frozen = True
        return True, '璇玑已冻结控制'

    def activate(self):
        self.stop()
        self.frozen = False
        return True, '璇玑已激活；请显式站立后移动'

    def close(self):
        self.cancel('本体关闭')
        self.stop()
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=2.5)
