"""ROS interface shared with the Xingji body, backed by the ZSL-1 adapter."""
import json
import math
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from lzp_interface.srv import SetTraversalMode

from car_vel.zsl1_adapter import Zsl1Adapter


class Zsl1Bridge(Node):
    # Vendor demo allows 4 s after standUp. This guard is not a physical-pose
    # certificate: additionally require continuously stationary feedback.
    STAND_SETTLE_SECONDS = 4.0
    STAND_STABLE_SECONDS = 0.5
    STAND_TIMEOUT_SECONDS = 8.0

    def __init__(self, profile, frozen=False):
        super().__init__('dog_remote_bridge')
        self.profile = profile
        self.frozen = frozen
        self.ever_connected = self.ever_controlled = False
        self.body_status = {}
        self.remote_active = False
        self.command_at = self.heartbeat_at = 0.0
        self.last_tick = time.monotonic()
        self.motion_authorized = False
        self.mapping_lio_paused = False
        self.mapping_rearm_allowed = False
        self.adapter = Zsl1Adapter(profile)
        self.last_error = ''
        self.stand_future = None
        self.stand_started = 0.0
        self.stable_since = None
        self.phase = 'idle'
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(String, '/dog_status', qos)
        self.events = self.create_publisher(String, '/dog_remote/events', 10)
        self._stop_publisher = self.create_publisher(Empty, '/motion_stop', 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.nav_velocity, 1)
        self.create_subscription(Twist, '/dog_remote/cmd_vel', self.remote_velocity, 1)
        self.create_subscription(String, '/dog_remote/command', self.command, 10)
        self.create_subscription(Bool, '/dog_remote/enable', self.enable, 10)
        self.create_subscription(Empty, '/dog_remote/heartbeat', self.heartbeat, 1)
        self.create_subscription(Empty, '/motion_stop', self.safety_stop, 10)
        self.create_subscription(Empty, '/emergency_stop', self.safety_stop, 10)
        self.create_subscription(Empty, '/clear_emergency_stop', self.reset, 10)
        self.create_subscription(Empty, '/gangben/mapping/lio_stop', self.mapping_lio_stop, 10)
        self.create_subscription(Empty, '/gangben/mapping/lio_resume', self.mapping_lio_resume, 10)
        # Awaiting services must not own the timer/subscription callback group.
        self.stand_services = ReentrantCallbackGroup()
        self.create_service(Trigger, '/dog_remote/ensure_standing', self.ensure_standing,
                            callback_group=self.stand_services)
        self.create_service(SetTraversalMode, '/dog_remote/set_traversal_mode', self.traversal,
                            callback_group=self.stand_services)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.25, self.publish_status)

    def event(self, message, level='info'):
        self.events.publish(String(data=json.dumps({'message': message, 'level': level}, ensure_ascii=False)))

    def stop(self):
        self.command_at = 0.0
        self.adapter.stop()

    def mapping_lio_stop(self, _message):
        if self.frozen or self.mapping_lio_paused:
            return
        self.mapping_lio_paused = True
        self.mapping_rearm_allowed = False
        self._cancel_stand('建图定位异常，站立等待已取消')
        self.motion_authorized = False
        try:
            self.stop()  # Normal SDK move(0,0,0) if moving; never passive here.
            self.phase = 'mapping_paused'
            self.event('建图定位异常：钢镚已停步，等待定位恢复；未执行阻尼急停', 'warning')
        except Exception as error:
            self._fault(error)  # An actual SDK failure retains its fail-safe path.

    def mapping_lio_resume(self, _message):
        if self.frozen or not self.mapping_lio_paused:
            return
        self.mapping_lio_paused = False
        self.mapping_rearm_allowed = not self.adapter.emergency and not self.adapter.error
        self.motion_authorized = False
        try:
            self.stop()
            self.phase = 'idle'
            self.event('建图定位已恢复，旧行走指令已清除；需重新确认站立')
        except Exception as error:
            self._fault(error)

    def enable(self, message):
        if self.frozen:
            return
        try:
            self.stop()
        except Exception as error:
            self._fault(error)
            return
        if not message.data:
            self._cancel_stand('遥控已退出；需重新确认站立')
        self.remote_active = bool(message.data)
        self.heartbeat_at = time.monotonic()

    def heartbeat(self, _message):
        if self.remote_active:
            self.heartbeat_at = time.monotonic()

    def accept_velocity(self, message):
        if self.frozen or self.mapping_lio_paused or self.adapter.emergency or not self.motion_authorized:
            return
        now = time.monotonic()
        try:
            if now - self.command_at > 0.3:
                self.adapter.stop()
            self.adapter.set_velocity((message.linear.x, message.linear.y, message.angular.z))
            self.command_at = now
            self.ever_controlled = True
        except Exception as error:
            self._fault(error)

    def nav_velocity(self, message):
        if not self.remote_active and self.profile['navigation_verified']:
            self.accept_velocity(message)

    def remote_velocity(self, message):
        if self.remote_active:
            self.accept_velocity(message)

    def safety_stop(self, _message=None):
        if self.frozen:
            return
        self.mapping_lio_paused = False
        self.mapping_rearm_allowed = False
        self.command_at = 0.0
        self.motion_authorized = False
        self._cancel_stand('零速停车已取消站立等待')
        if not self.adapter.emergency:
            self.adapter.soft_stop()
        self.phase = 'fault' if self.adapter.error else 'stopped'
        self.event(self.adapter.error or '钢镚已请求零速停车并锁定运动；确认安全后复位，不会自动站立',
                   'error' if self.adapter.error else 'warning')

    def emergency(self, _message=None):
        # Only command=emergency from the remote button/T reaches passive().
        if self.frozen:
            return
        self.mapping_lio_paused = False
        self.mapping_rearm_allowed = False
        self.command_at = 0.0
        self.motion_authorized = False
        self._cancel_stand('急停已取消站立等待')
        self.phase = 'emergency'
        try:
            self.adapter.emergency_stop()
            self.event('钢镚已执行阻尼急停；确认安全后复位，再单独站立', 'warning')
        except Exception as error:
            self.event(str(error), 'error')

    def reset(self, _message=None):
        if not self.frozen:
            self.mapping_rearm_allowed = False
            self._cancel_stand('复位已取消站立等待')
            self.motion_authorized = False
            self.command_at = 0.0
            self.adapter.reset()
            self.phase = 'idle'
            self.last_error = ''
            self.event('软件锁定已解除，未自动站立；硬件故障需另行排除')

    def stand(self):
        if self.frozen:
            raise RuntimeError('本体正在切换，禁止站立')
        if self.mapping_lio_paused:
            raise RuntimeError('建图定位异常，恢复前禁止站立')
        if self.adapter.error:
            raise RuntimeError(f'钢镚 SDK 故障锁定：{self.adapter.error}；确认安全后复位')
        if self.adapter.emergency:
            raise RuntimeError('钢镚急停已锁定；确认安全后复位，再单独站立')
        if self.stand_future is not None:
            return self.stand_future
        state = self.adapter.state()
        if not state.get('connected'):
            raise RuntimeError('钢镚连接不可用')
        if (state.get('battery') or 0) <= 15:
            raise RuntimeError('钢镚电量不高于 15%，禁止开始运动')
        if self.motion_authorized and state['sdk_control_mode'] in (1, 18):
            future = Future()
            future.set_result((True, '钢镚已完成站立等待'))
            return future
        if state['sdk_control_mode'] == 18 and self.mapping_rearm_allowed:
            # move(0,0,0) can leave the SDK in walking mode. Require the
            # usual explicit confirmation and stationary feedback, without
            # sending passive() or a redundant standUp() transition.
            self.stop()
            self.stand_started = time.monotonic()
            self.stable_since = None
            self.phase = 'rearming_after_mapping'
            self.stand_future = Future()
            self.event('钢镚保持行走模式，等待静止反馈后重新放行移动')
            return self.stand_future
        if state['sdk_control_mode'] not in (0, 1, 10, 51):
            raise RuntimeError('钢镚当前不是可站立模式；请先安全停止，必要时阻尼趴下后再站立')
        self.motion_authorized = False
        self.stop()
        self.ever_controlled = True
        self.adapter.call('standUp')
        self.stand_started = time.monotonic()
        self.stable_since = None
        self.phase = 'standing'
        self.stand_future = Future()
        self.event('钢镚站立请求已发送，等待至少 4 秒及稳定反馈；期间不接收行走指令')
        return self.stand_future

    def _cancel_stand(self, reason):
        self.motion_authorized = False
        self.command_at = 0.0
        self.stable_since = None
        future, self.stand_future = self.stand_future, None
        if future is not None and not future.done():
            future.set_result((False, reason))
            self.event(reason, 'warning')
        self.phase = 'idle'

    def _fault(self, error):
        self.mapping_rearm_allowed = False
        self._cancel_stand(str(error))
        if not self.adapter.emergency:
            self.adapter.fail_safe(str(error))
        self.phase = 'fault'
        message = self.adapter.error or str(error)
        if message != self.last_error:
            self.event(message, 'error')
            self.get_logger().error(message)
            self.last_error = message
            # Also revoke upstream Nav2/preview authority. The subscriber's
            # echo sees an existing latch and never retries the SDK zero call.
            self._stop_publisher.publish(Empty())

    def _advance_stand(self, state, now, elapsed):
        if now - self.stand_started >= self.STAND_TIMEOUT_SECONDS:
            raise RuntimeError(f'钢镚站立等待超时：模式={state.get("sdk_control_mode")}，'
                               f'速度={state.get("body_velocity")}；未放行运动')
        velocity = state.get('body_velocity')
        gyro = state.get('body_gyro')
        expected_mode = (18 if self.phase == 'rearming_after_mapping' else 1)
        stable = (state.get('sdk_control_mode') == expected_mode and velocity is not None
                  and len(velocity) == 3
                  and all(math.isfinite(v) and abs(v) < 0.03 for v in velocity)
                  and gyro is not None and len(gyro) == 3
                  and all(math.isfinite(v) and abs(v) < 0.1 for v in gyro))
        if not stable or elapsed > 0.2:
            self.stable_since = None
        if stable and self.stable_since is None:
            self.stable_since = now
        if (stable and now - self.stand_started >= self.STAND_SETTLE_SECONDS
                and now - self.stable_since >= self.STAND_STABLE_SECONDS):
            self.stop()  # Discard all pre-ready commands; no Move in standby.
            self.motion_authorized = True
            self.mapping_rearm_allowed = False
            self.phase = 'ready'
            future, self.stand_future = self.stand_future, None
            message = '钢镚站立等待完成，反馈稳定；可发送新的行走指令'
            future.set_result((True, message))
            self.event(message)

    def command(self, message):
        try:
            data = json.loads(message.data)
            command = data.get('command') if isinstance(data, dict) else None
            if self.frozen:
                raise RuntimeError('本体切换中，命令已拒绝')
            if command == 'stop':
                self.stop()
                if self.stand_future is not None:
                    self._cancel_stand('停止指令已取消站立等待；未放行运动')
                return
            if command == 'emergency':
                self.emergency()
                return
            if self.mapping_lio_paused and command != 'reset':
                raise RuntimeError('建图定位异常，恢复前禁止移动或站立')
            if not self.remote_active:
                raise RuntimeError('遥控会话未启用')
            if command == 'reset':
                self.reset()
            elif command == 'stand':
                self.stand()
            elif command == 'lie_down':
                self._cancel_stand('趴下指令已取消站立等待')
                self.stop()
                self.motion_authorized = False
                self.ever_controlled = True
                self.adapter.call('lieDown')
                self.event('已请求钢镚趴下')
            else:
                raise RuntimeError('钢镚首版不支持此动作/步态/停障开关；不会发送替代命令')
        except Exception as error:
            if self.adapter.error:
                self._fault(error)
            else:
                self.event(str(error), 'error')

    async def ensure_standing(self, _request, response):
        try:
            if not self.profile['navigation_verified']:
                raise RuntimeError('请先在设置页确认钢镚导航安装参数并完成校准')
            response.success, response.message = await self.stand()
        except Exception as error:
            if self.adapter.error:
                self._fault(error)
            response.success, response.message = False, str(error)
        return response

    async def traversal(self, request, response):
        if request.mode not in ('normal', ''):
            response.success, response.message = False, '钢镚尚未验证该地形模式，禁止执行坡道/楼梯/越障连接段'
            return response
        return await self.ensure_standing(request, response)

    def tick(self):
        now = time.monotonic()
        elapsed, self.last_tick = now - self.last_tick, now
        try:
            if self.frozen:
                return
            state = self.adapter.state()
            if not state.get('connected'):
                self._cancel_stand('钢镚连接丢失；重新连接后需重新站立')
                self.stop()
                return
            if self.remote_active and now - self.heartbeat_at > 0.45:
                self.remote_active = False
                self._cancel_stand('遥控心跳超时，已撤销运动授权')
                self.stop()
            if (state.get('battery') or 0) <= 15:
                self._cancel_stand('钢镚电量不足，未放行运动')
            if self.mapping_lio_paused:
                self.adapter.tick(elapsed, False)
                return
            if self.stand_future is not None:
                self._advance_stand(state, now, elapsed)
                return
            if self.motion_authorized and state.get('sdk_control_mode') not in (1, 18):
                self._cancel_stand('钢镚退出站立/行走模式，需重新确认站立')
            permitted = self.motion_authorized and now - self.command_at <= 0.3
            self.adapter.tick(elapsed, permitted)
        except Exception as error:
            self._fault(error)

    def publish_status(self):
        try:
            state = self.adapter.state()
        except Exception as error:
            state = {'connected': False, 'stale': True, 'parked': False, 'error': str(error)}
        state.update(remote_active=self.remote_active,
                     active_action='stand' if self.stand_future is not None else '',
                     stand_phase=self.phase, motion_authorized=self.motion_authorized,
                     mapping_lio_paused=self.mapping_lio_paused,
                     capabilities={'special_actions': False, 'traversal_modes': False,
                                   'obstacle_stop': False},
                     error=self.adapter.error or state.get('error', ''),
                     navigation_verified=self.profile['navigation_verified'])
        self.status_publisher.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def ready(self):
        return self.adapter.state().get('connected') is True

    def prepare_switch(self):
        state = self.adapter.state()
        unused = not self.ever_connected and not self.ever_controlled
        if self.stand_future is not None or self.remote_active or not (state.get('parked') or unused):
            return False, '请先让钢镚趴下并退出遥控；失联或站立中的本体不能切换'
        self.frozen = True
        self.motion_authorized = False
        self.stop()
        return True, '钢镚已安全停放并锁定'

    def activate(self):
        self.mapping_lio_paused = False
        self.mapping_rearm_allowed = False
        self._cancel_stand('本体启用；需单独站立')
        self.stop()
        self.motion_authorized = False
        self.frozen = False

    def close(self):
        self._cancel_stand('钢镚控制服务关闭')
        self.stop()
