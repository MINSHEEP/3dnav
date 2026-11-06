"""One supervised ROS body worker: Xingji UDP or Gangben ZSL-1 SDK.

The private multiprocessing pipe is the switch barrier. It is serviced in the
same thread as command callbacks, so a successful freeze cannot race a new
remote command. No SDK or ROS objects are shared between processes.
"""
import json
import time


def run_worker(body, profile, generation, frozen, pipe):
    import rclpy
    from rclpy.parameter import Parameter
    from car_vel.dog_remote_bridge import DogRemoteBridge
    from car_vel.velocity_shaping import TurnCommandShaper
    from car_vel.motion_diagnostics import MotionTrace, age, velocity, body_reason
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, Empty

    node = None
    ownership = None
    try:
        from car_vel.body_lock import acquire_body_lock
        ownership = acquire_body_lock()
        rclpy.init(args=[])
        if body == 'xingji':
            class XingjiBody(DogRemoteBridge):
                def __init__(self):
                    self.frozen = frozen
                    self.ever_connected = self.ever_controlled = False
                    self.body_status = {}
                    self.nav_at = 0.0
                    self.nav_armed = False
                    self.shaper = TurnCommandShaper()
                    self.nav_was_active = False
                    self.navigation_gait_blocked = False
                    self.mapping_lio_paused = False
                    self.mapping_teleop_active = False
                    self.mapping_rearm_after_neutral = False
                    self.mapping_release_seen = False
                    self._diag_navigation_active=False
                    self._diag_nav_received_at=0.0
                    self._diag_received=[0.0,0.0,0.0]
                    self._diag_send_at=0.0
                    self._diag_sent=[0.0,0.0,0.0]
                    self._diag_send_success=None
                    self._diag_send_kind='none'
                    self._diag_tick_permitted=None
                    self._diag_tick_gait_ready=None
                    super().__init__(parameter_overrides=[
                        Parameter('dog_ip', value=profile['robot_ip']),
                        Parameter('dog_port', value=profile['port']),
                        Parameter('camera_upstream_url', value=f"http://{profile['robot_ip']}:8080")])
                    self._motion_trace=MotionTrace(self.get_logger(),'NAV_BODY')
                    self.create_subscription(Twist, '/cmd_vel_safe', self.nav_velocity, 1)
                    self.create_subscription(Bool, '/navigation_active', self.navigation_diagnostic_active, 1)
                    self.create_subscription(Empty, '/motion_stop', lambda _: self._safety_stop(), 10)
                    # Old publishers remain compatible but may no longer
                    # request a native emergency or change body posture.
                    self.create_subscription(Empty, '/emergency_stop', lambda _: self._safety_stop(), 10)
                    self.create_subscription(Empty, '/clear_emergency_stop', lambda _: self.clear_latch(), 10)
                    self.create_subscription(Empty, '/xingji/mapping/lio_stop', self.mapping_lio_stop, 10)
                    self.create_subscription(Empty, '/xingji/mapping/lio_resume', self.mapping_lio_resume, 10)
                    self.create_subscription(Bool, '/xingji/mapping/teleop_active',
                                             self.mapping_teleop_active_callback, 10)
                    self.create_subscription(Empty, '/xingji/mapping/teleop_neutral',
                                             self.mapping_teleop_neutral_callback, 10)

                def mapping_teleop_active_callback(self, message):
                    if self.frozen:
                        return
                    was_active = self.mapping_teleop_active
                    self.mapping_teleop_active = bool(message.data)
                    if self.mapping_teleop_active:
                        return
                    if not was_active:
                        return
                    self.mapping_rearm_after_neutral = False
                    self.mapping_release_seen = False
                    self.nav_armed = False
                    self.nav_at = 0.0
                    self.nav_was_active = False
                    self.shaper.reset()
                    self._send_zero_burst()

                def mapping_teleop_neutral_callback(self, _message):
                    if (self.frozen or self.mapping_lio_paused
                            or not self.mapping_teleop_active
                            or not self.mapping_rearm_after_neutral):
                        return
                    now = time.monotonic()
                    if (self.emergency_latched
                            or now - self.last_status_time > self.status_timeout
                            or self.robot_status.get('motion_mode') != 2):
                        self.mapping_rearm_after_neutral = False
                        return
                    # Unlike /cmd_vel_safe, only the mapping page emits this
                    # after the last physically held direction key is released.
                    self.mapping_release_seen = True

                def mapping_lio_stop(self, _message):
                    if self.frozen or self.mapping_lio_paused:
                        return
                    self.mapping_lio_paused = True
                    self._cancel_action()
                    # Remember only an already confirmed standing session.
                    # The actual motion gate stays disarmed throughout the pause.
                    self.mapping_rearm_after_neutral = bool(
                        (self.nav_armed or self.mapping_rearm_after_neutral)
                        and self.mapping_teleop_active
                        and not self.remote_active and not self.emergency_latched)
                    self.mapping_release_seen = False
                    with self.state_lock:
                        self.requested_velocity = (0.0, 0.0, 0.0)
                        self.last_velocity_command = 0.0
                    self.nav_armed = False
                    self.nav_at = 0.0
                    self.nav_was_active = False
                    self.shaper.reset()
                    self._send_zero_burst()
                    self._event('warning', '建图定位异常：已暂停星玑移动，等待定位恢复；未发送机械狗急停命令。')

                def mapping_lio_resume(self, _message):
                    if self.frozen or not self.mapping_lio_paused:
                        return
                    self.mapping_lio_paused = False
                    now = time.monotonic()
                    self.mapping_rearm_after_neutral = bool(
                        self.mapping_rearm_after_neutral
                        and self.mapping_teleop_active
                        and not self.emergency_latched and not self.remote_active
                        and now - self.last_status_time <= self.status_timeout
                        and self.robot_status.get('motion_mode') == 2)
                    self.mapping_release_seen = False
                    with self.state_lock:
                        self.requested_velocity = (0.0, 0.0, 0.0)
                        self.last_velocity_command = 0.0
                    self.nav_armed = False
                    self.nav_at = 0.0
                    self.shaper.reset()
                    self._send_zero_burst()
                    if self.mapping_rearm_after_neutral:
                        self._event('info', '建图定位已恢复；请松开方向键后重新按键继续，无需重开遥控模式。')
                    else:
                        self._event('warning', '建图定位已恢复；站立或通信状态未确认，请重新开启遥控模式。')

                def _safety_stop(self):
                    self.mapping_rearm_after_neutral = False
                    self.mapping_release_seen = False
                    self.nav_armed = False
                    self.nav_at = 0.0
                    self.nav_was_active = False
                    self.shaper.reset()
                    super()._safety_stop()

                def clear_latch(self):
                    if not self.frozen and self.emergency_latched:
                        self.mapping_rearm_after_neutral = False
                        self.mapping_release_seen = False
                        self.nav_armed = False
                        self.shaper.reset()
                        self.nav_at = 0.0
                        self._reset_robot()

                def _enable_callback(self, message):
                    if self.frozen:
                        return
                    self.mapping_rearm_after_neutral = False
                    self.mapping_release_seen = False
                    self.nav_at = 0.0
                    self.nav_armed = False
                    self.shaper.reset()
                    super()._enable_callback(message)

                def _command_callback(self, message):
                    if self.frozen:
                        return
                    if self.mapping_lio_paused:
                        try:
                            command = json.loads(message.data).get('command')
                        except (AttributeError, TypeError, ValueError):
                            command = None
                        if command not in ('emergency', 'reset'):
                            self._event('warning', '建图定位异常，暂不接受移动或动作命令。')
                            return
                    self.ever_controlled = True
                    self.mapping_rearm_after_neutral = False
                    self.mapping_release_seen = False
                    self.nav_at = 0.0
                    self.shaper.reset()
                    super()._command_callback(message)

                def _velocity_callback(self, message):
                    if not self.frozen and not self.mapping_lio_paused:
                        if message.linear.x != 0 or message.linear.y != 0 or message.angular.z != 0:
                            self.ever_controlled = True
                        super()._velocity_callback(message)

                def _ensure_standing_callback(self, request, response):
                    if self.frozen or self.mapping_lio_paused or not profile['navigation_verified']:
                        response.success, response.message = False, '本体切换中、建图定位暂停或尚未确认导航安装参数'
                        return response
                    self.ever_controlled = True
                    response = super()._ensure_standing_callback(request, response)
                    self.nav_armed = bool(response.success)
                    self.mapping_rearm_after_neutral = False
                    self.mapping_release_seen = False
                    return response

                def _set_traversal_mode_callback(self, request, response):
                    if self.frozen:
                        response.success, response.message = False, '本体切换中'
                        return response
                    self.ever_controlled = True
                    return super()._set_traversal_mode_callback(request, response)

                def nav_velocity(self, message):
                    # Record even a rejected input; never refresh nav_at or
                    # arm the body from diagnostic receipt alone.
                    self._diag_nav_received_at=time.monotonic()
                    self._diag_received=velocity(message)
                    if self.frozen or self.mapping_lio_paused or self.remote_active or self.emergency_latched:
                        return
                    now = time.monotonic()
                    if self.mapping_rearm_after_neutral:
                        if (now - self.last_status_time > self.status_timeout
                                or self.robot_status.get('motion_mode') != 2):
                            self.mapping_rearm_after_neutral = False
                            self.mapping_release_seen = False
                            return
                        # An arbiter timeout also publishes zero, so it cannot
                        # stand in for the page's explicit all-keys-up signal.
                        if not self.mapping_release_seen:
                            return
                        if any(abs(value) > 1e-6 for value in (
                                message.linear.x, message.linear.y, message.angular.z)):
                            return
                        self.mapping_rearm_after_neutral = False
                        self.mapping_release_seen = False
                        self.nav_armed = True
                        self.nav_at = 0.0
                        self.shaper.reset()
                        return
                    if now - self.nav_at > 0.3:
                        self.shaper.reset()
                    self.shaper.set_target(message.linear.x, message.linear.y, message.angular.z)
                    self.nav_at = now

                def navigation_diagnostic_active(self, message):
                    self._diag_navigation_active=getattr(message,'data',None) is True

                def _send_velocity(self, values):
                    result=super()._send_velocity(values)
                    self._diag_send_at=time.monotonic()
                    self._diag_sent=list(values)
                    self._diag_send_success=result if type(result) is bool else None
                    self._diag_send_kind='velocity'
                    return result

                def _send_zero_burst(self):
                    result=super()._send_zero_burst()
                    self._diag_send_at=time.monotonic()
                    self._diag_sent=[0.0,0.0,0.0]
                    self._diag_send_success=None  # Original burst has no aggregate result.
                    self._diag_send_kind='zero_burst'
                    return result

                def _control_tick(self):
                    self._diag_tick_permitted=None
                    self._diag_tick_gait_ready=None
                    try:
                        return self._control_tick_impl()
                    finally:
                        # Never mask a control exception, re-arm or send an
                        # extra datagram because diagnostic collection failed.
                        try:
                            self._record_navigation_execution()
                        except Exception:
                            pass

                def _record_navigation_execution(self):
                    now=time.monotonic()
                    record=dict(stage='xingji_body',schema=1,body_id='xingji',body_generation=generation,
                        stamp_ns=self.get_clock().now().nanoseconds,
                        frozen=self.frozen,mapping_paused=self.mapping_lio_paused,
                        remote_active=self.remote_active,emergency=self.emergency_latched,
                        armed=self.nav_armed,status_fresh=now-self.last_status_time<=self.status_timeout,
                        status_age_sec=age(now,self.last_status_time),
                        standing=self.robot_status.get('motion_mode')==2,
                        motion_mode=self.robot_status.get('motion_mode'),model=self.robot_status.get('model'),
                        gait_ready=self._diag_tick_gait_ready,
                        permitted_by_body_tick=self._diag_tick_permitted,
                        input_seen=self._diag_nav_received_at>0,input_age_sec=age(now,self._diag_nav_received_at),
                        command_age_sec=age(now,self.nav_at),input=list(self._diag_received),
                        command_discarded=self._diag_nav_received_at>0 and self.nav_at<=0,
                        sent_sdk_velocity=list(self._diag_sent),send_age_sec=age(now,self._diag_send_at),
                        input_units='m/s,m/s,rad/s',send_units='vendor_normalized',
                        send_success=self._diag_send_success,send_kind=self._diag_send_kind,
                        execution_acknowledged=False)
                    record['reason_code']=body_reason(record)
                    self._motion_trace.emit(record,now=now,active=self._diag_navigation_active)

                def _control_tick_impl(self):
                    if self.frozen:
                        return
                    if self.mapping_lio_paused:
                        if (time.monotonic() - self.last_status_time > self.status_timeout
                                or self.emergency_latched
                                or not self.mapping_teleop_active
                                or self.robot_status.get('motion_mode') != 2):
                            self.mapping_rearm_after_neutral = False
                            self.mapping_release_seen = False
                        self._send_velocity((0.0, 0.0, 0.0))
                        return
                    if self.mapping_rearm_after_neutral and (
                            self.emergency_latched or self.remote_active
                            or not self.mapping_teleop_active
                            or time.monotonic() - self.last_status_time > self.status_timeout
                            or self.robot_status.get('motion_mode') != 2):
                        self.mapping_rearm_after_neutral = False
                        self.mapping_release_seen = False
                    if self.remote_active:
                        self.shaper.reset()
                        self.nav_at = 0.0
                        self.nav_was_active = False
                        return super()._control_tick()
                    now = time.monotonic()
                    if now - self.last_status_time > self.status_timeout:
                        self.nav_armed = False
                        self.mapping_rearm_after_neutral = False
                        self.mapping_release_seen = False
                    gait_ready = (self.mapping_teleop_active or self._navigation_gait_confirmed())
                    self._diag_tick_gait_ready=gait_ready
                    if self.nav_armed and not gait_ready:
                        # Do not trust cached/optimistically selected models.
                        # A physical gait change during autonomy requires a new
                        # confirmed initialization; discard the old velocity.
                        self.nav_armed = False
                        self.nav_at = 0.0
                        self.shaper.reset()
                        self._send_zero_burst()
                        self.nav_was_active = False
                        if not self.navigation_gait_blocked:
                            self._event('warning', '星玑导航越障模式反馈失效，已零速停车；请重新确认越障模式后再启动导航。')
                        self.navigation_gait_blocked = True
                    elif gait_ready:
                        self.navigation_gait_blocked = False
                    permitted = (self.nav_armed and gait_ready and not self.emergency_latched and now - self.nav_at <= 0.3
                                 and now - self.last_status_time <= self.status_timeout
                                 and self.robot_status.get('motion_mode') == 2)
                    self._diag_tick_permitted=permitted
                    if permitted:
                        self.ever_controlled = True
                        self.nav_was_active = True
                        self._send_velocity(self.shaper.step(0.05))
                    else:
                        self.shaper.reset()
                        if self.nav_was_active:
                            self._send_zero_burst()
                            self.nav_was_active = False

                def prepare_switch(self, powered_off_confirmed=False, switch_confirmed=False):
                    parked = (self.body_status.get('connected') is True and
                              self.body_status.get('motion_mode') in (0, 1))
                    unused = not self.ever_connected and not self.ever_controlled
                    offline_confirmed = powered_off_confirmed and self.body_status.get('connected') is False
                    if not switch_confirmed and (self.remote_active or self.active_action or not (parked or unused or offline_confirmed)):
                        return False, '请先让星玑趴下、退出遥控，并停止所有任务；失联本体不能直接切换'
                    self.frozen = True
                    self._cancel_action()
                    self.remote_active = False
                    self.mapping_rearm_after_neutral = False
                    self.mapping_release_seen = False
                    self.nav_at = 0.0
                    self.shaper.reset()
                    self._send_zero_burst()
                    return True, '旧本体已锁定'

                def activate(self):
                    self.mapping_lio_paused = False
                    self.mapping_teleop_active = False
                    self.mapping_rearm_after_neutral = False
                    self.mapping_release_seen = False
                    self.nav_at = 0.0
                    self.nav_armed = False
                    self.shaper.reset()
                    self.frozen = False

                def ready(self):
                    return self.body_status.get('connected') is True

            node = XingjiBody()
        elif body == 'gangben_zsl1':
            from car_vel.zsl1_bridge import Zsl1Bridge
            node = Zsl1Bridge(profile, frozen=frozen)
        elif body == 'xuanji':
            try:
                from car_vel.xuanji_bridge import XuanjiBridge
            except ImportError as error:
                raise RuntimeError('璇玑依赖未加载：请先编译并 source sw01_ws/install/setup.bash') from error
            node = XuanjiBridge(profile, frozen=frozen)
        else:
            raise ValueError('不支持的本体')

        real_publisher = node.status_publisher

        class StatusPublisher:
            def publish(self, message):
                data = json.loads(message.data)
                data.update(body_id=body, generation=generation, switching=node.frozen)
                if body == 'xingji':
                    data['navigation_armed']=node.nav_armed
                    data['navigation_input_age_sec']=age(time.monotonic(),node._diag_nav_received_at)
                    data['navigation_execution_reason']=node._motion_trace.last_record.get('reason_code','not_sampled')
                    data['navigation_execution_stamp_ns']=node._motion_trace.last_record.get('stamp_ns')
                    data['mapping_lio_paused'] = node.mapping_lio_paused
                    data['parked'] = data.get('connected') is True and data.get('motion_mode') in (0, 1)
                    data['capabilities'] = {'special_actions': True, 'traversal_modes': True,
                                            'obstacle_stop': True}
                node.body_status = data
                node.ever_connected = node.ever_connected or data.get('connected') is True
                message.data = json.dumps(data, ensure_ascii=False)
                real_publisher.publish(message)
                pipe.send({'kind': 'status', 'data': data})

        node.status_publisher = StatusPublisher()
        pending_ready = None
        parent_seen = time.monotonic()
        while rclpy.ok():
            while pipe.poll():
                request = pipe.recv()
                operation = request['operation']
                parent_seen = time.monotonic()
                if operation == 'heartbeat':
                    continue
                if request.get('deadline') is not None and parent_seen > request['deadline']:
                    pipe.send({'kind': 'reply', 'id': request['id'], 'success': False,
                               'message': '操作已过期，未执行'})
                    continue
                if operation == 'shutdown':
                    return
                if operation == 'ready':
                    pending_ready = request['id']
                    continue
                if operation == 'prepare':
                    success, message = node.prepare_switch()
                elif operation == 'prepare_xingji_powered_off' and body == 'xingji':
                    success, message = node.prepare_switch(powered_off_confirmed=True)
                elif operation == 'prepare_confirmed':
                    success, message = node.prepare_switch(switch_confirmed=True)
                elif operation == 'activate':
                    node.activate()
                    success, message = True, '本体已启用；未自动站立'
                else:
                    success, message = False, '未知操作'
                pipe.send({'kind': 'reply', 'id': request['id'],
                           'success': success, 'message': message})
            if pending_ready is not None and node.ready():
                pipe.send({'kind': 'reply', 'id': pending_ready, 'success': True})
                pending_ready = None
            if time.monotonic() - parent_seen > 4.0:
                raise RuntimeError('本体管理器心跳丢失，停止控制')
            rclpy.spin_once(node, timeout_sec=0.02)
    except (KeyboardInterrupt, EOFError, BrokenPipeError):
        pass
    except Exception as error:
        try:
            pipe.send({'kind': 'error', 'message': str(error)})
        except (OSError, EOFError):
            pass
    finally:
        if node:
            try:
                node.close()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if ownership:
            ownership.close()
        pipe.close()
