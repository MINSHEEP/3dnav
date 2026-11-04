"""ROS settings endpoints attached to the authoritative launch manager."""
import json
import multiprocessing
import threading
import time

import rclpy
from std_msgs.msg import Empty, String
from rclpy.qos import QoSProfile, DurabilityPolicy
from lzp_interface.srv import ControlCampusMission, ControlInspectionTask, SetRobotBody

from lzp_service.robot_body import BodySupervisor


class ProcessWorker:
    def __init__(self, body, profile, generation, frozen):
        from car_vel.robot_body_bridge import run_worker
        context = multiprocessing.get_context('spawn')
        self.pipe, child = context.Pipe()
        self.process = context.Process(target=run_worker,
            args=(body, profile, generation, frozen, child),
            name='robot_body_' + body, daemon=True)
        self.process.start()
        child.close()
        self.next_id = 0
        self.pending = []
        self.send_lock = threading.Lock()
        self.heartbeat_stop = threading.Event()
        self.heartbeat_thread = threading.Thread(target=self._heartbeat, daemon=True)
        self.heartbeat_thread.start()

    def _heartbeat(self):
        # Launch/stop callbacks may block for seconds. Their callback group is
        # not a valid liveness clock for the independent hardware worker.
        while not self.heartbeat_stop.wait(0.5):
            try:
                if not self.alive():
                    return
                self.send('heartbeat')
            except (OSError, EOFError):
                return

    def alive(self):
        return self.process.is_alive()

    def send(self, operation, deadline=None):
        with self.send_lock:
            self.next_id += 1
            self.pipe.send({'operation': operation, 'id': self.next_id, 'deadline': deadline})
            return self.next_id

    def drain(self):
        messages = []
        try:
            while self.pipe.poll():
                messages.append(self.pipe.recv())
        except (EOFError, OSError):
            pass
        return messages

    def poll(self):
        if self.alive():
            try:
                self.send('heartbeat')
            except (OSError, EOFError):
                pass
        result, self.pending = self.pending + self.drain(), []
        return result

    def request(self, operation, timeout):
        deadline = time.monotonic() + timeout
        request_id = self.send(operation, deadline)
        next_heartbeat = 0.0
        while self.alive() and time.monotonic() < deadline:
            for message in self.drain():
                if message.get('kind') == 'reply' and message.get('id') == request_id:
                    return message
                if message.get('kind') == 'error':
                    raise RuntimeError(message['message'])
                if message.get('kind') == 'status':
                    self.pending = [message]
            if time.monotonic() >= next_heartbeat:
                self.send('heartbeat')
                next_heartbeat = time.monotonic() + 0.5
            time.sleep(0.02)
        raise RuntimeError('本体响应超时或进程退出；请检查 SDK、机器人回传 IP 和网络')

    def close(self):
        if self.alive():
            try:
                self.send('shutdown')
            except (OSError, EOFError):
                pass
            self.process.join(timeout=3.0)
        if self.alive():
            # Used only after a successful parking barrier, or on host shutdown.
            self.process.terminate()
            self.process.join(timeout=2.0)
        if not self.alive():
            self.heartbeat_stop.set()
            self.heartbeat_thread.join(timeout=1.0)
            self.pipe.close()


class RobotBodyManager:
    def __init__(self, node):
        self.node = node
        self.supervisor = BodySupervisor(ProcessWorker)
        self.mode_at = 0.0
        self.mode_locked = True
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(String, '/robot_body/status', qos)
        self.offline_publisher = node.create_publisher(String, '/dog_status', qos)
        node.create_subscription(String, '/navigation/mode', self.mode_status, qos)
        node.create_service(SetRobotBody, '/robot_body/select', self.select)
        node.create_timer(0.5, self.tick)
        try:
            self.supervisor.start()
        except Exception as error:
            self.supervisor.error = str(error)

    def mode_status(self, message):
        try:
            data = json.loads(message.data)
            if type(data.get('locked')) is not bool:
                raise ValueError('invalid mode status')
            self.mode_locked = data['locked']
            self.mode_at = time.monotonic()
        except (ValueError, TypeError, AttributeError):
            self.mode_locked = True

    def idle_check(self):
        self.node._reap_exited_processes()
        with self.node.process_lock:
            if self.node.running_processes:
                raise ValueError('请先停止建图/导航程序，再切换本体')
        if time.monotonic() - self.mode_at > 3.0:
            raise ValueError('任务状态缺失或过期，暂不能切换本体')
        if self.mode_locked:
            raise ValueError('导航、巡检或园区任务仍在运行/暂停，请先结束任务')

    def stop_for_switch(self):
        # A separate node can receive service replies while the manager's
        # single-threaded executor is handling the switch request.
        node = rclpy.create_node('robot_body_switch_cleanup')
        mode = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def update(message):
            mode.clear()
            mode.update(json.loads(message.data))
        node.create_subscription(String, '/navigation/mode', update, qos)
        publisher = node.create_publisher(Empty, '/cancel_navigation', 10)
        try:
            deadline = time.monotonic() + 5.0
            while not mode and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not mode:
                raise RuntimeError('无法读取任务状态，未切换本体')
            for key, service_type, name, command in (
                ('campus_mission_active', ControlCampusMission, '/control_campus_mission', 'stop'),
                ('inspection_active', ControlInspectionTask, '/inspection/control', 'cancel'),
            ):
                if not mode.get(key):
                    continue
                client = node.create_client(service_type, name)
                if not client.wait_for_service(timeout_sec=3.0):
                    raise RuntimeError(f'无法连接 {name}，未切换本体')
                request = service_type.Request()
                request.command = command
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                if not future.done() or future.result() is None or not future.result().success:
                    raise RuntimeError(f'无法结束 {name} 任务，未切换本体')
            if mode.get('execution_mode') != 'idle':
                deadline = time.monotonic() + 3.0
                while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                if publisher.get_subscription_count() == 0:
                    raise RuntimeError('无法连接导航取消接口，未切换本体')
                publisher.publish(Empty())
            stopped, message = self.node._stop_all_modes()
            if not stopped:
                raise RuntimeError(f'停止建图或导航失败：{message}')
            self.node.publish_status()
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if mode.get('locked') is False:
                    return
            raise RuntimeError('任务未确认停止，未切换本体')
        finally:
            node.destroy_node()

    def select(self, request, response):
        # A switch and a page-driven launch cannot interleave.
        with self.node.transition_lock:
            try:
                profile = json.loads(request.profile_json)
                confirmed = profile.pop('offline_xingji_powered_off', False)
                switch_confirmed = profile.pop('switch_confirmed', False)
                if type(confirmed) is not bool:
                    raise ValueError('星玑关机确认无效')
                if type(switch_confirmed) is not bool:
                    raise ValueError('切换确认无效')
                self.supervisor.apply(request.body_id, profile,
                                      request.expected_version,
                                      self.stop_for_switch if switch_confirmed else self.idle_check,
                                      offline_xingji_powered_off=confirmed,
                                      switch_confirmed=switch_confirmed)
                response.success = True
                response.message = '本体已切换并保存；未自动站立。更换本体后请重新确认定位'
            except Exception as error:
                response.success = False
                response.message = str(error)
            self.tick()
            response.status_json = json.dumps(self.supervisor.snapshot(), ensure_ascii=False)
        return response

    def navigation_error(self):
        self.supervisor.poll()
        snapshot = self.supervisor.snapshot()
        config = self.supervisor.config
        if snapshot['switching'] or self.supervisor.error or not config:
            return self.supervisor.error or '本体正在切换或尚未配置'
        if not snapshot['connected'] or snapshot['active_body'] != config['selected']:
            return '所选本体未连接，请在设置页检查或重新应用'
        state = snapshot['status']
        if state.get('emergency_latched') or state.get('hardware_error') is True or state.get('error'):
            return state.get('error') or '本体处于停车锁定或故障状态，请先检查并人工复位'
        if not config['profiles'][config['selected']]['navigation_verified']:
            return '请先在设置页确认该本体的雷达安装与导航安全参数'
        return ''

    def tick(self):
        self.supervisor.poll()
        snapshot = self.supervisor.snapshot()
        if not self.supervisor.worker or not self.supervisor.worker.alive():
            self.offline_publisher.publish(String(data=json.dumps({
                'connected': False, 'stale': True, 'battery': None,
                'body_id': (self.supervisor.config or {}).get('selected'),
                'hardware_error': None, 'error': self.supervisor.error,
                'posture': '本体控制未运行',
            }, ensure_ascii=False)))
        try:
            self.idle_check()
            snapshot['locked'], snapshot['lock_reason'] = False, ''
        except ValueError as error:
            snapshot['locked'], snapshot['lock_reason'] = True, str(error)
        self.publisher.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))

    def close(self):
        self.supervisor.close()
