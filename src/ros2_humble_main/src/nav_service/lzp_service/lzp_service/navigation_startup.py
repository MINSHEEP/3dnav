"""Read-only navigation startup diagnosis. Never activates nodes or moves a body."""
import json
import time
from .launch_process import nav2_process_conflicts


class StartupTracker:
    REQUIRED = ('map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator')

    def __init__(self, timeout=60.0):
        self.timeout = timeout
        self.pid = 0
        self.started = 0.0
        self.last_launch = None

    def observe_launch(self, running, pid, now):
        pid = int(pid) if running else 0
        changed = pid != self.pid
        if changed:
            self.started = now
        self.pid = pid
        self.last_launch = now
        return changed

    def snapshot(self, now, states, action_ready, *, containers=()):
        elapsed = max(0, now - self.started) if self.pid else 0
        missing = [n for n in self.REQUIRED if n not in states]
        inactive = [f'{n}={states[n]}' for n in self.REQUIRED
                    if n in states and states[n] != 'active']
        if self.last_launch is None or now - self.last_launch > 4:
            state, message = 'status_stale', '启动管理器状态未收到或已过期，请检查后台'
        elif not self.pid:
            state, message = 'stopped', '导航进程已停止；进入遥控或建图会停止导航'
        elif missing:
            state, message = 'loading_nodes', '等待导航节点：' + '、'.join(missing)
        elif inactive:
            state, message = 'activating', '等待节点激活：' + '、'.join(inactive)
        elif not action_ready:
            state, message = 'waiting_action', '节点已激活，等待 /navigate_to_pose 动作服务'
        else:
            state, message = 'ready', '导航节点与动作服务已就绪，请选择并加载地图'
        if self.pid and elapsed >= self.timeout and state in (
                'loading_nodes', 'activating', 'waiting_action'):
            state, message = 'timeout', '导航启动超时；' + message
        if len(containers) > 1 or (containers and not self.pid):
            pids = '、'.join(str(item['pid']) for item in containers)
            state = 'process_conflict'
            message = (f'导航容器冲突或残留（PID {pids}）；'
                       '请停止旧后台并清理残留，再重试导航服务')
        return dict(state=state, message=message, pid=self.pid,
                    elapsed_sec=round(elapsed, 1), missing_nodes=missing,
                    lifecycle=states, action_ready=bool(action_ready),
                    container_processes=list(containers))


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from std_msgs.msg import String
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from lzp_interface.msg import LaunchStatus

    class Monitor(Node):
        def __init__(self):
            super().__init__('navigation_startup_monitor')
            self.declare_parameter('startup_timeout_sec', 60.0)
            self.tracker = StartupTracker(max(15.0, float(
                self.get_parameter('startup_timeout_sec').value)))
            self.states = {}
            self.pending = {}
            self._lifecycle_clients = {n: self.create_client(GetState, f'/{n}/get_state')
                            for n in self.tracker.REQUIRED}
            self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
            self.pub = self.create_publisher(String, '/navigation/startup/status', 1)
            self.create_subscription(LaunchStatus, '/launch_status', self.launch, 10)
            self.create_timer(1.0, self.tick)
            self.last_message = ''

        def launch(self, msg):
            if self.tracker.observe_launch(msg.navigation_running, msg.navigation_pid,
                                           time.monotonic()):
                for future, _ in self.pending.values():
                    future.cancel()
                self.pending.clear()
                self.states.clear()

        def tick(self):
            now = time.monotonic()
            names = set(self.get_node_names())
            for name, client in self._lifecycle_clients.items():
                if name not in names or not self.tracker.pid:
                    self.states.pop(name, None)
                item = self.pending.get(name)
                if item:
                    future, sent = item
                    if future.done():
                        try:
                            result = future.result()
                            if name in names and self.tracker.pid and result:
                                self.states[name] = (result.current_state.label, now)
                        except Exception:
                            self.states.pop(name, None)
                        del self.pending[name]
                    elif now - sent > 3:
                        future.cancel()
                        del self.pending[name]
                        self.states.pop(name, None)
                if self.tracker.pid and name in names and name not in self.pending:
                    if client.service_is_ready():
                        self.pending[name] = (client.call_async(GetState.Request()), now)
            states = {n: self.states[n][0] if n in self.states
                      and now - self.states[n][1] <= 4 else '状态查询无响应'
                      for n in self.tracker.REQUIRED if n in names}
            try:
                containers = nav2_process_conflicts()
                process_error = ''
            except (OSError, RuntimeError) as error:
                containers, process_error = [], str(error)
            data = self.tracker.snapshot(now, states, self.action.server_is_ready(),
                                         containers=containers)
            data['process_inspection_error'] = process_error
            # Heartbeat rather than a retained one-shot: browser reconnect gets
            # fresh state and never needs to trust a previous launch generation.
            self.pub.publish(String(data=json.dumps(data, ensure_ascii=False)))
            key = (data['pid'], data['state'], data['message'])
            if key != self.last_message:
                self.get_logger().info('NAV_STARTUP ' + json.dumps(data, ensure_ascii=False))
                self.last_message = key

    rclpy.init(args=args)
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
