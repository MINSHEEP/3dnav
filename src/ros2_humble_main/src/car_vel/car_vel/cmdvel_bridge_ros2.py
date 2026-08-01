#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import socket
import json
import time
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class CmdVelBridgeNode(Node):
    def __init__(self):
        super().__init__('zsl_cmdvel_bridge')

        self.declare_parameter('dog_ip', '192.168.96.2')
        self.declare_parameter('dog_port', 8080)
        self.declare_parameter('send_rate_hz', 20)
        self.declare_parameter('scale_factor', 1.5)  # 速度放大系数
        self.declare_parameter('cmd_timeout_sec', 0.3)
        self.declare_parameter('allow_lateral', False)

        dog_ip = self.get_parameter('dog_ip').value
        dog_port = self.get_parameter('dog_port').value
        self.send_rate = self.get_parameter('send_rate_hz').value
        self.scale = self.get_parameter('scale_factor').value
        self.cmd_timeout = self.get_parameter('cmd_timeout_sec').value
        self.allow_lateral = self.get_parameter('allow_lateral').value
        self.emergency_latched = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._closed = False
        self.dog_addr = (dog_ip, dog_port)
        self.get_logger().info(f'UDP target: {dog_ip}:{dog_port}')
        self.get_logger().info(f'Velocity scale factor: {self.scale}')
        self.get_logger().info(
            f'Lateral motion: {"enabled" if self.allow_lateral else "disabled"}')

        # QoS 配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmdvel_callback,
            qos_profile
        )
        self.emergency_sub = self.create_subscription(
            Empty, '/emergency_stop', self.emergency_stop_callback, qos_profile)
        self.clear_emergency_sub = self.create_subscription(
            Empty, '/clear_emergency_stop', self.clear_emergency_stop_callback,
            qos_profile)
        self.get_logger().info('Subscribed to /cmd_vel with QoS VOLATILE')

        self.lock = threading.Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.last_cmd_time = time.monotonic()

        self.timer = self.create_timer(1.0 / self.send_rate, self.timer_callback)

        self.get_logger().info('正在初始化机械狗...')
        self.send_reset()
        time.sleep(1.5)
        self.send_standup()
        time.sleep(1.5)
        self.get_logger().info('机械狗初始化完成，已站起')

    def send_udp(self, payload):
        if self._closed:
            return
        try:
            json_str = json.dumps(payload)
            self.sock.sendto(json_str.encode('utf-8'), self.dog_addr)
        except Exception as e:
            self.get_logger().error(f'UDP send error: {e}')

    def send_reset(self):
        self.send_udp({"msg_type": 3, "cmd_code": 4})
        self.get_logger().info('发送复位命令')

    def send_standup(self):
        self.send_udp({"msg_type": 3, "cmd_code": 10})
        self.get_logger().info('发送站起命令')

    def cmdvel_callback(self, msg: Twist):
        # 获取原始速度，乘以放大系数，并限幅到 [-1, 1]
        vx = msg.linear.x * self.scale
        # Stability-first differential-drive mode.  Keep this final hardware
        # guard even if an old browser page or another publisher sends linear.y.
        vy = msg.linear.y * self.scale if self.allow_lateral else 0.0
        vtheta = msg.angular.z * self.scale

        # 限幅
        vx = max(-1.0, min(1.0, vx))
        vy = max(-1.0, min(1.0, vy))
        vtheta = max(-1.0, min(1.0, vtheta))

        # 加入死区（避免微小抖动）
        deadband = 0.02
        if abs(vx) < deadband:
            vx = 0.0
        if abs(vy) < deadband:
            vy = 0.0
        if abs(vtheta) < deadband:
            vtheta = 0.0

        with self.lock:
            if self.emergency_latched:
                self.vx = 0.0
                self.vy = 0.0
                self.vtheta = 0.0
                return
            self.vx = vx
            self.vy = vy
            self.vtheta = vtheta
            self.last_cmd_time = time.monotonic()

    def timer_callback(self):
        with self.lock:
            if (
                self.emergency_latched
                or time.monotonic() - self.last_cmd_time > self.cmd_timeout
            ):
                # Fail safe: never keep walking on an old navigation command
                # if Nav2, ROS communication, or the velocity smoother stops.
                self.vx = 0.0
                self.vy = 0.0
                self.vtheta = 0.0
            vx = self.vx
            vy = self.vy
            vtheta = self.vtheta

        # 构造三字段格式 JSON
        payload = {
            "vel_move_frontback": float(vx),
            "vel_move_leftright": float(vy),
            "vel_turn_leftright": float(vtheta)
        }
        # 调试打印（可选，可注释掉）
        # self.get_logger().info(f'Sending: {payload}')
        self.send_udp(payload)

    def emergency_stop_callback(self, _message):
        with self.lock:
            self.emergency_latched = True
            self.vx = 0.0
            self.vy = 0.0
            self.vtheta = 0.0
        self.get_logger().warn(
            'Emergency stop latched; ignoring velocity commands until cleared.')
        self._send_zero_burst()

    def clear_emergency_stop_callback(self, _message):
        with self.lock:
            self.emergency_latched = False
            self.vx = 0.0
            self.vy = 0.0
            self.vtheta = 0.0
            self.last_cmd_time = time.monotonic()
        self.get_logger().info('Emergency stop latch cleared.')

    def _send_zero_burst(self):
        zero = {
            "vel_move_frontback": 0.0,
            "vel_move_leftright": 0.0,
            "vel_turn_leftright": 0.0
        }
        for _ in range(3):
            self.send_udp(zero)
            time.sleep(0.02)

    def stop_and_close(self):
        """Send redundant zero commands before releasing the UDP socket."""
        if self._closed:
            return
        with self.lock:
            self.vx = 0.0
            self.vy = 0.0
            self.vtheta = 0.0
        self._send_zero_burst()
        self.sock.close()
        self._closed = True

    def __del__(self):
        if hasattr(self, 'sock') and not getattr(self, '_closed', True):
            self.sock.close()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_and_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
