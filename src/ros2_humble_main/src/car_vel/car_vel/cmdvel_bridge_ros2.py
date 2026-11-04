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
from car_vel.velocity_shaping import TurnCommandShaper

class CmdVelBridgeNode(Node):
    def __init__(self):
        super().__init__('zsl_cmdvel_bridge')

        self.declare_parameter('dog_ip', '192.168.96.2')
        self.declare_parameter('dog_port', 8080)
        self.declare_parameter('send_rate_hz', 20)
        # Keep the established walking speed without amplifying turn rate.
        # A single 1.5 factor made the measured yaw response about 50% larger
        # than Nav2 requested, increasing foot slip and goal-heading overshoot.
        self.declare_parameter('linear_scale_factor', 1.5)
        self.declare_parameter('angular_scale_factor', 1.0)
        self.declare_parameter('min_in_place_turn_command', 0.15)
        self.declare_parameter('turn_command_accel', 0.4)
        self.declare_parameter('command_release_deadband', 0.01)
        self.declare_parameter('cmd_timeout_sec', 0.3)
        self.declare_parameter('allow_lateral', False)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')

        dog_ip = self.get_parameter('dog_ip').value
        dog_port = self.get_parameter('dog_port').value
        self.send_rate = self.get_parameter('send_rate_hz').value
        self.linear_scale = self.get_parameter('linear_scale_factor').value
        self.angular_scale = self.get_parameter('angular_scale_factor').value
        self.min_in_place_turn = self.get_parameter(
            'min_in_place_turn_command').value
        self.cmd_timeout = self.get_parameter('cmd_timeout_sec').value
        self.allow_lateral = self.get_parameter('allow_lateral').value
        self.turn_shaper = TurnCommandShaper(
            linear_scale=self.linear_scale,
            angular_scale=self.angular_scale,
            allow_lateral=self.allow_lateral,
            turn_floor=self.min_in_place_turn,
            angular_accel=float(self.get_parameter('turn_command_accel').value),
            release_deadband=float(self.get_parameter('command_release_deadband').value),
        )
        self.cmd_vel_topic = str(
            self.get_parameter('cmd_vel_topic').value).strip()
        if not self.cmd_vel_topic:
            raise RuntimeError('cmd_vel_topic must not be empty')
        self.emergency_latched = False

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._closed = False
        self.dog_addr = (dog_ip, dog_port)
        self.get_logger().info(f'UDP target: {dog_ip}:{dog_port}')
        self.get_logger().info(
            f'Velocity scale factors: linear={self.linear_scale}, '
            f'angular={self.angular_scale}')
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
            self.cmd_vel_topic,
            self.cmdvel_callback,
            qos_profile
        )
        self.emergency_sub = self.create_subscription(
            Empty, '/emergency_stop', self.emergency_stop_callback, qos_profile)
        self.motion_stop_sub = self.create_subscription(
            Empty, '/motion_stop', self.emergency_stop_callback, qos_profile)
        self.clear_emergency_sub = self.create_subscription(
            Empty, '/clear_emergency_stop', self.clear_emergency_stop_callback,
            qos_profile)
        self.get_logger().info(
            f'Subscribed to {self.cmd_vel_topic} with QoS VOLATILE')

        self.lock = threading.Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.last_cmd_time = time.monotonic()
        self.last_shape_time = self.last_cmd_time

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
        # Store a target here; advance the ramp only at the UDP output cadence.
        with self.lock:
            if self.emergency_latched:
                self._reset_velocity()
                return
            now = time.monotonic()
            if now - self.last_cmd_time > self.cmd_timeout:
                self._reset_velocity()
            self.turn_shaper.set_target(msg.linear.x, msg.linear.y, msg.angular.z)
            self.last_cmd_time = now

    def _reset_velocity(self):
        """Caller holds lock: discard targets, output and hysteresis together."""
        self.turn_shaper.reset()
        self.vx = self.vy = self.vtheta = 0.0

    def timer_callback(self):
        with self.lock:
            now = time.monotonic()
            elapsed = min(max(0.0, now - self.last_shape_time), 1.0 / self.send_rate)
            self.last_shape_time = now
            if (
                self.emergency_latched
                or now - self.last_cmd_time > self.cmd_timeout
            ):
                # Fail safe: never keep walking on an old navigation command
                # if Nav2, ROS communication, or the velocity smoother stops.
                self._reset_velocity()
            else:
                self.vx, self.vy, self.vtheta = self.turn_shaper.step(elapsed)
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
            self._reset_velocity()
        self.get_logger().warn(
            'Emergency stop latched; ignoring velocity commands until cleared.')
        self._send_zero_burst()

    def clear_emergency_stop_callback(self, _message):
        with self.lock:
            self.emergency_latched = False
            self._reset_velocity()
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
            self._reset_velocity()
        self._send_zero_burst()
        self.sock.close()
        self._closed = True

    def __del__(self):
        if hasattr(self, 'sock') and not getattr(self, '_closed', True):
            self.sock.close()


def main(args=None):
    from car_vel.body_lock import acquire_body_lock
    ownership = acquire_body_lock(expected_body='xingji')
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
        ownership.close()


if __name__ == '__main__':
    main()
