# ROS2 version of cmd_vel_to_move_motor.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Unitree SDK imports unchanged if installed into the same Python env


from .motor_api import MotorAPI

class CmdVelToDDS(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_move')
        self.get_logger().info('CmdVelToDDS node starting')

        # filter state
        self.last_cmd = Twist()
        self.alpha = 0.2
        self.max_delta_vx = 0.2
        self.max_delta_vy = 0.2
        self.max_delta_vyaw = 0.3

        # subscription
        qos_profile = 10
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile)

        # initialize Unitree channel (same API)
        # ChannelFactoryInitialize(0, "enp2s0")

        # start motor API
        self.motor = MotorAPI('/dev/ttyUSB0', 115200)
        self.motor.start()
        self.get_logger().info('cmd_vel_to_move_motor started.')

    def lowpass_filter(self, current, previous, zero_threshold=1e-3):
        if abs(current) < zero_threshold:
            return 0.0
        return self.alpha * current + (1 - self.alpha) * previous

    def clamp_delta(self, new, old, max_delta):
        delta = new - old
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        return old + delta

    def cmd_vel_callback(self, msg: Twist):
        vx_raw = msg.linear.x
        vy_raw = msg.linear.y
        vyaw_raw = msg.angular.z

        vx_filtered = self.lowpass_filter(vx_raw, self.last_cmd.linear.x)
        vy_filtered = self.lowpass_filter(vy_raw, self.last_cmd.linear.y)
        vyaw_filtered = self.lowpass_filter(vyaw_raw, self.last_cmd.angular.z)

        vx_safe = self.clamp_delta(vx_filtered, self.last_cmd.linear.x, self.max_delta_vx)
        vy_safe = self.clamp_delta(vy_filtered, self.last_cmd.linear.y, self.max_delta_vy)
        vyaw_safe = self.clamp_delta(vyaw_filtered, self.last_cmd.angular.z, self.max_delta_vyaw)

        self.last_cmd.linear.x = vx_safe
        self.last_cmd.linear.y = vy_safe
        self.last_cmd.angular.z = vyaw_safe

        vx = int(vx_safe * 1000)
        vy = int(vy_safe * 1000)
        # retain vyaw in radians/sec as float if MotorAPI expects it
        self.motor.set_vel(vx, vy, vyaw_safe)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToDDS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down CmdVelToDDS')
        node.motor.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()