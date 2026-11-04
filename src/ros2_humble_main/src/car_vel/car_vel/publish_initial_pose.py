#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point, Quaternion
import tf_transformations


class PublishInitialPose(Node):
    def __init__(self):
        super().__init__("publish_initial_pose")
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        
        # 声明参数
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)

    def publish_pose(self):
        # 获取参数值
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        roll = self.get_parameter('roll').value
        pitch = self.get_parameter('pitch').value
        yaw = self.get_parameter('yaw').value
        
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.pose.pose.position = Point(x=x, y=y, z=z)
        initial_pose.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        self.pub_pose.publish(initial_pose)

        self.get_logger().info(f"Initial Pose: {x} {y} {z} {roll} {pitch} {yaw}")


def main(args=None):
    rclpy.init(args=args)
    node = PublishInitialPose()

    # 发布初始位姿
    loop_count = 0
    while loop_count < 2:
        node.publish_pose()
        rclpy.spin_once(node, timeout_sec=1.0)
        loop_count += 1

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
