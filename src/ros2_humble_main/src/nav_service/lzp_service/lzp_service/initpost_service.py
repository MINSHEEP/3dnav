#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped
from lzp_interface.srv import SetInitialPose  # 替换为你的包名
import tf_transformations


class InitialPoseService(Node):
    def __init__(self):
        super().__init__("initial_pose_service")
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        # 使用自定义服务
        self.srv = self.create_service(SetInitialPose, "initial_pose_service", self.handle_set_pose)
        
        self.get_logger().info("Initial pose service is ready")

    def handle_set_pose(self, request, response):
        """处理设置初始位姿的请求"""
        try:
            # 将欧拉角转换为四元数
            quat = tf_transformations.quaternion_from_euler(
                request.roll, 
                request.pitch, 
                request.yaw
            )
            
            # 创建带协方差的位姿消息
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.pose.pose.position = Point(
                x=request.x, 
                y=request.y, 
                z=request.z
            )
            initial_pose.pose.pose.orientation = Quaternion(
                x=quat[0], 
                y=quat[1], 
                z=quat[2], 
                w=quat[3]
            )
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.header.frame_id = "map"
            
            # 发布初始位姿
            self.pub_pose.publish(initial_pose)
            
            # 设置服务响应
            response.success = True
            response.message = f"Initial pose set: pos=({request.x}, {request.y}, {request.z}), angles=({request.roll}, {request.pitch}, {request.yaw})"
            
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to set initial pose: {str(e)}"
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseService()

    try:
        node.get_logger().info("Initial pose service started, waiting for requests...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Initial pose service interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()