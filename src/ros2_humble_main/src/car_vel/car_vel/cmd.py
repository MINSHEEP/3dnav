#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
import math

class OdometryMonitor(Node):
    def __init__(self):
        super().__init__('odometry_monitor')
        
        # 创建QoS配置，用于获取发布者信息
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',  # 订阅/Odometry话题
            self.odometry_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        # 记录上次打印时间
        self.last_print_time = time.time()
        self.print_interval = 1  # 1秒打印一次

        # 记录订阅时的发布者信息
        self.get_logger().info("Subscribing to /Odometry topic...")
        self.log_publishers()

    def log_publishers(self):
        """记录当前的发布者"""
        publishers_info = self.get_publishers_info_by_topic('/Odometry')
        if publishers_info:
            self.get_logger().info(f'Found {len(publishers_info)} publishers to /Odometry:')
            for info in publishers_info:
                self.get_logger().info(f'  - Publisher: {info.node_name}, Namespace: {info.node_namespace}')
        else:
            self.get_logger().info('No publishers found for /Odometry')

    def quaternion_to_euler(self, x, y, z, w):
        """将四元数转换为欧拉角（弧度）"""
        # 计算roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # 计算pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # 计算yaw (z-axis rotation) - 这通常是最关心的朝向角
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def odometry_callback(self, msg):
        current_time = time.time()
        
        # 检查是否到达打印间隔
        if current_time - self.last_print_time >= self.print_interval:
            # 获取线速度和角速度
            linear = msg.twist.twist.linear
            angular = msg.twist.twist.angular
            
            self.get_logger().info(f'Received Odometry: linear=({linear.x}, {linear.y}, {linear.z}), '
                                  f'angular=({angular.x}, {angular.y}, {angular.z})')
            
            # 打印速度信息
            linear_speed = (linear.x**2 + linear.y**2 + linear.z**2)**0.5
            angular_speed = (angular.x**2 + angular.y**2 + angular.z**2)**0.5
            
            self.get_logger().info(f'Speed info - Linear speed: {linear_speed:.3f} m/s, '
                                  f'Angular speed: {angular_speed:.3f} rad/s')
            
            # 获取位置信息
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            
            # 转换四元数为欧拉角
            roll, pitch, yaw = self.quaternion_to_euler(
                orientation.x, 
                orientation.y, 
                orientation.z, 
                orientation.w
            )
            
            self.get_logger().info(f'Position: ({position.x:.3f}, {position.y:.3f}, {position.z:.3f})')
            self.get_logger().info(f'Orientation (Quaternion): ({orientation.x:.3f}, {orientation.y:.3f}, {orientation.z:.3f}, {orientation.w:.3f})')
            self.get_logger().info(f'Orientation (Euler - Roll/Pitch/Yaw): ({math.degrees(roll):.2f}°, {math.degrees(pitch):.2f}°, {math.degrees(yaw):.2f}°)')
            
            # 更新上次打印时间
            self.last_print_time = current_time
            
            # 获取当前活跃的发布者（只在打印时获取）
            publishers_info = self.get_publishers_info_by_topic('/Odometry')
            if publishers_info:
                publisher_names = [info.node_name for info in publishers_info]
                self.get_logger().info(f'Current publishers of /Odometry: {", ".join(publisher_names)}')
                #打印回车换行
                print()
            else:
                self.get_logger().info('No publishers currently available for /Odometry')

def main(args=None):
    rclpy.init(args=args)
    odometry_monitor = OdometryMonitor()
    rclpy.spin(odometry_monitor)
    odometry_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()