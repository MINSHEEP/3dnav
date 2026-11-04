#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, TransformBroadcaster
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
from tf2_ros import TransformException


class TF2DTransformer(Node):
    def __init__(self):
        super().__init__('trans_tf_2d')
        
        # 创建 TF2 缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建 TF2 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 设置发布频率
        self.timer = self.create_timer(0.1, self.broadcast_transform)  # 1000Hz
        
        self.get_logger().info('TF 2D Transformer node started')

    def broadcast_transform(self):
        try:
            # 查找 map 到 body 的变换
            transform = self.tf_buffer.lookup_transform(
                'map',  # 目标坐标系
                'base_link',  # 源坐标系
                rclpy.time.Time(),  # 最新可用时间
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 获取位置信息
            robot_pose_x = transform.transform.translation.x
            robot_pose_y = transform.transform.translation.y
            robot_pose_z = 0.0  # 强制 Z 为 0
            
            # 获取旋转信息（仅保留 Z 轴旋转，其他设为 0）
            robot_orientation_x = transform.transform.rotation.x
            robot_orientation_y = transform.transform.rotation.y
            robot_orientation_z = transform.transform.rotation.z
            robot_orientation_w = transform.transform.rotation.w
            
            # 创建新的变换消息
            new_transform = TransformStamped()
            new_transform.header.stamp = self.get_clock().now().to_msg()
            new_transform.header.frame_id = 'map'
            new_transform.child_frame_id = 'body_2d'
            
            # 设置位置（Z 轴设为 0）
            new_transform.transform.translation.x = robot_pose_x
            new_transform.transform.translation.y = robot_pose_y
            new_transform.transform.translation.z = robot_pose_z
            
            # 设置旋转（只保留 Z 轴旋转分量，其他保持不变）
            # 注意：这里保持四元数的完整性，只对旋转做2D处理
            new_transform.transform.rotation.x = 0.0
            new_transform.transform.rotation.y = 0.0
            # 保持 Z 和 W 分量，但需要重新标准化四元数以确保是纯Z轴旋转
            # 为了确保是2D变换，我们可以将旋转简化为仅Z轴旋转
            new_transform.transform.rotation.z = robot_orientation_z
            new_transform.transform.rotation.w = robot_orientation_w
            
            # 确保四元数是单位四元数（标准化）
            norm = (robot_orientation_z**2 + robot_orientation_w**2)**0.5
            if norm > 0:
                new_transform.transform.rotation.z = robot_orientation_z / norm
                new_transform.transform.rotation.w = robot_orientation_w / norm
            else:
                new_transform.transform.rotation.w = 1.0

            # 发布变换
            self.tf_broadcaster.sendTransform(new_transform)
            
        except TransformException as ex:
            self.get_logger().error(f'Transform exception: {ex}')
            return


def main(args=None):
    rclpy.init(args=args)
    
    node = TF2DTransformer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
