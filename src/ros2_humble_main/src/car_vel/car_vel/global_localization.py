#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import time
import sys

import open3d as o3d
import rclpy
from rclpy.node import Node

from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf_transformations as tf_trans
import threading

class GlobalLocalizationNode(Node):
    def __init__(self):
        super().__init__('fast_lio_localization')
        
        # 参数定义
        self.MAP_VOXEL_SIZE = 0.4
        self.SCAN_VOXEL_SIZE = 0.1
        self.FREQ_LOCALIZATION = 0.5
        self.LOCALIZATION_TH = 0.95
        self.FOV = 6.28
        self.FOV_FAR = 30
        
        # 状态变量
        self.global_map = None
        self.initialized = False
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        
        self.get_logger().info('Localization Node Inited...')
        
        # Publishers
        self.pub_pc_in_map = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry, '/map_to_odom', 1)
        
        # Subscribers
        self.create_subscription(PointCloud2, '/cloud_registered', self.cb_save_cur_scan, 1)
        self.create_subscription(Odometry, '/Odometry', self.cb_save_cur_odom, 1)
        
        # 启动初始化过程
        self.init_timer = self.create_timer(1.0, self.initialize_process)
        
    def initialize_process(self):
        self.get_logger().info('Publishing')
        """初始化过程"""
        if self.global_map is None:
            self.get_logger().warn('Waiting for global map......')
            return
            
        if not self.initialized:
            self.get_logger().warn('Waiting for initial pose....')
            # 这里我们不阻塞，而是定期检查
            return
            
        # 初始化完成，启动定位线程
        self.init_timer.destroy()  # 销毁定时器
        self.start_localization_thread()
        
    def set_global_map(self, pc_msg):
        """设置全局地图"""
        self.global_map = o3d.geometry.PointCloud()
        self.global_map.points = o3d.utility.Vector3dVector(self.msg_to_array(pc_msg)[:, :3])
        self.global_map = self.voxel_down_sample(self.global_map, self.MAP_VOXEL_SIZE)
        self.get_logger().info('Global map received.')
        
    def set_initial_pose(self, pose_msg):
        """设置初始位姿"""
        if self.cur_scan is None:
            self.get_logger().warn('First scan not received!!!!!')
            return False
            
        initial_pose = self.pose_to_mat(pose_msg)
        self.get_logger().info(f'Initial pose: {initial_pose}')
        success = self.global_localization(initial_pose)
        if success:
            self.initialized = True
            self.get_logger().info('')
            self.get_logger().info('Initialize successfully!!!!!!')
            self.get_logger().info('')
        return success
        
    def pose_to_mat(self, pose_msg):
        """将Pose消息转换为变换矩阵"""
        p = pose_msg.pose.pose.position
        q = pose_msg.pose.pose.orientation
        t_mat = tf_trans.translation_matrix([p.x, p.y, p.z])
        r_mat = tf_trans.quaternion_matrix([q.x, q.y, q.z, q.w])
        return np.matmul(t_mat, r_mat)
        
    def msg_to_array(self, pc_msg):
        """将PointCloud2消息转换为numpy数组"""
        points_list = list(point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True))
        pc = np.array(points_list, dtype=np.float32)
        return pc
        
    def cb_save_cur_odom(self, odom_msg):
        """保存当前里程计数据"""
        self.cur_odom = odom_msg
        
    def cb_save_cur_scan(self, pc_msg):
        """保存当前扫描数据"""
        # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
        pc_msg.header.frame_id = 'odom'
        try:
            pc_msg.header.stamp = self.get_clock().now().to_msg()
        except Exception:
            pass
        self.pub_pc_in_map.publish(pc_msg)

        # 转换为pcd
        # fastlio给的field有问题 处理一下
        if len(pc_msg.fields) >= 8:
            pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                             pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                             pc_msg.fields[3], pc_msg.fields[7]]
        pc = self.msg_to_array(pc_msg)

        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])
        
    def voxel_down_sample(self, pcd, voxel_size):
        """体素下采样"""
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        except:
            # for open3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
        return pcd_down
        
    def registration_at_scale(self, pc_scan, pc_map, initial, scale):
        """在指定尺度下进行配准"""
        result_icp = o3d.pipelines.registration.registration_icp(
            self.voxel_down_sample(pc_scan, self.SCAN_VOXEL_SIZE * scale), 
            self.voxel_down_sample(pc_map, self.MAP_VOXEL_SIZE * scale),
            1.0 * scale, initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        )
        return result_icp.transformation, result_icp.fitness
        
    def inverse_se3(self, trans):
        """计算SE3变换的逆"""
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse
        
    def publish_point_cloud(self, publisher, header, pc):
        """发布点云消息"""
        # 构造点的列表
        points = []
        if pc.shape[1] >= 4:
            # 包含强度信息 (x, y, z, intensity)
            for i in range(len(pc)):
                points.append([pc[i, 0], pc[i, 1], pc[i, 2], pc[i, 3]])
        else:
            # 仅包含坐标信息 (x, y, z)
            for i in range(len(pc)):
                points.append([pc[i, 0], pc[i, 1], pc[i, 2]])
        
        # 定义点云字段
        if pc.shape[1] >= 4:
            fields = [
                point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='intensity', offset=12, datatype=point_cloud2.PointField.FLOAT32, count=1)
            ]
        else:
            fields = [
                point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1)
            ]
        
        # 创建并发布 PointCloud2 消息
        msg = point_cloud2.create_cloud(header, fields, points)
        publisher.publish(msg)
        
    def crop_global_map_in_FOV(self, global_map, pose_estimation, cur_odom):
        """裁剪FOV内的全局地图"""
        # 当前scan原点的位姿
        T_odom_to_base_link = self.pose_to_mat(cur_odom) if hasattr(cur_odom, 'pose') else np.eye(4)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        # 把地图转换到lidar系下
        global_map_in_map = np.array(global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        # 将视角内的地图点提取出来
        if self.FOV > 3.14:
            # 环状lidar 仅过滤距离
            indices = np.where(
                (global_map_in_base_link[:, 0] < self.FOV_FAR) &
                (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.FOV / 2.0)
            )
        else:
            # 非环状lidar 保前视范围
            # FOV_FAR>x>0 且角度小于FOV
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0) &
                (global_map_in_base_link[:, 0] < self.FOV_FAR) &
                (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.FOV / 2.0)
            )
        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

        # 发布fov内点云
        header = cur_odom.header
        header.frame_id = 'map'
        self.publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV
        
    def global_localization(self, pose_estimation):
        """全局定位"""
        # 用icp配准
        self.get_logger().info('Global localization by scan-to-map matching......')

        if self.cur_scan is None or self.global_map is None or self.cur_odom is None:
            self.get_logger().warn('Missing required data for localization')
            return False

        scan_tobe_mapped = copy.copy(self.cur_scan)

        tic = time.time()

        global_map_in_FOV = self.crop_global_map_in_FOV(self.global_map, pose_estimation, self.cur_odom)

        # 粗配准
        transformation, _ = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

        # 精配准
        transformation, fitness = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                        scale=1)
        toc = time.time()
        self.get_logger().info('Time: {}'.format(toc - tic))
        self.get_logger().info('')

        # 当全局定位成功时才更新map2odom
        if fitness > self.LOCALIZATION_TH:
            # T_map_to_odom = np.matmul(transformation, pose_estimation)
            self.T_map_to_odom = transformation

            # 发布map_to_odom
            map_to_odom = Odometry()
            xyz = tf_trans.translation_from_matrix(self.T_map_to_odom)
            quat = tf_trans.quaternion_from_matrix(self.T_map_to_odom)
            map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            # preserve original timestamp if available, else use now
            try:
                map_to_odom.header.stamp = self.cur_odom.header.stamp
            except Exception:
                map_to_odom.header.stamp = self.get_clock().now().to_msg()
            map_to_odom.header.frame_id = 'map'
            self.pub_map_to_odom.publish(map_to_odom)
            return True
        else:
            self.get_logger().warn('Not match!!!!')
            self.get_logger().warn('{}'.format(transformation))
            self.get_logger().warn('fitness score:{}'.format(fitness))
            return False
            
    def start_localization_thread(self):
        """启动定位线程"""
        threading.Thread(target=self.thread_localization, daemon=True).start()
        
    def thread_localization(self):
        """定期执行全局定位"""
        while rclpy.ok():
            # 每隔一段时间进行全局定位
            time.sleep(1 / self.FREQ_LOCALIZATION)
            # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
            if self.initialized:
                self.global_localization(self.T_map_to_odom)


def main(args=None):
    print('Starting Global Localization Node...')
    
    # 初始化rclpy
    rclpy.init(args=args)
    
    # 创建节点
    node = GlobalLocalizationNode()
    
    # try:
    #     # 运行节点
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # except Exception as e:
    #     node.get_logger().error(f'Error in main loop: {e}')
    # finally:
    #     try:
    #         node.get_logger().info('Shutting down localization node')
    #         node.destroy_node()
    #     except Exception:
    #         pass
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
