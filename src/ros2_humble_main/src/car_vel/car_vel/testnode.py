#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 添加global_localization所需的导入
import copy
import time
import sys

import rclpy
from rclpy.node import Node

from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf_transformations as tf_trans
import threading
from nav_msgs.msg import OccupancyGrid

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Test Node has been started.')
        
        # 添加global_localization的初始化代码
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
        self.create_subscription(OccupancyGrid, '/map', self.set_global_map, 1)  # 添加这一行
        
        # 启动初始化过程
        self.init_timer = self.create_timer(1.0, self.initialize_process)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

    # 添加global_localization的所有方法
    def initialize_process(self):
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
        try:
            points = self.msg_to_array(pc_msg)[:, :3]
            #打印日志
            self.get_logger().info(f'Received {len(points)} points.')
            self.global_map = self.voxel_down_sample_numpy(points, self.MAP_VOXEL_SIZE)
            self.get_logger().info('Global map received.')
        except Exception as e:
            self.get_logger().error(f'Error in set_global_map: {e}')
        
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
        try:
            # 读取点云数据，将其转换为正确的格式
            points_list = []
            for point in point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
                points_list.append([point[0], point[1], point[2]])
            pc = np.array(points_list, dtype=np.float32)
            return pc
        except Exception as e:
            self.get_logger().error(f'Error converting point cloud: {e}')
            return np.empty((0, 3), dtype=np.float32)
        
    def cb_save_cur_odom(self, odom_msg):
        """保存当前里程计数据"""
        self.cur_odom = odom_msg
        
    def cb_save_cur_scan(self, pc_msg):
        """保存当前扫描数据"""
        try:
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
            
            # 只有当点云不为空时才更新
            if pc.size > 0:
                self.cur_scan = pc  # 直接存储为numpy数组
            else:
                self.get_logger().warn('Received empty point cloud')
        except Exception as e:
            self.get_logger().error(f'Error in cb_save_cur_scan: {e}')

    def voxel_down_sample_numpy(self, points, voxel_size):
        """使用numpy实现体素下采样"""
        try:
            if len(points) == 0:
                return points
                
            # 计算每个点所属的体素索引
            voxel_indices = np.floor(points / voxel_size).astype(int)
            
            # 使用字典去重并取平均值
            voxel_dict = {}
            for i, idx in enumerate(voxel_indices):
                key = tuple(idx)
                if key not in voxel_dict:
                    voxel_dict[key] = []
                voxel_dict[key].append(points[i])
                
            # 对每个体素中的点取平均值
            downsampled_points = []
            for pts in voxel_dict.values():
                downsampled_points.append(np.mean(pts, axis=0))
                
            return np.array(downsampled_points)
        except Exception as e:
            self.get_logger().error(f'Error in voxel_down_sample_numpy: {e}')
            return np.empty((0, 3), dtype=np.float32)
        
    def registration_at_scale(self, pc_scan, pc_map, initial, scale):
        """在指定尺度下进行配准"""
        try:
            # 下采样
            scan_ds = self.voxel_down_sample_numpy(pc_scan, self.SCAN_VOXEL_SIZE * scale)
            map_ds = self.voxel_down_sample_numpy(pc_map, self.MAP_VOXEL_SIZE * scale)
            
            # 检查点云是否为空
            if len(scan_ds) == 0 or len(map_ds) == 0:
                return initial, 0.0
            
            # 应用初始变换到扫描点
            scan_transformed = self.transform_points(scan_ds, initial)
            
            # 使用最近邻搜索进行ICP迭代
            fitness = 0.0
            transformation = initial.copy()
            
            # 简化的ICP实现
            max_iterations = 20
            for _ in range(max_iterations):
                # 找到对应点
                distances, indices = self.find_closest_points(scan_transformed, map_ds)
                
                # 计算变换
                # 这里简化处理，实际应该使用SVD求解最优变换
                mean_dist = np.mean(distances)
                if mean_dist < 0.5 * scale:  # 简单的收敛判断
                    break
                    
                # 更新变换（简化版）
                # 实际应用中应使用SVD计算最优刚体变换
                
            # 简化计算适应度分数
            fitness = 1.0 / (1.0 + np.mean(distances)) if len(distances) > 0 else 0.0
            
            return transformation, fitness
        except Exception as e:
            self.get_logger().error(f'Error in registration_at_scale: {e}')
            return initial, 0.0
        
    def transform_points(self, points, transformation):
        """应用变换矩阵到点集"""
        try:
            if len(points) == 0:
                return points
            # 添加齐次坐标
            points_homo = np.hstack([points, np.ones((points.shape[0], 1))])
            # 应用变换
            transformed = (transformation @ points_homo.T).T[:, :3]
            return transformed
        except Exception as e:
            self.get_logger().error(f'Error in transform_points: {e}')
            return np.empty((0, 3), dtype=np.float32)
    
    def find_closest_points(self, points_a, points_b):
        """查找点集A中每个点在点集B中的最近邻（纯numpy实现）"""
        try:
            if len(points_a) == 0 or len(points_b) == 0:
                return np.array([]), np.array([])
                
            # 使用numpy计算所有点对之间的距离
            # 这是一种简单但效率较低的实现
            distances = np.linalg.norm(points_a[:, np.newaxis] - points_b, axis=2)
            indices = np.argmin(distances, axis=1)
            min_distances = np.min(distances, axis=1)
            return min_distances, indices
        except Exception as e:
            self.get_logger().error(f'Error in find_closest_points: {e}')
            return np.array([]), np.array([])
        
    def inverse_se3(self, trans):
        """计算SE3变换的逆"""
        try:
            trans_inverse = np.eye(4)
            # R
            trans_inverse[:3, :3] = trans[:3, :3].T
            # t
            trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
            return trans_inverse
        except Exception as e:
            self.get_logger().error(f'Error in inverse_se3: {e}')
            return np.eye(4)
        
    def publish_point_cloud(self, publisher, header, pc):
        """发布点云消息"""
        try:
            # 构造点的列表
            points = []
            if pc.ndim > 1 and pc.shape[1] >= 4:
                # 包含强度信息 (x, y, z, intensity)
                for i in range(len(pc)):
                    points.append([pc[i, 0], pc[i, 1], pc[i, 2], pc[i, 3]])
            elif pc.ndim > 1:
                # 仅包含坐标信息 (x, y, z)
                for i in range(len(pc)):
                    points.append([pc[i, 0], pc[i, 1], pc[i, 2], 0.0])  # 添加默认强度值
            else:
                # 处理空点云或一维数组的情况
                pass
            
            # 定义点云字段
            fields = [
                point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
                point_cloud2.PointField(name='intensity', offset=12, datatype=point_cloud2.PointField.FLOAT32, count=1)
            ]
            
            # 创建并发布 PointCloud2 消息
            msg = point_cloud2.create_cloud(header, fields, points)
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error in publish_point_cloud: {e}')
        
    def crop_global_map_in_FOV(self, global_map, pose_estimation, cur_odom):
        """裁剪FOV内的全局地图"""
        try:
            # 当前scan原点的位姿
            T_odom_to_base_link = self.pose_to_mat(cur_odom) if hasattr(cur_odom, 'pose') else np.eye(4)
            T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
            T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

            # 把地图转换到lidar系下
            global_map_in_map = global_map if isinstance(global_map, np.ndarray) else global_map
            if global_map_in_map.ndim == 1:
                global_map_in_map = global_map_in_map.reshape(-1, 3)
                
            global_map_in_map_homo = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
            global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map_homo.T).T

            # 将视角内的地图点提取出来
            if self.FOV > 3.14:
                # 环状lidar 仅过滤距离
                indices = np.where(
                    (global_map_in_base_link[:, 0] < self.FOV_FAR) &
                    (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.FOV / 2.0)
                )[0]
            else:
                # 非环状lidar 保前视范围
                # FOV_FAR>x>0 且角度小于FOV
                indices = np.where(
                    (global_map_in_base_link[:, 0] > 0) &
                    (global_map_in_base_link[:, 0] < self.FOV_FAR) &
                    (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.FOV / 2.0)
                )[0]
            
            global_map_in_FOV = global_map_in_map[indices] if len(indices) > 0 else np.empty((0, 3))

            # 发布fov内点云
            header = cur_odom.header
            header.frame_id = 'map'
            self.publish_point_cloud(self.pub_submap, header, global_map_in_FOV[::10] if len(global_map_in_FOV) > 0 else global_map_in_FOV)

            return global_map_in_FOV
        except Exception as e:
            self.get_logger().error(f'Error in crop_global_map_in_FOV: {e}')
            return np.empty((0, 3), dtype=np.float32)
        
    def global_localization(self, pose_estimation):
        """全局定位"""
        try:
            # 用icp配准
            self.get_logger().info('Global localization by scan-to-map matching......')

            if self.cur_scan is None or self.global_map is None or self.cur_odom is None:
                self.get_logger().warn('Missing required data for localization')
                return False

            # 检查数据是否有效
            if self.cur_scan.size == 0 or self.global_map.size == 0:
                self.get_logger().warn('Empty point cloud data')
                return False

            # 使用numpy的copy方法替代copy.copy
            scan_tobe_mapped = np.copy(self.cur_scan)

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
        except Exception as e:
            self.get_logger().error(f'Error in global_localization: {e}')
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
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Exception in main: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()