#!/usr/bin/env python3
"""
航点管理服务节点
提供基于SQLite存储的航点管理功能
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.action import ActionClient
from lzp_interface.srv import (
    CreateWaypoint,
    ReadWaypoint,
    ListWaypoints,
    UpdateWaypoint,
    DeleteWaypoint,
    NavigateToWaypoint,
    MultiNavigate,
    PauseNavigation,
    ResumeNavigation
)
from lzp_interface.msg import Waypoint as WaypointMsg
from lzp_interface.msg import NavigationStatus
from .database import WaypointDatabase

# 导入导航相关消息类型
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Twist
import threading
from typing import Optional
from nav2_msgs.action import FollowWaypoints
from lzp_interface.msg import CurrentMap


class WaypointServiceNode(Node):
    """航点管理服务节点"""
    
    def __init__(self):
        super().__init__('waypoint_service')
        
        # 初始化数据库
        self.database = WaypointDatabase()

        # 添加暂停相关属性
        self.is_paused = False
        
        # 创建回调组以支持并发处理
        callback_group = ReentrantCallbackGroup()
        
        # 创建服务
        self.create_service(
            CreateWaypoint, 'create_waypoint', self.handle_create_waypoint,
            callback_group=callback_group
        )
        
        self.create_service(
            ReadWaypoint, 'read_waypoint', self.handle_read_waypoint,
            callback_group=callback_group
        )
        
        self.create_service(
            ListWaypoints, 'list_waypoints', self.handle_list_waypoints,
            callback_group=callback_group
        )
        
        self.create_service(
            UpdateWaypoint, 'update_waypoint', self.handle_update_waypoint,
            callback_group=callback_group
        )
        
        self.create_service(
            DeleteWaypoint, 'delete_waypoint', self.handle_delete_waypoint,
            callback_group=callback_group
        )

        # 单点导航服务
        self.create_service(
            NavigateToWaypoint, 'navigate_to_waypoint', self.handle_navigate_to_waypoint,
            callback_group=callback_group
        )
        # 多点导航服务
        self.create_service(
            MultiNavigate, 'multi_navigate', self.handle_multi_navigate,
            callback_group=callback_group
        )

        # 添加暂停服务
        self.create_service(
            PauseNavigation, 'pause_navigation', self.handle_pause_navigation,
            callback_group=callback_group
        )
        

        # 创建导航动作客户端
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 发布零速度命令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 存储当前导航信息
        self.current_goal_handle = None
        self.current_waypoint = None
        self.navigation_lock = threading.Lock()
        self.is_multi_nav_active = False
        self.multi_nav_waypoints = []
        self.current_multi_nav_index = -1
        self._multi_nav_thread = None

        # 创建导航状态发布者
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.navigation_status_pub = self.create_publisher(
            NavigationStatus, 'navigation_status', qos_profile
        )
        
        self.get_logger().info('Waypoint service started with SQLite storage')
        #订阅获取当前地图
        self.current_map_name = ""
        self.create_subscription(
            CurrentMap,
            'current_map_info',
            self.current_map_name_callback,
            qos_profile
        )
    

    def current_map_name_callback(self, msg):
        if msg.success and msg.map_name:
            self.current_map_name = msg.map_name

    def handle_pause_navigation(self, request, response):
        """处理暂停导航请求"""
        try:
            if self.is_multi_nav_active == False and self.current_waypoint is None:
                response.success = False
                response.message = "No active navigation to pause"
                self.get_logger().warn("No active navigation to pause")
                return response
            
            # if self.is_paused:
            #     response.success = False
            #     response.message = "Navigation is already paused"
            #     self.get_logger().warn("Navigation is already paused")
            #     return response
            

            # 取消当前导航 - 正确的方式
            try:
                # cancel_goal() 是同步调用，直接返回结果
                cancel_result = self.current_goal_handle.cancel_goal()
                
                # 检查取消是否成功
                if cancel_result is not None:
                    self.is_paused = True
                    with self.navigation_lock:
                        self.is_multi_nav_active = False
                    
                    response.success = True
                    response.message = "Navigation paused successfully"
                    self.get_logger().info("Navigation paused successfully")
                else:
                    response.success = False
                    response.message = "Failed to pause navigation"
                    self.get_logger().error("Failed to pause navigation")
                    
            except Exception as cancel_error:
                response.success = False
                response.message = f"Error cancelling navigation: {str(cancel_error)}"
                self.get_logger().error(f"Error cancelling navigation: {str(cancel_error)}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error pausing navigation: {str(e)}"
            self.get_logger().error(f"Error pausing navigation: {str(e)}")
        
        return response
    

    def send_zero_velocity(self):
        """发送零速度命令"""
        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.angular.z = 0.0
        self.cmd_vel_pub.publish(zero_twist)


    def waypoint_to_msg(self, waypoint_dict: dict) -> WaypointMsg:
        """将字典转换为航点消息"""
        msg = WaypointMsg()
        msg.id = int(waypoint_dict.get('id', 0))
        msg.name = str(waypoint_dict.get('name', ''))
        msg.x = float(waypoint_dict.get('x', 0.0))
        msg.y = float(waypoint_dict.get('y', 0.0))
        msg.theta = float(waypoint_dict.get('theta', 0.0))
        return msg
    
    def msg_to_dict(self, msg: WaypointMsg) -> dict:
        """将航点消息转换为字典"""
        return {
            'id': int(msg.id),
            'name': str(msg.name),
            'x': float(msg.x),
            'y': float(msg.y),
            'theta': float(msg.theta)
        }
    
    def handle_create_waypoint(self, request, response):

        """处理创建航点请求"""
        try:
            waypoint = self.msg_to_dict(request.waypoint)
            
            # 创建航点
            waypoint_id = self.database.create_waypoint(
                waypoint['name'],
                waypoint['x'],
                waypoint['y'],
                waypoint['theta'],
                self.current_map_name
            )
            
            response.id = waypoint_id
            response.success = True
            response.message = f"Waypoint '{waypoint['name']}' created with ID {waypoint_id}"
            
            self.get_logger().info(f"Created waypoint: {waypoint['name']} (ID: {waypoint_id})")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to create waypoint: {str(e)}"
            self.get_logger().error(f"Error creating waypoint: {str(e)}")
        
        return response
    
    def handle_read_waypoint(self, request, response):
        """处理读取航点请求"""
        try:
            waypoint_id = request.id
            
            # 获取航点
            waypoint = self.database.get_waypoint(waypoint_id)
            
            if waypoint:
                response.waypoint = self.waypoint_to_msg(waypoint)
                response.found = True
                response.message = f"Waypoint found with ID {waypoint_id}"
                self.get_logger().info(f"Retrieved waypoint ID: {waypoint_id}")
            else:
                response.found = False
                response.message = f"Waypoint with ID {waypoint_id} not found"
                self.get_logger().warn(f"Waypoint ID {waypoint_id} not found")
                
        except Exception as e:
            response.found = False
            response.message = f"Error reading waypoint: {str(e)}"
            self.get_logger().error(f"Error reading waypoint: {str(e)}")
        
        return response
    
    def handle_list_waypoints(self, request, response):
        """处理列出所有航点请求"""
        try:
            # 获取所有航点
            waypoints = self.database.list_waypoints(self.current_map_name)
            
            # 转换为消息列表
            response.waypoints = [self.waypoint_to_msg(wp) for wp in waypoints]
            
            self.get_logger().info(f"Listed {len(waypoints)} waypoints")
            
        except Exception as e:
            response.waypoints = []
            self.get_logger().error(f"Error listing waypoints: {str(e)}")
        
        return response
    
    def handle_update_waypoint(self, request, response):
        """处理更新航点请求"""
        try:
            waypoint = self.msg_to_dict(request.waypoint)
            waypoint_id = waypoint['id']
            
            # 更新航点
            success = self.database.update_waypoint(
                waypoint_id,
                name=waypoint['name'],
                x=waypoint['x'],
                y=waypoint['y'],
                theta=waypoint['theta']
            )
            
            if success:
                response.success = True
                response.message = f"Waypoint with ID {waypoint_id} updated"
                self.get_logger().info(f"Updated waypoint ID: {waypoint_id}")
            else:
                response.success = False
                response.message = f"Waypoint with ID {waypoint_id} not found"
                self.get_logger().warn(f"Failed to update waypoint ID {waypoint_id}: not found")
                
        except Exception as e:
            response.success = False
            response.message = f"Error updating waypoint: {str(e)}"
            self.get_logger().error(f"Error updating waypoint: {str(e)}")
        
        return response
    
    def handle_delete_waypoint(self, request, response):
        """处理删除航点请求"""
        try:
            waypoint_id = request.id
            
            # 检查航点是否存在
            waypoint = self.database.get_waypoint(waypoint_id)
            
            if not waypoint:
                response.success = False
                response.message = f"Waypoint with ID {waypoint_id} not found"
                self.get_logger().warn(f"Failed to delete waypoint ID {waypoint_id}: not found")
                return response
            
            # 删除航点
            success = self.database.delete_waypoint(waypoint_id)
            
            if success:
                response.success = True
                response.message = f"Waypoint '{waypoint['name']}' (ID: {waypoint_id}) deleted"
                self.get_logger().info(f"Deleted waypoint: {waypoint['name']} (ID: {waypoint_id})")
            else:
                response.success = False
                response.message = f"Failed to delete waypoint with ID {waypoint_id}"
                self.get_logger().error(f"Failed to delete waypoint ID {waypoint_id}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error deleting waypoint: {str(e)}"
            self.get_logger().error(f"Error deleting waypoint: {str(e)}")
        
        return response

    def handle_navigate_to_waypoint(self, request, response):
        """处理导航到航点请求"""
        try:
            waypoint_id = request.waypoint_id
            
            # 获取航点
            waypoint = self.database.get_waypoint(waypoint_id)
            
            if not waypoint:
                response.success = False
                response.message = f"Waypoint with ID {waypoint_id} not found"
                self.get_logger().warn(f"Navigation failed: Waypoint ID {waypoint_id} not found")
                return response
            
            # 发送导航目标
            success = self._send_navigation_goal(waypoint)
            
            if success:
                response.success = True
                response.message = f"Navigation to waypoint '{waypoint['name']}' (ID: {waypoint_id}) started"
                self.get_logger().info(f"Started navigation to waypoint: {waypoint['name']} (ID: {waypoint_id})")
            else:
                response.success = False
                response.message = "Failed to send navigation goal"
                
        except Exception as e:
            response.success = False
            response.message = f"Failed to start navigation: {str(e)}"
            self.get_logger().error(f"Error starting navigation: {str(e)}")
        
        return response

    def _send_navigation_goal(self, waypoint: dict) -> bool:
        """发送导航目标到Nav2"""
        try:
            # 等待动作服务器
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Navigation action server not available')
                return False
            
            # 创建目标
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(waypoint['x'])
            goal_msg.pose.pose.position.y = float(waypoint['y'])
            goal_msg.pose.pose.position.z = 0.0
            
            # 设置四元数方向
            from tf_transformations import quaternion_from_euler
            quat = quaternion_from_euler(0, 0, float(waypoint['theta']))
            goal_msg.pose.pose.orientation.x = float(quat[0])
            goal_msg.pose.pose.orientation.y = float(quat[1])
            goal_msg.pose.pose.orientation.z = float(quat[2])
            goal_msg.pose.pose.orientation.w = float(quat[3])
            
            # 发送目标并注册反馈回调
            future = self.nav_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self._navigation_feedback_callback
            )
            
            # 注册结果回调
            future.add_done_callback(lambda f: self._navigation_result_callback(f, waypoint))
            
            # 保存当前导航信息
            with self.navigation_lock:
                self.current_waypoint = waypoint
                
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending navigation goal: {str(e)}")
            return False

    def _navigation_feedback_callback(self, feedback_msg):
        """处理导航反馈信息"""
        try:
            feedback = feedback_msg.feedback
            waypoint = self.current_waypoint
            
            if waypoint is None:
                return
            
            # 创建并发布导航状态消息
            msg = NavigationStatus()
            msg.waypoint_id = int(waypoint['id'])
            msg.waypoint_name = str(waypoint['name'])
            msg.status = "navigating"
            msg.distance_to_goal = float(feedback.distance_remaining)
            msg.current_x = float(feedback.current_pose.pose.position.x)
            msg.current_y = float(feedback.current_pose.pose.position.y)
            msg.target_x = float(waypoint['x'])
            msg.target_y = float(waypoint['y'])
            msg.target_theta = float(waypoint['theta'])
            
            self.navigation_status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing navigation feedback: {str(e)}")

    def _navigation_result_callback(self, future, waypoint):
        """处理导航结果"""
        try:
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().warn('Navigation goal rejected')
                self._publish_final_status(waypoint, "failed")
                return
            
            # 获取结果
            # Use the public ROS 2 action API.  The private _get_result method
            # is not stable across rclpy releases and can break waypoint runs.
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: self._handle_navigation_result(f, waypoint))
            
        except Exception as e:
            self.get_logger().error(f"Error handling navigation result: {str(e)}")
            self._publish_final_status(waypoint, "failed")

    def _handle_navigation_result(self, future, waypoint):
        """处理导航最终结果"""
        try:
            result = future.result()
            # 根据结果判断导航是否成功
            if result.status == 4:  # SUCCEEDED
                self._publish_final_status(waypoint, "arrived")
                self.get_logger().info(f"Navigation to waypoint '{waypoint['name']}' completed successfully")
            else:
                self._publish_final_status(waypoint, "failed")
                self.get_logger().warn(f"Navigation to waypoint '{waypoint['name']}' failed with status: {result.status}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing navigation result: {str(e)}")
            self._publish_final_status(waypoint, "failed")
        finally:
            # 清除当前导航信息
            with self.navigation_lock:
                self.current_waypoint = None

    def _publish_final_status(self, waypoint: dict, status: str):
        """发布最终导航状态"""
        try:
            msg = NavigationStatus()
            msg.waypoint_id = int(waypoint['id'])
            msg.waypoint_name = str(waypoint['name'])
            msg.status = status
            msg.distance_to_goal = 0.0 if status == "arrived" else -1.0
            msg.current_x = 0.0
            msg.current_y = 0.0
            msg.target_x = float(waypoint['x'])
            msg.target_y = float(waypoint['y'])
            msg.target_theta = float(waypoint['theta'])
            
            self.navigation_status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing final status: {str(e)}")

    # 处理多点导航请求 传入id集合
    def handle_multi_navigate(self, request, response):
        self.get_logger().error("start multi-navigation1")
        """处理多点导航请求"""
        try:
            # 检查是否已经在进行多点导航
            with self.navigation_lock:
                if self.is_multi_nav_active:
                    response.success = False
                    response.message = "Multi-navigation is already in progress"
                    self.get_logger().warn("Multi-navigation request rejected: already in progress")
                    return response
                    
            waypoint_ids = request.waypoint_ids
            
            if not waypoint_ids:
                response.success = False
                response.message = "No waypoints provided"
                self.get_logger().warn("Multi-navigation failed: No waypoints provided")
                return response
            
            # 验证所有航点都存在
            waypoints = []
            for waypoint_id in waypoint_ids:
                waypoint = self.database.get_waypoint(waypoint_id)
                if not waypoint:
                    response.success = False
                    response.message = f"Waypoint with ID {waypoint_id} not found"
                    self.get_logger().warn(f"Multi-navigation failed: Waypoint ID {waypoint_id} not found")
                    return response
                waypoints.append(waypoint)
            
            # 启动多点导航任务
            success = self._start_multi_navigation(waypoints)
            
            if success:
                response.success = True
                waypoint_names = [wp['name'] for wp in waypoints]
                response.message = f"Multi-navigation started for waypoints: {', '.join(waypoint_names)}"
                self.get_logger().info(f"Started multi-navigation for {len(waypoints)} waypoints")
            else:
                response.success = False
                response.message = "Failed to start multi-navigation"
                
        except Exception as e:
            response.success = False
            response.message = f"Failed to start multi-navigation: {str(e)}"
            self.get_logger().error(f"Error starting multi-navigation: {str(e)}")
        
        return response

    # 开始多点导航
    def _start_multi_navigation(self, waypoints):
        """启动多点导航任务 (基于 asyncio)"""
        try:
            # 设置多点导航状态
            with self.navigation_lock:
                self.multi_nav_waypoints = waypoints
                self.is_multi_nav_active = True  # 标记多点导航正在进行
            
            # 创建多点导航任务对象
            multi_nav_task = MultiNavigationTask(
                self,
                waypoints,
                self.nav_to_pose_client,
            )
            
            # 使用 asyncio 创建任务
            import asyncio
            # 在新线程中运行事件循环，避免阻塞 ROS2 主循环
            loop = asyncio.new_event_loop()
            navigation_thread = threading.Thread(
                target=self._run_asyncio_loop,
                args=(loop, multi_nav_task.execute_async()),
                daemon=True
            )
            self._multi_nav_thread = navigation_thread
            navigation_thread.start()
            return True
        except Exception as e:
            # 出错时重置状态
            with self.navigation_lock:
                self.is_multi_nav_active = False
                self.multi_nav_waypoints = []
            self.get_logger().error(f"Error starting multi-navigation task: {str(e)}")
            return False
    def _run_asyncio_loop(self, loop, coro):
        """在新线程中运行 asyncio 事件循环"""
        import asyncio
        asyncio.set_event_loop(loop)
        loop.run_until_complete(coro)

    def update_multi_nav_status(self, index, status):
        """更新多点导航状态"""
        try:
            # 确保索引有效
            if 0 <= index < len(self.multi_nav_waypoints):
                waypoint = self.multi_nav_waypoints[index]
                
                # 创建并发布导航状态消息
                msg = NavigationStatus()
                msg.waypoint_id = int(waypoint['id'])
                msg.waypoint_name = str(waypoint['name'])
                msg.status = str(status)
                msg.distance_to_goal = -1.0  # 默认值
                msg.current_x = 0.0
                msg.current_y = 0.0
                msg.target_x = float(waypoint['x'])
                msg.target_y = float(waypoint['y'])
                msg.target_theta = float(waypoint['theta'])
                
                self.navigation_status_pub.publish(msg)
                self.get_logger().info(f"Published navigation status: {status} for waypoint {waypoint['name']} (ID: {waypoint['id']})")
            else:
                self.get_logger().warn(f"Invalid waypoint index: {index}")
        except Exception as e:
            self.get_logger().error(f"Error updating multi-nav status: {str(e)}")

    def reset_multi_nav_state(self):
        """重置多点导航状态"""
        with self.navigation_lock:
            self.is_multi_nav_active = False
            self.multi_nav_waypoints = []
            self.current_multi_nav_index = -1
            self._multi_nav_thread = None  
        self.get_logger().info("Multi-navigation state has been reset")

class MultiNavigationTask:
    """多点导航任务类 - 使用循环单点导航实现 (基于 asyncio)"""
    
    def __init__(self, node, waypoints, nav_client):
        self.node = node
        self.waypoints = waypoints
        self.nav_client = nav_client
        self.cancelled = False
        self.current_waypoint_index = -1
        self.last_feedback_time = 0.0  # 添加时间戳记录上次反馈时间
        self.feedback_interval = 0.5   # 设置反馈间隔为0.5秒
        
    async def execute_async(self):
        """异步执行多点导航任务 - 使用循环单点导航"""
        try:
            self.node.get_logger().info(f"Starting multi-navigation with {len(self.waypoints)} waypoints using sequential single-point navigation")
            
            # 触发开始导航事件
            self._on_multi_nav_start()
            
            # 依次导航到每个航点
            for i, waypoint in enumerate(self.waypoints):
                if self.cancelled or self.node.is_multi_nav_active == False:
                    self.node.get_logger().info("Multi-navigation cancelled")
                    self._on_multi_nav_cancelled()
                    break  # 使用break而不是return以确保finally执行
                
                self.current_waypoint_index = i
                self.node.get_logger().info(
                    f"Navigating to waypoint {i+1}/{len(self.waypoints)}: "
                    f"{waypoint['name']} (ID: {waypoint['id']})"
                )
                
                # 触发接近航点事件
                self._on_waypoint_approaching(waypoint)
                self.node.update_multi_nav_status(i, "approaching")
                
                # 导航到当前航点
                success = await self._navigate_to_single_waypoint_async(waypoint, i)
                
                if success:
                    self.node.get_logger().info(
                        f"Arrived at waypoint: {waypoint['name']} (ID: {waypoint['id']})"
                    )
                    # 触发到达航点事件
                    self._on_waypoint_arrived(waypoint)
                    self.node.update_multi_nav_status(i, "arrived")
                else:
                    self.node.get_logger().error(
                        f"Failed to reach waypoint: {waypoint['name']} (ID: {waypoint['id']})"
                    )
                    # 触发导航失败事件
                    self._on_waypoint_failed(waypoint)
                    self.node.update_multi_nav_status(i, "failed")
                    self._on_multi_nav_failed()
                    break  # 导航失败则停止整个任务
            
            # 只有在没有被取消且没有出错的情况下才标记为完成
            if not self.cancelled and not any(self.node.multi_nav_waypoints[i] is None for i in range(len(self.node.multi_nav_waypoints))):
                self.node.get_logger().info("Multi-navigation completed successfully")
                self._on_multi_nav_completed()
                # 发送最终完成状态
                if self.waypoints:
                    self.node.update_multi_nav_status(len(self.waypoints) - 1, "completed")
                
        except Exception as e:
            self.node.get_logger().error(f"Error during multi-navigation: {str(e)}")
            self._on_multi_nav_failed()
            # 发送失败状态
            if self.waypoints and self.current_waypoint_index >= 0:
                self.node.update_multi_nav_status(self.current_waypoint_index, "failed")
        finally:
            # 确保总是重置多点导航状态
            self.node.reset_multi_nav_state()
    
    async def _navigate_to_single_waypoint_async(self, waypoint, index):
        """异步导航到单个航点并实时更新状态"""
        try:
            # 等待动作服务器
            import asyncio
            loop = asyncio.get_event_loop()
            
            # 将 wait_for_server 包装成协程
            def wait_server():
                return self.nav_client.wait_for_server(timeout_sec=5.0)
            
            server_available = await loop.run_in_executor(None, wait_server)
            if not server_available:
                self.node.get_logger().error('Navigation action server not available')
                return False
            
            # 创建目标
            from nav2_msgs.action import NavigateToPose
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(waypoint['x'])
            goal_msg.pose.pose.position.y = float(waypoint['y'])
            goal_msg.pose.pose.position.z = 0.0
            
            # 设置四元数方向
            from tf_transformations import quaternion_from_euler
            quat = quaternion_from_euler(0, 0, float(waypoint['theta']))
            goal_msg.pose.pose.orientation.x = float(quat[0])
            goal_msg.pose.pose.orientation.y = float(quat[1])
            goal_msg.pose.pose.orientation.z = float(quat[2])
            goal_msg.pose.pose.orientation.w = float(quat[3])
            
            # 发送目标并注册反馈回调
            future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback: self._single_nav_feedback_callback(feedback, waypoint, index)
            )
            
            # 等待目标被接受
            while not future.done():
                await asyncio.sleep(0.1)  # 非阻塞等待
                # 可以添加超时检查逻辑
            
            if not future.done():
                self.node.get_logger().error('Timeout waiting for navigation goal response')
                return False
                
            goal_handle = future.result()
            self.node.current_goal_handle = goal_handle
            
            if not goal_handle.accepted:
                self.node.get_logger().warn('Navigation goal rejected')
                return False
            
            # 等待结果
            result_future = goal_handle.get_result_async()
            while not result_future.done():
                await asyncio.sleep(0.1)  # 非阻塞等待
                # 可以添加超时检查逻辑
            
            # 获取结果
            result = result_future.result()
            
            # 根据结果判断导航是否成功
            return result.status == 4  # SUCCEEDED
            
        except Exception as e:
            self.node.get_logger().error(f"Error navigating to single waypoint: {str(e)}")
            return False
    
    def _single_nav_feedback_callback(self, feedback_msg, waypoint, index):
        """处理单点导航的反馈信息 (保持不变)"""
        try:
            # 检查是否应该发布反馈（控制频率）
            current_time = self.node.get_clock().now().nanoseconds / 1e9  # 转换为秒
            if current_time - self.last_feedback_time < self.feedback_interval:
                return  # 未达到发布时间间隔，跳过
            
            self.last_feedback_time = current_time

            feedback = feedback_msg.feedback
            
            # 创建并发布导航状态消息
            msg = NavigationStatus()
            msg.waypoint_id = int(waypoint['id'])
            msg.waypoint_name = str(waypoint['name'])
            msg.status = "navigating"
            
            # 设置距离信息
            if hasattr(feedback, 'distance_remaining'):
                msg.distance_to_goal = float(feedback.distance_remaining)
            else:
                msg.distance_to_goal = -1.0
                
            # 设置当前位置信息
            if hasattr(feedback, 'current_pose'):
                msg.current_x = float(feedback.current_pose.pose.position.x)
                msg.current_y = float(feedback.current_pose.pose.position.y)
            else:
                msg.current_x = 0.0
                msg.current_y = 0.0
                
            # 设置目标位置信息
            msg.target_x = float(waypoint['x'])
            msg.target_y = float(waypoint['y'])
            msg.target_theta = float(waypoint['theta'])
            
            self.node.navigation_status_pub.publish(msg)
            
        except Exception as e:
            self.node.get_logger().debug(f"Error processing single nav feedback: {str(e)}")
    
    # 以下是可以被重写的扩展接口方法 (保持不变)
    def _on_multi_nav_start(self):
        """多点导航开始事件 - 可用于扩展功能如语音播报"""
        self.node.get_logger().info("Multi-navigation task started")
        if self.waypoints:
            self.node.update_multi_nav_status(0, "started")
        # TODO: 添加语音播报"开始多点导航"
    
    def _on_waypoint_approaching(self, waypoint):
        """接近航点事件 - 可用于扩展功能如语音播报"""
        self.node.get_logger().info(f"Approaching waypoint: {waypoint['name']}")
        # TODO: 添加语音播报"正在前往 {waypoint['name']}"
    
    def _on_waypoint_arrived(self, waypoint):
        """到达航点事件 - 可用于扩展功能如语音播报"""
        self.node.get_logger().info(f"Arrived at waypoint: {waypoint['name']}")
        # TODO: 添加语音播报"已到达 {waypoint['name']}"
    
    def _on_waypoint_failed(self, waypoint):
        """航点导航失败事件 - 可用于扩展功能如语音播报"""
        self.node.get_logger().info(f"Failed to reach waypoint: {waypoint['name']}")
        # TODO: 添加语音播报"无法到达 {waypoint['name']}，导航中断"
    
    def _on_multi_nav_completed(self):
        """多点导航完成事件 - 可用于扩展功能如语音播报"""
        self.node.get_logger().info("Multi-navigation completed")
        # TODO: 添加语音播报"多点导航已完成"
    
    def _on_multi_nav_failed(self):
        """多点导航失败事件 - 可用于扩展功能如语音播报"""
        self.node.get_logger().info("Multi-navigation failed")
        # TODO: 添加语音播报"多点导航失败"
    
    def _on_multi_nav_cancelled(self):
        """多点导航取消事件 - 可用于扩展功能如语音播报"""
        self.node.get_logger().info("Multi-navigation cancelled")
        # TODO: 添加语音播报"多点导航已取消"


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 创建节点
    node = WaypointServiceNode()
    
    # 使用多线程执行器以支持并发服务调用
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # 运行节点
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
