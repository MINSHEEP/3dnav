#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
from lzp_interface.srv import ArmControl
import threading
from sensor_msgs.msg import JointState

class ArmControlService(Node):
    def __init__(self):
        super().__init__('arm_control_service')
        
        # 创建回调组以支持多线程
        self.callback_group = ReentrantCallbackGroup()

        # 存储最新的关节状态
        self.latest_joint_states = {}
        
        # 创建服务
        self.srv = self.create_service(
            ArmControl, 
            'arm_control', 
            self.arm_control_callback,
            callback_group=self.callback_group
        )
        
        # 创建Action客户端用于左右臂
        self.left_arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        self.right_arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/right_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        

        # 订阅关节状态
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',  # 或其他相应的主题名
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )


        # 定义关节名称
        self.left_joint_names = [
            'openarm_left_joint1',
            'openarm_left_joint2', 
            'openarm_left_joint3',
            'openarm_left_joint4',
            'openarm_left_joint5',
            'openarm_left_joint6',
            'openarm_left_joint7'
        ]
        
        self.right_joint_names = [
            'openarm_right_joint1',
            'openarm_right_joint2', 
            'openarm_right_joint3',
            'openarm_right_joint4',
            'openarm_right_joint5',
            'openarm_right_joint6',
            'openarm_right_joint7'
        ]
        
        self.get_logger().info('Arm Control Service initialized')



    def joint_state_callback(self, msg):
        """更新最新的关节状态"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.latest_joint_states[name] = msg.position[i]
    
    def get_current_positions(self, joint_names):
        """获取指定关节的当前实际位置"""
        positions = []
        for name in joint_names:
            if name in self.latest_joint_states:
                positions.append(self.latest_joint_states[name])
            else:
                positions.append(0.0)  # 默认值
                self.get_logger().warn(f'No state found for joint {name}, using 0.0')
        return positions
    def cubic_interpolation(self, start_pos, end_pos, duration, num_points=50):
        """
        使用三次多项式插值生成平滑轨迹
        """
        times = np.linspace(0, duration, num_points)
        trajectory_points = []
        
        for t in times:
            # 归一化时间
            s = t / duration
            
            # 三次多项式系数 (起点0,终点1,速度边界为0)
            # p(s) = 3s² - 2s³
            coeff_pos = 3 * s**2 - 2 * s**3
            # v(s) = 6s - 6s²
            coeff_vel = (6 * s - 6 * s**2) / duration
            
            point = JointTrajectoryPoint()
            point.positions = [
                start + (end - start) * coeff_pos 
                for start, end in zip(start_pos, end_pos)
            ]
            point.velocities = [
                (end - start) * coeff_vel 
                for start, end in zip(start_pos, end_pos)
            ]
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            trajectory_points.append(point)
            
        return trajectory_points

    def send_trajectory_async(self, arm_side, target_positions, duration, completion_callback):
        """
        同步发送轨迹到指定机械臂（在独立线程中运行）
        """
        try:
            # 选择对应的手臂客户端和关节名称
            if arm_side == 'left':
                client = self.left_arm_client
                joint_names = self.left_joint_names
            elif arm_side == 'right':
                client = self.right_arm_client
                joint_names = self.right_joint_names
            else:
                self.get_logger().error(f'Unknown arm side: {arm_side}')
                completion_callback(False, f'Unknown arm side: {arm_side}')
                return
            
            # 等待action服务器
            if not client.wait_for_server(timeout_sec=5.0):
                error_msg = f'{arm_side} arm action server not available'
                self.get_logger().error(error_msg)
                completion_callback(False, error_msg)
                return

            # 创建目标
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = joint_names
            
            # 获取当前关节位置作为起始点
            current_positions = self.get_current_positions(joint_names)
            
            # 生成平滑轨迹点
            trajectory_points = self.cubic_interpolation(
                current_positions, 
                target_positions, 
                duration
            )
            
            goal_msg.trajectory.points = trajectory_points
            #打印轨迹点信息
            self.get_logger().warn(f'Trajectory points for {arm_side} arm: {trajectory_points}')
            
            # 发送目标并等待接受
            self.get_logger().info(f'Sending smooth trajectory to {arm_side} arm: {target_positions}')
            result = client.send_goal(goal_msg)
            

            # 处理结果
            if hasattr(result, 'result') and hasattr(result.result, 'error_code'):
                if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                    self.get_logger().info(f'{arm_side} arm trajectory executed successfully')
                    completion_callback(True, 'Success')
                else:
                    error_msg = f'{arm_side} arm trajectory failed with error code: {result.result.error_code}'
                    self.get_logger().error(error_msg)
                    completion_callback(False, error_msg)
            else:
                self.get_logger().info(f'{arm_side} arm trajectory executed')
                completion_callback(True, 'Success')
                
        except Exception as e:
            error_msg = f'Exception during trajectory execution: {str(e)}'
            self.get_logger().error(error_msg)
            completion_callback(False, error_msg)



    def arm_control_callback(self, request, response):
        """
        处理手臂控制服务请求 - 异步非阻塞版本
        """
        self.get_logger().info(f'Received arm control request: side={request.side}, duration={request.duration}')
        
        # 验证手臂侧别
        if request.side not in ['left', 'right']:
            response.success = False
            response.message = f'Invalid arm side: {request.side}. Must be "left" or "right"'
            return response
        
        # 验证关节数量
        expected_joints = 7
        if len(request.positions) != expected_joints:
            response.success = False
            response.message = f'Invalid number of joints: {len(request.positions)}. Expected {expected_joints}'
            return response
        
        # 使用回调方式处理，立即返回响应
        def completion_callback(success, message):
            # 这个回调在另一个线程中执行，不能直接修改response
            self.get_logger().info(f'Arm control completed: success={success}, message={message}')
        
        # 在新线程中执行轨迹发送，不等待结果
        execution_thread = threading.Thread(
            target=self.send_trajectory_async,
            args=(request.side, list(request.positions), request.duration, completion_callback)
        )
        execution_thread.daemon = True
        execution_thread.start()
        
        # 立即返回响应
        response.success = True
        response.message = 'Command sent to arm controller'
        self.get_logger().info('Arm control command sent asynchronously')
        return response

def main(args=None):
    rclpy.init(args=args)
    
    # 创建服务节点
    arm_service = ArmControlService()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(arm_service)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        arm_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()