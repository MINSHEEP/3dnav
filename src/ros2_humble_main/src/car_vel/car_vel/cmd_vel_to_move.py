#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.b2.sport.sport_client import SportClient

class CmdVelToDDS:
    def __init__(self):
        # 初始化节点
        rospy.init_node('cmd_vel_to_move', anonymous=True)
        self.last_cmd = Twist()         # 上一次滤波后的值

        # 滤波参数（越小越平滑）
        self.alpha = 0.2
        self.max_delta_vx = 0.2           # 最大线速度跳变幅度 m/s
        self.max_delta_vyaw = 0.3         # 最大角速度跳变幅度 rad/s

        # 初始化订阅器
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.loginfo("监听中: /cmd_vel")

        ChannelFactoryInitialize(0, "enp2s0")

        # 初始化宇树控制SDK
        self.sport_client = SportClient() 
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()

        # 设置为全地形模式
        self.sport_client.SwitchGait(1)  


    # 滤波算法  
    def lowpass_filter(self, current, previous, zero_threshold=1e-3):
        """
        一阶低通滤波：如果 current 接近 0，直接返回 0，避免拖尾。
        1e-3 = 0.001
        """
        if abs(current) < zero_threshold:
            return 0.0
        return self.alpha * current + (1 - self.alpha) * previous

    # 加速度限幅
    def clamp_delta(self, new, old, max_delta):
        delta = new - old
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        return old + delta
    

    # 回调函数
    def cmd_vel_callback(self, msg):
        cmd_vel = {
            'x': msg.linear.x,
            'y': msg.linear.y,
            'z': msg.angular.z,
        }
        # rospy.loginfo("cmd_vel规划的速度: {}".format(cmd_vel))

        # 原始速度指令
        vx_raw = msg.linear.x
        vyaw_raw = msg.angular.z

        """
        将cmd中滤波后的Twist消息转换为底层机器狗DDS指令。
        """
        vx_filtered = self.lowpass_filter(vx_raw, self.last_cmd.linear.x)
        vyaw_filtered = self.lowpass_filter(vyaw_raw, self.last_cmd.angular.z)

        # 加速度限幅处理（保险）
        vx_safe = self.clamp_delta(vx_filtered, self.last_cmd.linear.x, self.max_delta_vx)
        vyaw_safe = self.clamp_delta(vyaw_filtered, self.last_cmd.angular.z, self.max_delta_vyaw)

        # 更新保存的指令
        self.last_cmd.linear.x = vx_safe
        self.last_cmd.angular.z = vyaw_safe

        move_cmd = {
            'vx': vx_safe,
            'vyaw': vyaw_safe,
        }
        # rospy.loginfo("发送DDS move命令的msg: {}".format(move_cmd))
        # 发送速度指令
        self.sport_client.Move(vx_safe, 0, vyaw_safe)


if __name__ == '__main__':
    try:
        CmdVelToDDS()
        rospy.loginfo("节点初始化成功，开始运行")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
