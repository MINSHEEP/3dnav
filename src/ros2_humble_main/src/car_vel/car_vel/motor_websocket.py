#!/usr/bin/env python
# coding=utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import websockets
import json
from .motor_api import MotorAPI

class MotorWebsocketNode(Node):
    def __init__(self):
        super().__init__('motor_websocket_node')
        
        # 初始化电机API
        self.motor = MotorAPI('/dev/ttyUSB0', 115200)
        self.motor.start()
        
        # 创建订阅者，订阅/cmd_vel话题
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # 启动WebSocket服务器
        self.websocket_server = None
        self.start_websocket_server()
        
        self.get_logger().info('Motor WebSocket node initialized')

    def cmd_vel_callback(self, msg):
        """
        处理/cmd_vel话题的消息
        """
        vx = msg.linear.x
        vy = msg.linear.y
        vtheta = msg.angular.z
        
        # 设置电机速度
        self.motor.set_vel(vx*1000, vy*1000, vtheta)
        
        self.get_logger().debug(f'Received cmd_vel: vx={vx}, vy={vy}, vtheta={vtheta}')

    def start_websocket_server(self):
        """
        启动WebSocket服务器
        """
        try:
            # 在新线程中启动WebSocket服务器
            import threading
            self.websocket_thread = threading.Thread(target=self.run_websocket_server)
            self.websocket_thread.daemon = True
            self.websocket_thread.start()
        except Exception as e:
            self.get_logger().error(f'Failed to start WebSocket server: {e}')

    async def handle_websocket_client(self, websocket):
        """
        处理WebSocket客户端连接
        """
        async for message in websocket:
            try:
                data = json.loads(message)
                vx = data.get("vx", 0)
                vy = data.get("vy", 0)
                vtheta = data.get("vtheta", 0)
                
                # 设置电机速度
                self.motor.set_vel(vx, vy, vtheta)
                
                # 发送确认消息
                response = f"Set velocity: vx={vx}, vy={vy}, vtheta={vtheta}"
                # self.get_logger().info(response)
                await websocket.send(response)
                
                
                
            except json.JSONDecodeError:
                await websocket.send("Invalid JSON format")
            except Exception as e:
                await websocket.send(f"Error: {str(e)}")

    def run_websocket_server(self):
        """
        运行WebSocket服务器
        """
        async def server_routine():
            self.websocket_server = await websockets.serve(
                self.handle_websocket_client, 
                "0.0.0.0", 
                8765
            )
            await self.websocket_server.wait_closed()
            
        asyncio.run(server_routine())

    def destroy_node(self):
        """
        清理资源
        """
        # 停止电机
        if self.motor:
            self.motor.stop()
            
        # 关闭WebSocket服务器
        if self.websocket_server:
            self.websocket_server.close()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = MotorWebsocketNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()