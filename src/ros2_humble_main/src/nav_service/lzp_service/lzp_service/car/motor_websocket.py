#!/usr/bin/env python
# coding=utf-8

import rclpy
from rclpy.node import Node
import asyncio
import websockets
import json
import threading
from motor_api import MotorAPI

class MotorWebsocketNode(Node):
    def __init__(self):
        super().__init__('motor_websocket')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('websocket_port', 8765)
        self.declare_parameter('websocket_host', '0.0.0.0')
        
        # 获取参数
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.websocket_port = self.get_parameter('websocket_port').get_parameter_value().integer_value
        self.websocket_host = self.get_parameter('websocket_host').get_parameter_value().string_value
        
        # 初始化电机API
        self.motor = MotorAPI(serial_port, baud_rate)
        self.motor.start()
        
        # 启动WebSocket服务器线程
        self.websocket_thread = threading.Thread(target=self.start_websocket_server, daemon=True)
        self.websocket_thread.start()
        
        self.get_logger().info(f'Motor WebSocket server started on {self.websocket_host}:{self.websocket_port}')
    
    async def echo(self, websocket, path):
        """处理客户端连接的异步函数"""
        async for msg in websocket:
            try:
                data = json.loads(msg)
                vx = data.get("vx", 0)
                vy = data.get("vy", 0)
                vtheta = data.get("vtheta", 0)
                self.motor.set_vel(vx, vy, vtheta)
                
                # 向客户端发送确认消息
                response = {"status": "success", "received": data}
                await websocket.send(json.dumps(response))
                
            except json.JSONDecodeError:
                error_response = {"status": "error", "message": "Invalid JSON format"}
                await websocket.send(json.dumps(error_response))
            except Exception as e:
                error_response = {"status": "error", "message": str(e)}
                await websocket.send(json.dumps(error_response))
    
    def start_websocket_server(self):
        """在单独线程中启动WebSocket服务器"""
        asyncio.run(self.run_websocket_server())
    
    async def run_websocket_server(self):
        """运行WebSocket服务器"""
        async with websockets.serve(self.echo, self.websocket_host, self.websocket_port):
            await asyncio.Future()  # 保持服务器运行

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