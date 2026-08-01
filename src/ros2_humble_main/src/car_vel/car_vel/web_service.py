from flask import Flask, request, jsonify
from flask_cors import CORS
from motor_api import MotorAPI
import argparse

app = Flask(__name__)
CORS(app)

motor = MotorAPI('/dev/ttyUSB0', 115200)
motor.start()

# 设置移动速度
@app.route('/set_vel', methods=['POST'])
def set_vel():
    data = request.json
    motor.set_vel(data['vx'], data['vy'], data['vtheta'])
    return "success"

@app.route('/forward')
def forward():
    motor.set_vel(100, 0, 0)
    return "success"

@app.route('/back')
def back():
    motor.set_vel(-100, 0, 0)
    return "success"

@app.route('/left')
def left():
    motor.set_vel(0, 100, 0)
    return "success"

@app.route('/right')
def right():
    motor.set_vel(0, -100, 0)
    return "success"

@app.route('/around')
def around():
    motor.set_vel(0, 0, 1)
    return "success"

@app.route('/stop')
def stop():
    motor.set_vel(0, 0, 0)
    return "success"

if __name__ == '__main__':
    # 添加命令行参数解析
    parser = argparse.ArgumentParser(description='运行任务管理系统')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='服务器IP地址')
    parser.add_argument('--port', type=int, default=5000, help='服务器端口号')
    parser.add_argument('--debug', action='store_true', help='开启调试模式')
    
    args = parser.parse_args()
    
    # 使用指定的IP和端口运行应用
    app.run(host=args.host, port=args.port, debug=args.debug)
