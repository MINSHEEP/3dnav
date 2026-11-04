#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import requests
import tempfile
import os
import sqlite3
import json
import time
from pydub import AudioSegment
from pydub.playback import play

# ---------- 全局变量 ----------
waypoints = []          # 存储导航点位列表（字典）
current_index = 0       # 当前点索引
nav_status = {
    "running": False,
    "current_point": None,
    "success": None,
    "message": ""
}
last_goal_done = None
nav_lock = threading.Lock()
obstacle_alerted = False

# ROS 2 节点（在 main 中初始化）
node = None
action_client = None
tf_buffer = None
tf_listener = None
cmd_vel_pub = None
cancel_pub = None

# Flask app
app = Flask(__name__)
CORS(app)

# TTS 服务地址（可配置）
TTS_API_URL = "http://10.0.1.14:10253/tts/generate/v1"

# 数据库路径（可修改）
DB_PATH = "/home/a/3dnav/nav.db"

# ---------- 数据库操作 ----------
def execute_sql(sql, params=(), fetch=None):
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    cursor.execute(sql, params)
    if fetch == "all":
        result = [dict(row) for row in cursor.fetchall()]
    elif fetch == "one":
        row = cursor.fetchone()
        result = dict(row) if row else None
    else:
        result = None
    conn.commit()
    cursor.close()
    conn.close()
    return result

# ---------- TTS 语音 ----------
def speak(text):
    try:
        schema = {"id": "male_03", "text": text}
        response = requests.post(TTS_API_URL, data=schema, timeout=5)
        response.raise_for_status()
    except Exception as e:
        node.get_logger().error(f"TTS 接口调用失败: {e}")
        return
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=".wmv") as tmp:
            tmp.write(response.content)
            tmp_path = tmp.name
        audio = AudioSegment.from_file(tmp_path)
        play(audio)
        os.remove(tmp_path)
    except Exception as e:
        node.get_logger().error(f"音频播放失败: {e}")

# ---------- 导航 Action 回调 ----------
def feedback_cb(feedback):
    pos = feedback.current_pose.pose.position
    nav_status["message"] = f"前往 {nav_status['current_point']}, 当前位置 x={pos.x:.2f}, y={pos.y:.2f}"

def done_cb(future):
    global current_index, waypoints, nav_status, last_goal_done
    with nav_lock:
        if not waypoints or current_index >= len(waypoints):
            return
        if last_goal_done == current_index:
            return
        last_goal_done = current_index

        result = future.result()
        if result.status == rclpy.action.GoalStatus.STATUS_SUCCEEDED:
            node.get_logger().info(f"已到达点位：{waypoints[current_index]['point_name']}")
            speak(f"已到达 {waypoints[current_index]['point_name']}")
        else:
            node.get_logger().error(f"导航失败：{waypoints[current_index]['point_name']}")
            nav_status.update({
                "success": False,
                "running": False,
                "message": f"导航失败：{waypoints[current_index]['point_name']}"
            })
            speak("导航失败")
            return

        current_index += 1
        if current_index < len(waypoints):
            threading.Timer(0.5, lambda: send_nav_goal(waypoints[current_index])).start()
        else:
            nav_status.update({
                "success": True,
                "running": False,
                "message": "任务完成"
            })
            node.get_logger().info("任务完成")
            speak("任务完成")

def send_nav_goal(point):
    if not waypoints or current_index >= len(waypoints):
        return
    nav_status.update({
        "current_point": point["point_name"],
        "running": True,
        "message": f"前往 {point['point_name']}"
    })

    goal_msg = PoseStamped()
    goal_msg.header.frame_id = "map"
    goal_msg.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.position.x = float(point["pos_x"])
    goal_msg.pose.position.y = float(point["pos_y"])
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = float(point["ori_z"])
    goal_msg.pose.orientation.w = float(point["ori_w"])

    action_client.send_goal_async(goal_msg, feedback_callback=feedback_cb).add_done_callback(done_cb)

# ---------- 停止导航 ----------
def stop_robot():
    global waypoints, current_index, last_goal_done
    with nav_lock:
        node.get_logger().info("执行停止导航...")
        # 取消当前目标
        if action_client:
            action_client.cancel_goal_async()
        # 清空队列
        waypoints = []
        current_index = 0
        last_goal_done = None
        # 发布零速
        if cmd_vel_pub:
            from geometry_msgs.msg import Twist
            stop_msg = Twist()
            for _ in range(10):
                cmd_vel_pub.publish(stop_msg)
                time.sleep(0.1)
        nav_status.update({
            "running": False,
            "success": False,
            "message": "导航已停止"
        })
    node.get_logger().info("机器人已停止。")
    speak("已取消导航")
    return True

# ---------- Flask 路由 ----------
@app.route('/stop_nav', methods=['GET'])
def stop_navigation_api():
    success = stop_robot()
    return jsonify({"success": success, "msg": "导航已停止"})

@app.route('/send_route', methods=['GET'])
def send_nav_route():
    global waypoints, current_index, last_goal_done
    route_id = request.args.get("route_id")
    if not route_id:
        return jsonify({"success": False, "msg": "请提供 'route_id' 参数"}), 400

    nav_route = execute_sql("SELECT * FROM nav_route WHERE id = ?", (route_id,), fetch="one")
    if not nav_route or not nav_route["route_points"]:
        return jsonify({"success": False, "msg": "路线不存在"}), 400
    try:
        json_array = json.loads(nav_route["route_points"])
    except:
        return jsonify({"success": False, "msg": "路线格式错误"}), 400

    pointIds = [int(item['point_id']) for item in json_array if 'point_id' in item]
    waypoints = []
    for pid in pointIds:
        point = execute_sql("SELECT * FROM point WHERE id = ?", (pid,), fetch="one")
        if point:
            waypoints.append(point)
    if len(waypoints) < 1:
        speak(f"{nav_route['route_name']}路线设置有误，取消导航")
        return jsonify({"success": False, "msg": f"{nav_route['route_name']}路线设置有误"}), 400

    speak(f"已开始按照{nav_route['route_name']}路线进行导航")
    current_index = 0
    last_goal_done = None
    send_nav_goal(waypoints[0])
    return jsonify({"success": True, "msg": f"已开始按照'{nav_route['route_name']}'路线进行导航"})

@app.route('/send_point', methods=['GET'])
def send_point():
    global waypoints, current_index, last_goal_done
    point_ids = request.args.get("point_ids")
    map_name = request.args.get("map_name")
    if not point_ids:
        return jsonify({"success": False, "msg": "请提供 'point_ids' 参数"}), 400
    try:
        points = [int(p.strip()) for p in point_ids.split(',') if p.strip()]
    except:
        return jsonify({"success": False, "msg": "点位ID必须是整数"}), 400

    placeholders = ",".join("?" for _ in points)
    sql = f"SELECT * FROM point WHERE id IN ({placeholders}) AND map_name = ?"
    params = points + [map_name]
    point_list = execute_sql(sql, params, fetch="all")
    if not point_list:
        return jsonify({"success": False, "msg": "没有找到点位信息"}), 404

    waypoints = point_list
    current_index = 0
    last_goal_done = None
    speak("开始导航")
    send_nav_goal(waypoints[0])
    return jsonify({"success": True, "msg": "导航已开始", "points": points})

@app.route('/nav_status', methods=['GET'])
def get_nav_status():
    return jsonify(nav_status)

@app.route('/get_robot_pose', methods=['GET'])
def get_robot_pose():
    map_name = request.args.get("map_name")
    point_name = request.args.get("point_name")
    if not map_name or not point_name:
        return jsonify({"error": "请提供 map_name 和 point_name 参数"}), 400
    try:
        # 使用 tf_buffer 获取 map 到 base_link 的变换
        if not tf_buffer:
            return jsonify({"error": "TF buffer not ready"}), 500
        transform = tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        trans = transform.transform.translation
        rot = transform.transform.rotation
        pos_x, pos_y = trans.x, trans.y
        ori_z, ori_w = rot.z, rot.w

        sql = """
        INSERT INTO point (map_name, point_name, pos_x, pos_y, ori_z, ori_w)
        VALUES (?, ?, ?, ?, ?, ?)
        """
        execute_sql(sql, (map_name, point_name, pos_x, pos_y, ori_z, ori_w))

        pose_data = {
            "map_name": map_name,
            "point_name": point_name,
            "Pos_x": pos_x,
            "Pos_y": pos_y,
            "Ori_z": ori_z,
            "Ori_w": ori_w
        }
        speak(f"已保存 {point_name} 的位姿信息")
        return jsonify(pose_data)
    except TransformException as ex:
        node.get_logger().error(f"TF transform error: {ex}")
        return jsonify({"error": "TF transform not available"}), 500

@app.route('/get_points', methods=['GET'])
def get_points():
    map_name = request.args.get("map_name")
    point_list = execute_sql("SELECT * FROM point WHERE map_name = ?", (map_name,), fetch="all")
    return jsonify({"success": True, "msg": "查询成功", "data": point_list})

@app.route('/get_routes', methods=['GET'])
def get_routes():
    map_name = request.args.get("map_name")
    route_list = execute_sql("SELECT * FROM nav_route WHERE map_name = ?", (map_name,), fetch="all")
    return jsonify({"success": True, "msg": "查询成功", "data": route_list})

@app.route('/api/speak', methods=['POST'])
def speak_api():
    data = request.get_json()
    if not data or "text" not in data:
        return jsonify({"success": False, "msg": "缺少text参数"}), 400
    speak(data["text"])
    return {"success": True, "message": "播放完成"}

# ---------- 雷达回调（避障提醒） ----------
def scan_callback(msg):
    global obstacle_alerted
    if not nav_status["running"]:
        return
    ranges = msg.ranges
    if not ranges:
        return
    angle_increment = msg.angle_increment
    num_points = len(ranges)
    center_index = num_points // 2
    window = int((15.0/180.0*3.14159) / angle_increment)
    front_ranges = ranges[center_index - window:center_index + window]
    front_ranges = [r for r in front_ranges if r > 0.0]
    if front_ranges and min(front_ranges) < 2.0:
        if not obstacle_alerted:
            obstacle_alerted = True
            node.get_logger().warn("⚠️ 前方检测到障碍物")
            threading.Thread(target=lambda: speak("您好，请注意避让")).start()
    else:
        obstacle_alerted = False

# ---------- 主函数 ----------
def run_flask():
    # 配置日志级别
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.INFO)
    app.run(host="0.0.0.0", port=5002, debug=False, use_reloader=False)

def main(args=None):
    global node, action_client, tf_buffer, tf_listener, cmd_vel_pub
    rclpy.init(args=args)
    node = Node("nav_http_node_ros2")

    # 创建 Action Client (NavigateToPose)
    action_client = ActionClient(node, nav2_msgs.action.NavigateToPose, 'navigate_to_pose')
    # 等待服务器（超时可配置）
    if not action_client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("NavigateToPose action server not available!")
        return

    # 创建 TF 监听器
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    # 创建 cmd_vel 发布者（用于停止）
    from geometry_msgs.msg import Twist
    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # 订阅 /scan
    node.create_subscription(LaserScan, '/scan', scan_callback, 10)

    # 启动 Flask 线程
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
