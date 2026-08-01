#!/usr/bin/env python3
# coding: utf-8

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import requests
import tempfile
from pydub import AudioSegment
from pydub.playback import play
import os
import sqlite3
import tf
import time
import actionlib
import json
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

'''
    全局变量
'''
waypoints = []       # 存储要导航的点位
current_index = 0    # 当前点的索引
cancel_pub = None    # cancel Publisher
cmd_pub = None       # cmd_vel Publisher

# 导航状态
nav_status = {
    "running": False,
    "current_point": None,
    "success": None,
    "message": ""
}
last_goal_done = None   # 防止重复 DONE
nav_lock = threading.Lock()   # ✅ 避免并发修改全局变量

app = Flask(__name__)
CORS(app)

obstacle_alerted = False   # 防止重复播报

# 雷达回调函数 ：处理检测到障碍物的时候播放避障语音
def scan_callback(msg):
    global obstacle_alerted

    if nav_status["running"]:
        # 提取激光点数据
        ranges = msg.ranges
        if not ranges:
            return

        # 前方 +-15° 的区域
        angle_increment = msg.angle_increment
        num_points = len(ranges)

        center_index = num_points // 2
        window = int((15.0/180.0*3.14159) / angle_increment)  # ±15度对应的点数

        front_ranges = ranges[center_index - window:center_index + window]
        # 过滤掉 inf
        front_ranges = [r for r in front_ranges if r > 0.0]

        if front_ranges and min(front_ranges) < 2:  # 小于 0.8m 判定为障碍物
            if not obstacle_alerted:
                obstacle_alerted = True
                rospy.logwarn("⚠️ 前方检测到障碍物")
                threading.Thread(target=lambda: speak("您好，请注意避让")).start()
        else:
            obstacle_alerted = False

# 导航回调函数
def feedback_cb(feedback):
    # 可实时更新当前位置
    pos = feedback.base_position.pose.position
    nav_status["message"] = f"前往 {nav_status['current_point']}, 当前位置 x={pos.x:.2f}, y={pos.y:.2f}"

# 导航回调函数
def done_cb(state, result):
    global current_index, waypoints, nav_status, last_goal_done

    # ✅ 加锁，避免并发问题
    with nav_lock:  
        if not waypoints or current_index >= len(waypoints):
            rospy.logwarn("done_cb 触发，但 waypoints 已清空或越界，忽略。")
            return
        rospy.logwarn(f"进回调函数 current_index = {current_index}, last_goal_done = {last_goal_done}")
        # ✅ 去重
        if last_goal_done == current_index:
            rospy.logwarn("重复 DONE，忽略")
            return
        last_goal_done = current_index

        # 成功 / 失败处理
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"已到达点位：{waypoints[current_index]['point_name']}")
            speak(f"已到达 {waypoints[current_index]['point_name']}")
        else:
            rospy.logerr(f"导航失败：{waypoints[current_index]['point_name']}")
            nav_status.update({
                "success": False,
                "running": False,
                "message": f"导航失败：{waypoints[current_index]['point_name']}"
            })
            play(AudioSegment.from_file("/home/b2w/ws_livox/src/NEXTE_Sentry_Nav/sentry_nav/audio/daohangshibai.wav"))
            return

        # 导航下一个点
        current_index += 1
        if current_index < len(waypoints):
            # 延时 0.5s 再发下一个目标，避免状态机冲突
            threading.Timer(0.5, lambda: send_nav_goal_cb(waypoints[current_index])).start()
            # send_nav_goal_cb(waypoints[current_index])
        else:
            nav_status.update({
                "success": True,
                "running": False,
                "message": "任务完成"
            })
            rospy.loginfo("任务完成")
            play(AudioSegment.from_file("/home/b2w/ws_livox/src/NEXTE_Sentry_Nav/sentry_nav/audio/renwuwancheng.wav"))

# 发送导航目标点
def send_nav_goal_cb(point):
    if not waypoints or current_index >= len(waypoints):
        rospy.logwarn("send_nav_goal_cb 被调用，但队列为空，忽略。")
        return
    nav_status.update({
        "current_point": point["point_name"],
        "running": True,
        "message": f"前往 {point['point_name']}"
    })

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = float(point["pos_x"])
    goal.target_pose.pose.position.y = float(point["pos_y"])
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = float(point["ori_z"])
    goal.target_pose.pose.orientation.w = float(point["ori_w"])

    ac.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)


# 停止导航函数
def stop_robot():
    global waypoints, current_index, last_goal_done
    with nav_lock:
        rospy.loginfo("执行停止导航...")
        # 取消所有导航目标
        cancel_pub.publish(GoalID())
        rospy.loginfo("已取消所有导航目标。")

        # 清空任务队列，防止继续走下一个点
        waypoints = []
        current_index = 0
        last_goal_done = None

        # 发布零速度，确保机器人立即停下
        stop_msg = Twist()
        for _ in range(10): 
            cmd_pub.publish(stop_msg)
            rospy.sleep(0.1)

        nav_status.update({
            "running": False,
            "success": False,
            "message": "导航已停止"
        })

    rospy.loginfo("机器人已停止。")
    play(AudioSegment.from_file("/home/b2w/ws_livox/src/NEXTE_Sentry_Nav/sentry_nav/audio/yiquxiaodaohang.wav"))
    return True


'''
    取消导航接口
'''
@app.route('/stop_nav', methods=['GET'])
def stop_navigation_api():
    success = stop_robot()
    return jsonify({"success": success, "msg": "导航已停止"})


'''
    发布导航路线接口
'''
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
    except json.JSONDecodeError as e:
        raise Exception(f"JSON解析错误: {str(e)}")
    
    print(f"json_array = {json_array}")
    pointIds = [int(item['point_id']) for item in json_array if 'point_id' in item]
    waypoints = []
    for pointId in pointIds:
        point = execute_sql("SELECT * FROM point WHERE id = ?", (pointId,), fetch="one")
        if point:
            waypoints.append(point)

    if len(waypoints) < 1:
        speak(f"{nav_route['route_name']}路线设置有误，取消导航")
        return jsonify({"success": False, "msg": f"{nav_route['route_name']}路线设置有误，取消导航"}), 400

    # 播放语音
    speak(f"已开始按照{nav_route['route_name']}路线进行导航")
    
    # 更新点位并开始导航
    current_index = 0
    last_goal_done = None
    send_nav_goal_cb(waypoints[0])
    return jsonify({"success": True, "msg": f"已开始按照'{nav_route['route_name']}'路线进行导航", "points": waypoints})


'''
    发送导航目标点接口
'''
@app.route('/send_point', methods=['GET'])
def send_point():
    global waypoints, current_index, last_goal_done

    point_ids = request.args.get("point_ids")
    map_name = request.args.get("map_name")
    if not point_ids:
        return jsonify({"success": False, "msg": "请提供 'point_ids' 参数，例如 ?point_ids=1,2,3"}), 400

    try:
        # 按逗号拆分成列表
        points = [int(p.strip()) for p in point_ids.split(',') if p.strip()]
    except ValueError:
        return jsonify({"success": False, "msg": "点位ID必须是整数"}), 400
    
    # 查询点位信息
    placeholders = ",".join("?" for _ in points)  # "?,?,?"
    sql = f"SELECT * FROM point WHERE id IN ({placeholders}) AND map_name = ?"
    params = points + [map_name]  # 展开列表
    point_list = execute_sql(sql, params, fetch="all")

    if not point_list:
        return jsonify({"success": False, "msg": "没有找到点位信息"}), 500

    # 初始化全局导航队列
    waypoints = point_list
    current_index = 0
    last_goal_done = None

    # 启动第一个点
    send_nav_goal_cb(waypoints[0])
    play(AudioSegment.from_file("/home/b2w/ws_livox/src/NEXTE_Sentry_Nav/sentry_nav/audio/kaishidaohang.wav"))

    return jsonify({"success": True, "msg": "导航已开始", "points": points})

'''
    查询导航状态
'''
@app.route('/nav_status', methods=['GET'])
def get_nav_status():
    return jsonify(nav_status)

'''
    获取机器人位姿并入库
'''
@app.route("/get_robot_pose", methods=["GET"])
def get_robot_pose():
    map_name = request.args.get("map_name")
    point_name = request.args.get("point_name")
    if not map_name or not point_name:
        return jsonify({"error": "请提供 map_name 和 point_name 参数"}), 400
    try:
        # 创建 TF 监听器
        listener = tf.TransformListener()
        time.sleep(1.0)  # 等待 1 秒，确保 TF 数据开始接收
        # 获取从 map 到 body 的变换
        (trans, rot) = listener.lookupTransform("map", "body", rospy.Time(0))

        pos_x, pos_y = trans[0], trans[1]
        ori_z, ori_w = rot[2], rot[3]

        # 保存到数据库
        sql = """
        INSERT INTO point (map_name, point_name, pos_x, pos_y, ori_z, ori_w)
        VALUES (?, ?, ?, ?, ?, ?)
        """
        execute_sql(sql, (map_name, point_name, pos_x, pos_y, ori_z, ori_w))

        # 返回 JSON
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

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return jsonify({"error": "TF transform not available"}), 500


# 配置你的 TTS 服务地址
TTS_API_URL = "http://10.0.1.14:10253/tts/generate/v1"


# tts语音函数
def speak(text):
    # 向 TTS 接口发送请求
    try:
        schema = {"id":"male_03", "text": text}
        print(schema)
        response = requests.post(TTS_API_URL, data=schema)
        response.raise_for_status()
    except Exception as e:
        rospy.logerr(f"TTS 接口调用失败: {str(e)}")

    # 保存音频文件到临时目录
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=".wmv") as tmp_audio:
            tmp_audio.write(response.content)
            tmp_audio_path = tmp_audio.name

        # 播放音频
        audio = AudioSegment.from_file(tmp_audio_path)
        play(audio)
        os.remove(tmp_audio_path)
    except Exception as e:
        rospy.logerr(f"音频播放失败: {str(e)}")
    return True


'''
    tts语音接口api
'''
@app.route('/api/speak', methods=['POST'])
def speak_api():
    # 向 TTS 接口发送请求
    data = request.get_json()
    if not data or "text" not in data:
        return jsonify({"success": False, "msg": "缺少text参数"}), 400
    speak(data["text"])
    return {"success": True, "message": "播放完成"}


# 数据库地址
DB_PATH = "/home/b2w/ws_livox/src/NEXTE_Sentry_Nav/sentry_nav/nav.db"

'''
    数据库操作工具函数
'''
def execute_sql(sql, params=(), fetch=None):
    """
    通用 SQL 执行函数
    :param sql: SQL 语句
    :param params: 参数元组
    :param fetch: None / "one" / "all"
    :return: 根据 fetch 返回 None / 单行数据 / 多行列表
    """
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row   # 支持字典方式取值
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

'''
    查询点位列表
'''
@app.route('/get_points', methods=['GET'])
def get_points():
    map_name = request.args.get("map_name")
    point_list = execute_sql("SELECT * FROM point WHERE map_name = ? ", (map_name,), fetch="all")
    return jsonify({"success": True, "msg": "查询成功", "data": point_list})

'''
    查询路线列表
'''
@app.route('/get_routes', methods=['GET'])
def get_route():
    map_name = request.args.get("map_name")
    route_list = execute_sql("SELECT * FROM nav_route WHERE map_name = ? ", (map_name,), fetch="all")
    return jsonify({"success": True, "msg": "查询成功", "data": route_list})

'''
    启动flask
'''
def run_flask():
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.DEBUG)  # 打印请求日志
    app.run(host="0.0.0.0", port=5002, debug=True, use_reloader=False)


'''
    主函数
'''
if __name__ == "__main__":
    # 先初始化 ROS 节点
    rospy.init_node("nav_http_node", anonymous=True)
    # 发送取消导航话题
    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    # 发送运动话题
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # 雷达话题
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    # 全局创建 ActionClient
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    ac.wait_for_server()

    threading.Thread(target=run_flask, daemon=True).start()

    # ROS 主循环
    rospy.spin()
