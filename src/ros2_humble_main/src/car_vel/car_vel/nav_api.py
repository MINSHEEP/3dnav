#!/usr/bin/env python3
# coding: utf-8

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from flask import Flask, request, jsonify
import threading
import requests
import tempfile
from pydub import AudioSegment
from pydub.playback import play
import os
from flask_cors import CORS


'''
    全局变量
'''
waypoints = []       # 存储要导航的点位
current_index = 0    # 当前点的索引
navi_pub = None      # 发布目标点 Publisher
cancel_pub = None    # cancel Publisher
cmd_pub = None       # cmd_vel Publisher

app = Flask(__name__)
CORS(app)

'''
    ROS 导航逻辑
'''
# 发送导航目标
def send_goal(point_name):
    rospy.loginfo("➡️ 前往点位：%s", point_name)
    msg = String()
    msg.data = point_name
    navi_pub.publish(msg)

# 导航结果回调函数
def NavResultCallback(msg):
    global current_index
    # 防止旧消息触发导航
    if current_index >= len(waypoints):
        rospy.logwarn("⚠️ 收到旧的导航结果消息，已忽略")
        return

    rospy.logwarn("导航结果 = %s", msg.data)

    if msg.data == "done":
        rospy.loginfo("✅ 已到达点位：%s", waypoints[current_index])
    else:
        rospy.logwarn("❌ 导航失败：%s", waypoints[current_index])
    
    current_index += 1
    if current_index < len(waypoints):
        # 继续导航下一个点
        send_goal(waypoints[current_index])
    else:
        rospy.loginfo("🚀 当前任务完成")

'''
    取消导航逻辑
'''
def stop_robot():
    global waypoints, current_index
    rospy.loginfo("执行停止导航...")

    # 取消所有导航目标
    cancel_pub.publish(GoalID())
    rospy.loginfo("已取消 move_base 的所有导航目标。")

    # 清空任务队列，防止继续走下一个点
    waypoints = []
    current_index = 0

    # 发布零速度，确保机器人立即停下
    stop_msg = Twist()
    rate = rospy.Rate(10)
    for _ in range(20): 
        cmd_pub.publish(stop_msg)
        rate.sleep()

    rospy.loginfo("机器人已停止。")
    return True

'''
    取消导航接口
'''
@app.route('/stop_nav', methods=['GET'])
def stop_navigation_api():
    success = stop_robot()
    return jsonify({"success": success, "msg": "导航已停止"})

'''
    发布导航目标接口
'''
@app.route('/send_waypoints', methods=['GET'])
def send_waypoints_api():
    """
    GET 请求方式：
    /send_waypoints?points=A,B,C
    """
    global waypoints, current_index

    points_str = request.args.get("points")
    if not points_str:
        return jsonify({"success": False, "msg": "请提供 'points' 参数，例如 ?points=A,B,C"}), 400

    # 按逗号拆分成列表
    points = [p.strip() for p in points_str.split(',') if p.strip()]
    if not points:
        return jsonify({"success": False, "msg": "点位列表为空"}), 400

    # 先停止当前导航
    stop_robot()

    # 更新点位并开始导航
    waypoints = points
    current_index = 0
    send_goal(waypoints[0])
    return jsonify({"success": True, "msg": f"已开始导航 {len(points)} 个点位", "points": waypoints})

# 配置你的 TTS 服务地址
TTS_API_URL = "http://10.0.1.14:10253/tts/generate/v1"

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

'''
    tts语音函数
'''
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
    主函数
'''
if __name__ == "__main__":
    # 先初始化 ROS 节点
    rospy.init_node("wp_http_node", anonymous=True)
    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # 发送目标航点的话题
    navi_pub = rospy.Publisher("/waterplus/navi_waypoint", String, queue_size=10)
    # 订阅导航执行结果话题
    rospy.Subscriber("/waterplus/navi_result", String, NavResultCallback, queue_size=10)

    # 启动 Flask HTTP 服务放到子线程
    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5001, use_reloader=False),
        daemon=True
    ).start()

    # ROS 主循环
    rospy.spin()
