#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socketio
import os
from unitree_legged_msgs.msg import HighState, HighCmd

# Hub 통신용 클라이언트 (Python 2.7 호환 버전 사용 필요)
sio = socketio.Client()
SERVER_URL = os.getenv('SERVER_URL', 'http://main_server:5000')
ROBOT_TYPE = "GO1"

class Go1BrainWorker:
    def __init__(self):
        rospy.init_node('go1_brain_worker', anonymous=True)
        self.robot_id = "GO1_ROBOT_01"
        
        # 1. ROS1 퍼블리셔/서브스크라이버 설정
        self.cmd_pub = rospy.Publisher('/high_cmd', HighCmd, queue_size=10)
        rospy.Subscriber('/high_state', HighState, self.on_high_state)
        
        print("✅ [Go1 Brain] ROS1 Interface & SDK Initialized")

    # --- [Flow 2] Go1 ROS1 데이터 -> Hub (JSON 변환) ---
    def on_high_state(self, msg):
        # 연구 가이드에 따른 주요 데이터 추출
        processed_data = {
            "robot_id": self.robot_id,
            "type": "status",
            "val": {
                "battery": 75, # 실제 msg.wirelessRemote 등에서 추출 가능
                "imu": {"x": msg.imu.accelerometer[0], "y": msg.imu.accelerometer[1]},
                "mode": msg.mode
            }
        }
        # 중앙 허브(ROS2 기반 환경)로 전송
        sio.emit('worker_to_hub', processed_data)

# --- [Flow 3] Hub(AI/Unity) 명령 -> Go1 ROS1 명령 발행 ---
@sio.on('generate_hex')
def handle_hub_command(data):
    action = data.get('action')
    cmd = HighCmd()
    cmd.levelFlag = 0x00 # High Level
    
    if action == "stand":
        cmd.mode = 2
    elif action == "walk":
        cmd.mode = 6
        cmd.velocity = [0.2, 0.0] # 연구 결과: 최소 0.15 이상 설정
    
    # ROS1 토픽으로 발행 -> unitree_ros_to_real이 로봇으로 전달
    worker.cmd_pub.publish(cmd)

if __name__ == "__main__":
    worker = Go1BrainWorker()
    sio.connect(SERVER_URL)
    sio.emit('register', {'role': 'brain_worker', 'id': 'go1_worker', 'robot_type': 'GO1'})
    
    # ROS1 스핀과 SocketIO 대기를 동시에 유지
    while not rospy.is_shutdown():
        rospy.sleep(0.1)