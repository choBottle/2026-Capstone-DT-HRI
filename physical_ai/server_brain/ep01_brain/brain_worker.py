import socketio
import os
from robomaster import robot
import time
import sys

# 1. 환경 변수 및 접속 정보 설정
sio = socketio.Client(reconnection=True)
SERVER_URL = os.getenv('SERVER_URL', 'http://127.0.0.1:5000')
TARGET_IP = os.getenv('ROBOT_IP', '192.168.50.130') # 라즈베리파이 IP

class EP01BrainWorker:
    def __init__(self):
        self.robot_id = "EP01_CHASSIS_01"
        # SDK 객체 생성 (로봇 탐색 모드 비활성화를 위해 sn=None 주입 준비)
        self.ep_robot = robot.Robot()
        
        try:
            print(f"🚀 [Brain] SDK 초기화 시도 (Target Relay: {TARGET_IP})")
            
            # [핵심] 릴레이를 사용할 때는 직접 IP를 지정하여 초기화해야 합니다.
            # SDK 버전에 따라 주소 강제 할당이 필요할 수 있습니다.
            self.ep_robot.initialize(conn_type="sta", sn=None)
            
            # SDK 내부 클라이언트 주소를 라즈베리파이로 강제 고정 (매우 중요)
            self.ep_robot._client.client_ip = TARGET_IP
            
            print(f"✅ [Brain] SDK 초기화 완료! (Relay connected)")
            
            # 2. 센서 데이터 구독 설정
            # 배터리: 1Hz (초당 1회)
            self.ep_robot.battery.sub_battery_info(freq=1, callback=self.on_battery)
            # IMU/가속도: 10Hz (충격 감지용)
            self.ep_robot.chassis.sub_imu(freq=10, callback=self.on_imu)
            
            print("📡 [Brain] 데이터 구독 시작...")
            
        except Exception as e:
            print(f"❌ [Brain] 로봇 연결 에러: {e}")
            sys.exit(1)

    def on_battery(self, info):
        # info는 (percent,) 형태의 튜플일 수 있습니다.
        percent = info
        print(f"🔋 [Live Data] Battery: {percent}%")
        
        # 허브 서버로 전송
        try:
            sio.emit('worker_to_hub', {
                "robot_id": self.robot_id, 
                "robot_type": "EP01",
                "type": "battery", 
                "val": percent
            })
        except Exception as e:
            print(f"❌ 허브 전송 실패 (Battery): {e}")

    def on_imu(self, info):
        # 가속도 x, y, z, 자이로 x, y, z
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = info
        
        # 일정 수치 이상의 충격(가속도) 감지 시 이벤트 발생
        if abs(acc_x) > 1.5 or abs(acc_y) > 1.5:
            print(f"⚠️ [Event] Impact Detected! (X: {acc_x:.2f}, Y: {acc_y:.2f})")
            sio.emit('worker_to_hub', {
                "robot_id": self.robot_id, 
                "robot_type": "EP01",
                "type": "impact", 
                "val": "COLLISION"
            })

# 3. 메인 실행 루프
if __name__ == '__main__':
    try:
        # 허브(노트북의 main_server)에 소켓 연결
        print(f"🔗 [Hub] Connecting to {SERVER_URL}...")
        sio.connect(SERVER_URL)
        print("✅ [Hub] Connected!")
        
        # 브레인 워커 가동
        worker = EP01BrainWorker()
        
        # [중요] 프로그램이 종료되지 않도록 무한 대기
        # 이 루프가 있어야 콜백 함수들이 계속 동작합니다.
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n👋 사용자에 의해 프로그램이 종료되었습니다.")
    except Exception as e:
        print(f"🔥 치명적 오류 발생: {e}")
    finally:
        # 자원 해제
        if 'worker' in locals():
            worker.ep_robot.close()
        sio.disconnect()