import os
import time
import sys
from robomaster import robot
import robomaster.config

def start_control():
    # 1. 환경변수에서 로봇 IP 가져오기 (Hub가 주입해준 값)
    robot_ip = os.getenv('ROBOT_IP')
    
    print("====================================", flush=True)
    print(f"🚀 [Edge-Control] 로봇 제어 프로세스 시작", flush=True)
    print(f"📡 타겟 로봇 IP: {robot_ip}", flush=True)
    print("====================================", flush=True)

    if not robot_ip:
        print("❌ 에러: ROBOT_IP 환경변수를 찾을 수 없습니다.", flush=True)
        return

    # 2. [핵심] SDK 전역 설정에 IP 직접 주입
    # 이렇게 하면 initialize() 호출 시 브로드캐스팅 없이 해당 IP로 직행합니다.
    robomaster.config.DEFAULT_CONN_TYPE = "sta"
    robomaster.config.DEFAULT_STA_IP = robot_ip
    robomaster.config.DEFAULT_CONN_PROTO = "fd" # 통신 프로토콜 설정

    # 3. 로봇 객체 생성 및 초기화
    ep_robot = robot.Robot()
    
    try:
        print(f"🔗 로봇 연결 시도 중...", flush=True)
        # 설정된 IP를 사용하므로 별도 인자 없이 initialize 호출
        res = ep_robot.initialize(conn_type="sta", proto_type="tcp")
        
        # SDK 버전에 따라 성공 시 0 또는 True 반환
        if res == 0 or res is True:
            print(f"✅ 연결 성공! 로봇 기동을 시작합니다.", flush=True)
            
            # [동작 시퀀스]
            print("🚲 1. 전진 (0.3m)...", flush=True)
            ep_robot.chassis.move(x=0.3, y=0, z=0, xy_speed=0.6).wait_for_completed(timeout=5)
            
            time.sleep(1)
            
            print("🚲 2. 후진 (0.3m)...", flush=True)
            ep_robot.chassis.move(x=-0.3, y=0, z=0, xy_speed=0.6).wait_for_completed(timeout=5)
            
            print("✨ 모든 제어 시퀀스가 완료되었습니다.", flush=True)
        else:
            print(f"❌ 연결 실패 (결과코드: {res})", flush=True)
            print("💡 팁: 로봇과 라즈베리파이가 같은 Wi-Fi에 있는지 확인하세요.", flush=True)

    except Exception as e:
        print(f"❌ 제어 중 예외 발생: {str(e)}", flush=True)
        import traceback
        traceback.print_exc()
    finally:
        # 4. 자원 해제 및 종료
        ep_robot.close()
        print("🏁 제어 프로세스 종료 및 파드 반환", flush=True)

if __name__ == "__main__":
    start_control()