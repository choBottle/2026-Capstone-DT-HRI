import robomaster
from robomaster import robot
import time

def main():
    ep_robot = robot.Robot()
    
    # 1. 로봇 연결 (공유기 모드)
    print("--- [1] 로봇 연결 시도 ---")
    ep_robot.initialize(conn_type="sta")
    print(">>> 연결 성공!")

    # 2. 배터리 정보 구독 (Callback 방식)
    def battery_handler(percent):
        print(f">>> 현재 배터리 잔량: {percent}%")
    
    ep_robot.battery.sub_battery_info(1, battery_handler)
    time.sleep(1) # 정보 수신 대기

    # 3. LED 제어
    print("\n--- [2] LED 테스트 ---")
    print(">>> LED: 빨간색 (경고)")
    ep_robot.led.set_led(comp="all", r=255, g=0, b=0, effect="on")
    time.sleep(1)

    print(">>> LED: 초록색 (정상)")
    ep_robot.led.set_led(comp="all", r=0, g=255, b=0, effect="on")
    time.sleep(1)

    # 4. 이동 제어 (안전거리 확보 필수)
    print("\n--- [3] 이동 테스트 ---")
    chassis = ep_robot.chassis
    speed = 0.5

    print(">>> 앞으로 0.5m 전진")
    chassis.move(x=0.5, y=0, z=0, xy_speed=speed).wait_for_completed()
    
    print(">>> 우측으로 0.3m 이동 (게걸음)")
    chassis.move(x=0, y=0.3, z=0, xy_speed=speed).wait_for_completed()

    print(">>> 제자리 회전 180도")
    chassis.move(x=0, y=0, z=180, z_speed=100).wait_for_completed()

    # 5. 종료 및 자원 해제
    print("\n--- [4] 테스트 종료 ---")
    ep_robot.battery.unsub_battery_info()
    ep_robot.close()
    print(">>> 연결을 종료했습니다.")

if __name__ == '__main__':
    main()