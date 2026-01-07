import rclpy
from rclpy.node import Node
from robomaster import robot
import time

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        self.get_logger().info('RoboMaster 연결 및 센서 초기화 중...')

        # 로봇 객체 생성 및 초기화
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="sta")

        # 각 모듈 가져오기
        self.ep_battery = self.ep_robot.battery
        self.ep_chassis = self.ep_robot.chassis
        self.ep_gimbal = self.ep_robot.gimbal
        self.ep_sensor = self.ep_robot.sensor
        self.ep_camera = self.ep_robot.camera
        self.ep_armor = self.ep_robot.armor

        # 1. 배터리 정보 구독 (1Hz: 1초에 한 번)
        self.ep_battery.sub_battery_info(1, self.sub_battery_handler)
        
        # 2. 섀시 위치 정보 구독 (5Hz)
        self.ep_chassis.sub_position(freq=5, callback=self.sub_position_handler)
        
        # 3. 짐벌 각도 정보 구독 (5Hz)
        self.ep_gimbal.sub_angle(freq=5, callback=self.sub_gimbal_handler)

        # 4. TOF(거리) 센서 구독 (5Hz) - 어댑터가 있어야 작동하지만 코드엔 포함
        self.ep_sensor.sub_distance(freq=5, callback=self.sub_tof_handler)

        # 5. 아머(충격) 감지 이벤트 구독
        self.ep_armor.set_hit_sensitivity(comp="all", sensitivity=5) # 민감도 설정
        self.ep_armor.sub_hit_event(self.sub_armor_handler)

        # 6. 카메라 스트림 시작 (화면 표시는 끄고 데이터 수신만 확인)
        self.ep_camera.start_video_stream(display=False)
        self.get_logger().info('모든 센서 구독 완료! 데이터를 기다립니다...')

    # --- 콜백 함수들 (데이터가 들어오면 실행됨) ---

    def sub_battery_handler(self, battery_info):
        percent, voltage = battery_info
        self.get_logger().info(f'[배터리] 잔량: {percent}%, 전압: {voltage}mV')

    def sub_position_handler(self, position_info):
        x, y, z = position_info
        # 소수점 2자리까지만 출력
        self.get_logger().info(f'[위치] X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f}')

    def sub_gimbal_handler(self, angle_info):
        pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = angle_info
        self.get_logger().info(f'[짐벌] Pitch:{pitch_angle:.1f}, Yaw:{yaw_angle:.1f}')

    def sub_tof_handler(self, dist_info):
        dist = dist_info[0] # 거리 (mm)
        self.get_logger().info(f'[TOF 거리센서] 전방 장애물까지: {dist}mm')

    def sub_armor_handler(self, armor_info):
        armor_id, hit_type = armor_info
        self.get_logger().warn(f'!!! [충격 감지] 맞은 부위 ID: {armor_id}, 타입: {hit_type} !!!')

    def close(self):
        self.ep_camera.stop_video_stream()
        self.ep_robot.close()

def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 중...')
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
