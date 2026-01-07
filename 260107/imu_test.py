import rclpy
from rclpy.node import Node
from robomaster import robot
import time

class RoboMasterImpactTest(Node):
    def __init__(self):
        super().__init__('impact_test_node')
        self.get_logger().info('=== [충격 감지 전용 모드] 시작 ===')
        self.get_logger().info('로봇을 전후좌우로 툭툭 쳐보며 로그를 확인하세요.')
        
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="sta")

        # IMU 데이터만 구독 (주기 20Hz로 상향 - 더 민감하게 감지)
        self.ep_robot.chassis.sub_imu(freq=20, callback=self.sub_imu_handler)

    def sub_imu_handler(self, imu_info):
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
        
        # 문턱값 설정 (1.0이 평소 중력)
        # 1.2: 아주 민감함 / 1.5: 보통 충격 / 2.0: 강한 충돌
        threshold = 0.5
        
        impact_detected = False
        msg = ""

        # X축 (전후)
        if acc_x > threshold: 
            msg += f" [FRONT] {acc_x:.2f}"
            impact_detected = True
        elif acc_x < -threshold: 
            msg += f" [BACK] {acc_x:.2f}"
            impact_detected = True

        # Y축 (좌우)
        if acc_y > threshold: 
            msg += f" [LEFT] {acc_y:.2f}"
            impact_detected = True
        elif acc_y < -threshold: 
            msg += f" [RIGHT] {acc_y:.2f}"
            impact_detected = True

        if impact_detected:
            # 충격 발생 시에만 노란색 경고 로그 출력
            self.get_logger().warn(f'>>> IMPACT DETECTED!{msg}')

    def stop(self):
        self.ep_robot.chassis.unsub_imu()
        self.ep_robot.close()
        self.get_logger().info('테스트 종료')

def main(args=None):
    rclpy.init(args=args)
    node = RoboMasterImpactTest()
    try:
        # 30초 동안 테스트 진행
        time.sleep(30)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
