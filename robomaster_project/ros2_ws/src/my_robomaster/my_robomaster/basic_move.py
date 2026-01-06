import rclpy
from rclpy.node import Node
from robomaster import robot

class BasicMove(Node):
    def __init__(self):
        super().__init__('basic_move')
        # 로봇 초기화 (SDK 직접 사용)
        self.get_logger().info('로봇 연결 시도 중...')
        self.ep_robot = robot.Robot()
        # 연결 타입은 로봇 설정에 맞춰 "sta" 또는 "ap"로 변경하세요.
        self.ep_robot.initialize(conn_type="sta") 
        
        self.get_logger().info('로봇과 연결되었습니다! 1m 전진합니다.')
        
        # 1미터 전진 예시
        self.chassis = self.ep_robot.chassis
        self.chassis.move(x=1, y=0, z=0, v=0.5).wait_for_completed()

    def destroy_node(self):
        self.ep_robot.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BasicMove()
    try:
        # 노드를 한 번 실행하고 종료 (단순 이동 명령)
        rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
