import rclpy
from rclpy.node import Node
from robomaster import robot
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class MonitorRobot(Node):
    def __init__(self):
        super().__init__('monitor_robot')
        self.get_logger().info('RoboMaster 모니터링 노드 시작...')

        # 1. 로봇 연결 (AP 모드: 로봇 와이파이 직접 연결)
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="ap")
        self.get_logger().info('로봇 연결 성공!')

        # 2. ROS 퍼블리셔 생성
        self.bat_pub = self.create_publisher(Int32, '/robomaster/battery', 10)
        self.pos_pub = self.create_publisher(Point, '/robomaster/position', 10)
        self.img_pub = self.create_publisher(Image, '/robomaster/camera/image_raw', 10)
        
        # 이미지 변환 도구
        self.bridge = CvBridge()

        # 3. 로보마스터 구독 시작
        self.ep_robot.battery.sub_battery_info(freq=1, callback=self.battery_callback)
        self.ep_robot.chassis.sub_position(freq=10, callback=self.position_callback)
        
        # 카메라 시작 (720p 해상도)
        self.ep_robot.camera.start_video_stream(display=False, resolution='720p')

        # 4. 주기적 실행 (0.05초마다 = 20Hz)
        self.timer = self.create_timer(0.05, self.camera_loop)
        
        # 저장용 카운터 (너무 자주 저장하지 않기 위해)
        self.save_count = 0

    def battery_callback(self, battery_info):
        percent = battery_info
        msg = Int32()
        msg.data = percent
        self.bat_pub.publish(msg)

    def position_callback(self, position_info):
        x, y, z = position_info
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.pos_pub.publish(msg)

    def camera_loop(self):
        try:
            # 로봇에서 가장 최신 이미지 한 장을 가져옴
            img = self.ep_robot.camera.read_cv2_image(strategy="newest")
            
            if img is not None:
                # 1. ROS 메시지로 변환하여 발행 (노트북 전송용)
                ros_image = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.img_pub.publish(ros_image)

                # 2. 사진 파일로 저장 (확인용)
                # 매번 저장하면 느려지니 20번 실행될 때마다(약 1초에 한번) 저장
                self.save_count += 1
                if self.save_count % 20 == 0:
                    save_path = '/root/robomaster_project/check_camera.jpg'
                    cv2.imwrite(save_path, img)
                    # self.get_logger().info(f'사진 저장 완료: {save_path}')
                    
        except Exception as e:
            self.get_logger().warn(f'카메라 에러: {e}')

    def destroy_node(self):
        self.ep_robot.camera.stop_video_stream()
        self.ep_robot.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MonitorRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
