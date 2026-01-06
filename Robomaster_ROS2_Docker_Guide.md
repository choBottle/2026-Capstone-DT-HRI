# [Manual] Docker 기반 ROS 2 Foxy & RoboMaster SDK 환경 구축

이 매뉴얼은 Ubuntu 22.04(Humble) 환경에서 RoboMaster SDK와의 호환성 문제를 해결하기 위해 **Docker를 이용한 ROS 2 Foxy 환경 구축** 및 **순수 SDK 제어 노드 작성** 과정을 설명합니다.

## 1. 전제 조건 (Prerequisites)
* 호스트 OS: Ubuntu 22.04 LTS
* 로보마스터와 호스트 PC가 동일한 네트워크(STA 모드) 또는 Direct 연결(AP 모드) 상태.
* 기존의 복잡한 `robomaster_ros` 패키지 대신, ROS 2 노드 내에서 SDK를 직접 호출하는 방식 채택.

## 2. Docker 환경 설정 (Environment Setup)

### 2.1 Docker 설치 및 권한 설정
```bash
sudo apt update && sudo apt install docker.io -y
sudo usermod -aG docker $USER
newgrp docker
```

### 2.2 Docker 이미지 빌드 및 실행
작업 디렉토리(`~/robomaster_project`)에서 실행합니다.
```bash
# 이미지 빌드 (Dockerfile이 있는 위치에서)
docker build -t rm_foxy .

# 컨테이너 실행 (네트워크 및 볼륨 공유)
docker run -it --name rm_container --net=host \
  -v ~/robomaster_project:/root/robomaster_project \
  rm_foxy
```

## 3. ROS 2 워크스페이스 구축 (Workspace Setup)

### 3.1 패키지 생성 (컨테이너 내부)
```bash
mkdir -p /root/robomaster_project/ros2_ws/src
cd /root/robomaster_project/ros2_ws/src
ros2 pkg create --build-type ament_python my_robomaster --dependencies rclpy geometry_msgs
```

### 3.2 제어 소스 코드 작성 (`basic_move.py`)
`ros2_ws/src/my_robomaster/my_robomaster/basic_move.py` 파일 생성:
```python
import rclpy
from rclpy.node import Node
from robomaster import robot

class BasicMove(Node):
    def __init__(self):
        super().__init__('basic_move')
        self.get_logger().info('RoboMaster 연결 시도 중...')
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="sta") 
        
        self.get_logger().info('연결 성공! 1m 전진을 시작합니다.')
        self.chassis = self.ep_robot.chassis
        self.chassis.move(x=1, y=0, z=0, v=0.5).wait_for_completed()

    def destroy_node(self):
        self.ep_robot.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BasicMove()
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 3.3 실행 파일 등록 (`setup.py`)
`entry_points` 섹션 수정:
```python
    entry_points={
        'console_scripts': [
            'basic_move = my_robomaster.basic_move:main'
        ],
    },
```

## 4. 빌드 및 실행 (Build & Run)
```bash
cd /root/robomaster_project/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select my_robomaster --symlink-install
source install/setup.bash
ros2 run my_robomaster basic_move
```
