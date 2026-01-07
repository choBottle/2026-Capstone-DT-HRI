# [Manual] Docker 기반 ROS 2 Foxy & RoboMaster SDK 환경 구축

이 매뉴얼은 Ubuntu 22.04(Humble) 환경에서 RoboMaster SDK와의 호환성 문제를 해결하기 위해 **Docker를 이용한 ROS 2 Foxy 환경 구축** 및 **순수 SDK 제어 노드 작성** 과정을 설명합니다.

## 1. 전제 조건 (Prerequisites)
* 호스트 OS: Ubuntu 22.04 LTS
* 로보마스터와 호스트 PC가 동일한 네트워크(STA 모드) 또는 Direct 연결(AP 모드) 상태.
* 기존의 복잡한 `robomaster_ros` 패키지 대신, ROS 2 노드 내에서 SDK를 직접 호출하는 방식 채택.

## 1.1 추가 내용
* 로보마스터의 카메라 연결을 위한 가이드 내용으로 변경함. 과정의 2.1 까지는 동일함.
* 일반적으로는 cv2.imshow 함수를 사용하나, 도커 컨테이너는 모니터가 없는 상태이므로 imshow(창 띄우기 명령)시 에러 발생. 따라서 'X11 Forwarding' 기술을 사용해 도커 안에서 실행한 프로그램의 화면을 노트북 모니터로 전송하는 방법을 이용함.

## 2. Docker 환경 설정 (Environment Setup)

### 2.1 Docker 설치 및 권한 설정
```bash
sudo apt update && sudo apt install docker.io -y
sudo usermod -aG docker $USER
newgrp docker
```

### 2.2 화면 권한 허용하기 (Host)
도커가 모니터에 화면을 띄울 수 있도록 권한을 허용해줘야 함.
터미널 (user1@... 와 같이 도커 안이 아닌 바깥)에서 아래 명령어 입력.

```
xhost +local:docker
```

non-network local connections being added to access control list 라는 메세지가 나오면 성공.

### 2.3 Docker 설치 및 권한 설정
새로운 작업 디렉토리(~/robomaster_camera)에 아래 내용으로 Dockerfile을 생성.
다른 작업과 구분하기 위해 robomaster_camera 디렉토리로 변경하였음.

```
FROM osrf/ros:foxy-desktop

# 1. 필수 도구 설치
# libgl1-mesa-glx: OpenCV가 창을 띄울 때 필요한 그래픽 라이브러리 (이거 없으면 에러 남)
# nano: 파일 수정용 에디터
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions \
    git wget unzip nano libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# 2. RoboMaster SDK 및 OpenCV 설치
RUN pip3 install --upgrade pip
# setuptools 버전은 사용자 검증 버전(58.2.0) 유지 + opencv-python 추가
RUN pip3 install robomaster setuptools==58.2.0 opencv-python

# 3. 작업 경로 설정
WORKDIR /root/robomaster_camera

# 4. ROS 환경 자동 로드
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```


### 2.4 Docker 이미지 빌드 및 실행
작업 디렉토리(`~/robomaster_camera`)에서 실행합니다.
```bash
# 이미지 빌드. Dockerfile이 있는 위치에서 실행.
cd ~/robomaster_camera
docker build -t rm_cam_image .

# 컨테이너 실행 (네트워크 및 볼륨 공유)
docker run -it --net=host --privileged \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v ~/robomaster_camera:/root/robomaster_camera \
  --name rm_camera_container \
  rm_cam_image
```

### 2.5 테스트 코드 작성
도커 터미널(root@...)에서 바로 파일을 만들어서 테스트 함.
```
nano camera_test.py
```

```
import cv2
from robomaster import robot
import time

def main():
    print("1. 로봇 연결을 시도합니다... (AP 모드)")
    ep_robot = robot.Robot()

    # 로봇 와이파이에 직접 연결된 경우 "ap" 사용
    ep_robot.initialize(conn_type="ap")
    print(">>> 연결 성공!")

    print("2. 카메라를 켭니다.")
    ep_robot.camera.start_video_stream(display=False, resolution='720p')

    print("3. 화면 출력을 시작합니다. (종료하려면 화면 클릭 후 'q' 누르세요)")
    try:
        while True:
            # 로봇에게서 가장 최신 이미지 한 장 가져오기
            img = ep_robot.camera.read_cv2_image(strategy="newest")

            if img is not None:
                # 도커에서 내 노트북으로 화면 쏘기!
                cv2.imshow("RoboMaster Camera Test", img)

                # 'q' 키를 누르면 종료
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        ep_robot.camera.stop_video_stream()
        ep_robot.close()
        cv2.destroyAllWindows()
        print("종료되었습니다.")

if __name__ == '__main__':
    main()
```

이후 명령어로 테스트.

```
python3 camera_test.py
```

로봇의 실시간 영상이 새로운 팝업 창으로 나온다면 성공.

## 3. ROS 2 워크스페이스 구축 (Workspace Setup)

### 3.1 패키지 생성 (컨테이너 내부)
```bash
mkdir -p /root/robomaster_camera/ros2_ws/src
cd /root/robomaster_camera/ros2_ws/src
ros2 pkg create --build-type ament_python my_robomaster --dependencies rclpy geometry_msgs sensor_msgs cv_bridge
```

이후 아래 명령어를 실행하여 코드 생성.

```
cd /root/robomaster_camera/ros2_ws/src/my_robomaster/my_robomaster
nano camera_driver.py
```

### 3.2 제어 소스 코드 작성 (`camera_driver.py`)
`ros2_ws/src/my_robomaster/my_robomaster/camera_driver.py` 파일 생성:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from robomaster import robot
import cv2
import time

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.get_logger().info('RoboMaster 카메라 노드 시작 (AP 모드)...')

        # 1. 로봇 연결 및 카메라 초기화
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="ap")
        self.ep_robot.camera.start_video_stream(display=False, resolution='720p')
        self.get_logger().info('>>> 로봇 연결 성공! 카메라 데이터를 ROS로 송출합니다.')

        # 2. ROS Publisher 설정 (채널명: camera/raw)
        self.publisher_ = self.create_publisher(Image, 'camera/raw', 10)
        self.bridge = CvBridge()

        # 3. 타이머 (30fps)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        img = self.ep_robot.camera.read_cv2_image(strategy="newest")

        if img is not None:
            # [기능 1] 내 노트북 화면에 띄우기 (확인용)
            cv2.imshow("ROS 2 Camera Driver", img)
            cv2.waitKey(1)

            # [기능 2] ROS 네트워크로 송출 (다른 노드용)
            try:
                ros_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                ros_msg.header.stamp = self.get_clock().now().to_msg()
                ros_msg.header.frame_id = "camera_optical_frame"
                self.publisher_.publish(ros_msg)
            except Exception as e:
                self.get_logger().error(f'변환 에러: {e}')

    def destroy_node(self):
        self.ep_robot.camera.stop_video_stream()
        self.ep_robot.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 3.3 실행 파일 등록 (`setup.py`)
먼저 setup.py를 열어야 함.

```
cd /root/robomaster_camera/ros2_ws/src/my_robomaster
nano setup.py
```

src/my_robomaster/setup.py 파일의 entry_points 섹션을 다음과 같이 수정합니다.
`entry_points` 섹션 수정:
```python
    entry_points={
        'console_scripts': [
            'camera_driver = my_robomaster.camera_driver:main',
        ],
    },
```

## 4. 빌드 및 실행 (Build & Run)
### 4.1 패키지 빌드
```bash
# 빌드
cd /root/robomaster_camera/ros2_ws
colcon build --symlink-install

# 설정 로드
source install/setup.bash

```
### 4.2 노드 실행
```bash
# 로봇이 켜져 있고 네트워크에 연결된 상태에서 실행
ros2 run my_robomaster camera_driver
```

이후 ROS 2 Camera Driver 창이 뜨면서 로봇의 카메라 화면이 나오면 성공.
터미널에서 Ctrl+C를 눌러 종료.

## 5. 트러블슈팅 (Troubleshooting) (5번은 변경사항 없음)
1. No executable found: setup.py 수정 후 반드시 colcon build를 다시 수행하고 source install/setup.bash를 실행해야 함.

2. Connection Timeout: 컨테이너 실행 시 --net=host 옵션이 빠졌는지 확인하고, 로봇의 IP와 호스트 PC의 IP 대역이 일치하는지 점검.

3. ModuleNotFoundError: Docker 내부에서 pip install robomaster가 정상적으로 완료되었는지 확인.
