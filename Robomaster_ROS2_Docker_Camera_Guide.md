# [Manual] Docker 기반 ROS 2 Foxy & RoboMaster SDK 환경 구축

이 매뉴얼은 Ubuntu 22.04(Humble) 환경에서 RoboMaster SDK와의 호환성 문제를 해결하기 위해 **Docker를 이용한 ROS 2 Foxy 환경 구축** 및 **순수 SDK 제어 노드 작성** 과정을 설명합니다.

## 1. 전제 조건 (Prerequisites)
* 호스트 OS: Ubuntu 22.04 LTS
* 로보마스터와 호스트 PC가 동일한 네트워크(STA 모드) 또는 Direct 연결(AP 모드) 상태.
* 기존의 복잡한 `robomaster_ros` 패키지 대신, ROS 2 노드 내에서 SDK를 직접 호출하는 방식 채택.

## 1.1 추가 내용
* 로보마스터의 카메라 연결을 위한 가이드 내용으로 변경함. 과정의 2.1 까지는 동일함.
* 일반적으로는 cv2.imshow 함수를 사용하나, 도커 컨테이너는 모니터가 없는 상태이므로 imshow(창 띄우기 명령)시 에러 발생. 따라서 'X11 Forwarding' 기술을 사용해 도커 안에서 실행한 프로그램의 화면을 노트북 모니터로 전송하는 방법을 이용함.
* STA 모드와 AP 모드 둘 다 지원. STA 모드의 경우 보마스터와 호스트 PC가 동일한 와이파이(공유기)에 연결되어 있어야 하며
  로보마스터가 미리 해당 와이파이에 연결되어 IP를 할당받은 상태여야 함.

## 1.2 로봇 네트워크 설정 (STA 모드)
코드를 실행하기 전, 로봇을 공유기(Router)에 연결해야 합니다. (이 과정은 Windows PC나 모바일 앱에서 최초 1회 수행해야 합니다.)

1. 로봇의 전원을 켜고, 인텔리전트 컨트롤러의 스위치를 공유기 연결 모드로 전환합니다.
2. 공식 RoboMaster 앱 (PC 또는 모바일)을 실행 후 연결 > 연결 모드 메뉴로 진입하여 공유기 연결 모드를 선택.
3. 와이파이 정보(SSID, PW)를 입력하고 로봇을 연결합니다.
4. 연결 성공 후, 설정 > 시스템 > 네트워크 정보에서 로봇의 IP 주소 (예: 192.168.xxx.xxx)를 확인합니다. (핑 테스트 시 필요)
5. 연결이 완료되면 공식 로보마스터 앱(PC 또는 모바일)은 종료.


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

터미널을 새로 실행할때마다 해당 명령어 입력 필요.

```
xhost +local:docker
```

non-network local connections being added to access control list 라는 메세지가 나오면 성공.

#### 매번 자동으로 실행시키고 싶은 경우

1. 터미널을 열고 편집기로 설정 파일을 열기.
```
nano ~/.bashrc
```

2. 파일의 맨 마지막 줄로 내려가서(화살표 키 이용), 아래 명령어를 그대로 복사해서 붙여넣기.
```
# Docker 화면 권한 자동 허용
xhost +local:docker > /dev/null 2>&1
```

3. Ctrl+O로 저장, Enter를 누른 뒤 Ctrl+X로 나오기.
4. 터미널을 껐다 켜거나, 아래 명령어를 한 번 입력하면 바로 적용됨. 터미널을 새로 켜도 xhost 명령어를 칠 필요가 없음.
```
source ~/.bashrc
```

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

### 2.5 테스트 코드 작성 (AT 연결 모드)
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

### 2.5 테스트 코드 작성 (STA 연결 모드)
```
nano camera_test.py
```

```
import cv2
from robomaster import robot
import time

def main():
    print("1. 로봇 연결을 시도합니다... (STA 모드)")
    ep_robot = robot.Robot()

    # [수정됨] AP 모드 대신 STA(공유기) 모드로 연결
    try:
        ep_robot.initialize(conn_type="sta")
        print(f">>> 연결 성공! 로봇 IP: {ep_robot.ip}")
    except Exception as e:
        print(f"!!! 연결 실패: {e}")
        print("로봇과 노트북이 같은 와이파이에 있는지 확인하세요.")
        return

    print("2. 카메라를 켭니다.")
    ep_robot.camera.start_video_stream(display=False, resolution='720p')

    print("3. 화면 출력을 시작합니다. (종료하려면 화면 클릭 후 'q' 누르세요)")
    try:
        while True:
            img = ep_robot.camera.read_cv2_image(strategy="newest")
            if img is not None:
                cv2.imshow("RoboMaster Camera Test", img)
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

### 3.2 제어 소스 코드 작성 (`camera_driver.py`. AT 연결 모드)
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

### 3.2 제어 소스 코드 작성 (`camera_driver.py`. STA 연결 모드)
`ros2_ws/src/my_robomaster/my_robomaster/camera_driver.py` 파일 생성:

```
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
        self.get_logger().info('RoboMaster 카메라 노드 시작 (STA 모드)...')

        # 1. 로봇 연결 (STA 모드)
        self.ep_robot = robot.Robot()
        try:
            # [수정됨] conn_type="sta"로 변경
            self.ep_robot.initialize(conn_type="sta")
            self.get_logger().info(f'>>> 로봇 연결 성공! (IP: {self.ep_robot.ip})')
        except Exception as e:
            self.get_logger().error(f'로봇 연결 실패: {e}')
            return

        # 카메라 스트림 시작
        self.ep_robot.camera.start_video_stream(display=False, resolution='720p')
        self.get_logger().info('>>> 카메라 데이터를 ROS로 송출합니다.')

        # 2. ROS Publisher 설정 (채널명: camera/raw)
        self.publisher_ = self.create_publisher(Image, 'camera/raw', 10)
        self.bridge = CvBridge()

        # 3. 타이머 (30fps)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        img = self.ep_robot.camera.read_cv2_image(strategy="newest")

        if img is not None:
            # [기능 1] 내 노트북 화면에 띄우기 (확인용)
            # [옵션] 서버/도커 환경이라 창 띄우기가 힘들다면 아래 두 줄 주석 처리
            # cv2.imshow("ROS 2 Camera Driver", img)
            # cv2.waitKey(1)

            # [기능 2] ROS 네트워크로 송출 (다른 노드용)
            try:
                ros_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                ros_msg.header.stamp = self.get_clock().now().to_msg()
                ros_msg.header.frame_id = "camera_optical_frame"
                self.publisher_.publish(ros_msg)
            except Exception as e:
                self.get_logger().error(f'변환 에러: {e}')

    def destroy_node(self):
        try:
            self.ep_robot.camera.stop_video_stream()
            self.ep_robot.close()
        except:
            pass
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

### 5. 카메라 영상 저장 코드 작성 (video_recorder.py)
로봇에서 송출하는 영상 데이터(camera/raw)를 수신하여 .avi 파일로 저장하는 별도의 노드를 생성하는 코드를 작성함.

1. 파일 생성
```
cd /root/robomaster_camera/ros2_ws/src/my_robomaster/my_robomaster
nano video_recorder.py
```

2. 코드 작성 (날짜/시간 기반으로 파일명을 자동 생성하여 덮어쓰기를 방지함)
```
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.get_logger().info('영상 저장 노드가 시작되었습니다.')

        # 1. 구독 설정 ('camera/raw' 토픽 수신)
        self.subscription = self.create_subscription(
            Image,
            'camera/raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        # 2. 비디오 저장 경로 설정 (/root/robomaster_camera -> 호스트 ~/robomaster_camera 와 연동됨)
        self.save_dir = '/root/robomaster_camera/saved_videos'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # 3. 파일명 및 코덱 설정
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.save_dir, f"robot_video_{timestamp}.avi")
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = None
        self.is_recording = False
        self.get_logger().info(f'녹화 대기 중... 파일명: {self.filename}')

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 첫 프레임 수신 시 VideoWriter 초기화
            if not self.is_recording:
                height, width, _ = frame.shape
                self.out = cv2.VideoWriter(self.filename, self.fourcc, 30.0, (width, height))
                self.is_recording = True
                self.get_logger().info('>>> 녹화 시작!')

            if self.out is not None:
                self.out.write(frame)

        except Exception as e:
            self.get_logger().error(f'저장 중 에러 발생: {e}')

    def destroy_node(self):
        if self.out is not None:
            self.out.release()
            self.get_logger().info(f'녹화 완료. 파일 저장됨: {self.filename}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

3. 실행 파일 등록 업데이트 (setup.py)
새로 만든 video_recorder 노드를 실행할 수 있도록 setup.py를 수정.

```
cd /root/robomaster_camera/ros2_ws/src/my_robomaster
nano setup.py
```

console_scripts 섹션에 video_recorder 라인을 추가.
```
entry_points={
        'console_scripts': [
            'camera_driver = my_robomaster.camera_driver:main',
            'video_recorder = my_robomaster.video_recorder:main', # <-- 이 줄 추가됨
        ],
    },
```

4. 빌드 후 실행
코드가 추가되었으므로, 다시 빌드 필요.
```
cd /root/robomaster_camera/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

실행 시 2개의 터미널이 필요함.
로봇의 영상을 송출하는 camera_driver와 이를 받아서 저장하는 video_recorder를 동시에 실행하기 위함.

**터미널 1 (송신용): Camera Driver 실행 (이미 열려있는 도커 터미널에서 실행) -> 로봇과 연결되고 영상 송출이 시작.**
```
ros2 run my_robomaster camera_driver
```

**터미널 2 (수신/녹화용): Video Recorder 실행 노트북에서 새 터미널을 열고 도커 내부에 접속하여 실행합니다.**
```
# 1. 도커 접속
docker exec -it rm_camera_container bash

# 2. 환경 설정 로드
source /root/robomaster_camera/ros2_ws/install/setup.bash

# 3. 녹화 노드 실행
ros2 run my_robomaster video_recorder
```

5. 결과 확인

녹화 종료: 원하는 만큼 녹화 후, 터미널 2에서 Ctrl+C를 눌러 종료.  
파일 확인: 호스트 PC(노트북)의 ~/robomaster_camera/saved_videos 폴더에 .avi 파일이 생성되었는지 확인.  
영상 재생: 생성된 파일을 실행하여 녹화가 정상적으로 되었는지 확인.  
이후 터미널 1에서도 Ctrl+C로 영상 데이터 송신 종료.

## 6. 트러블슈팅 (Troubleshooting) (4번 추가)
1. No executable found: setup.py 수정 후 반드시 colcon build를 다시 수행하고 source install/setup.bash를 실행해야 함.

2. Connection Timeout: 컨테이너 실행 시 --net=host 옵션이 빠졌는지 확인하고, 로봇의 IP와 호스트 PC의 IP 대역이 일치하는지 점검.

3. ModuleNotFoundError: Docker 내부에서 pip install robomaster가 정상적으로 완료되었는지 확인.

4. STA 모드 연결 실패 및 화면 출력 에러 : Connection Failed: 로봇과 노트북이 서로 다른 와이파이(SSID)에 연결되어 있는지 확인하세요. / cb / qt.qpa.plugin 에러: 도커 내부에서 창을 띄울 권한이 없어서 발생합니다.

해결 1: 호스트 터미널에서 xhost +local:docker 명령어 실행.
해결 2: camera_driver.py 코드 내의 cv2.imshow 부분을 주석 처리하여 창을 띄우지 않도록 설정.
