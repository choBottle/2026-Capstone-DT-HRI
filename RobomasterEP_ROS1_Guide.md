```markdown
# ROS & Docker를 활용한 RoboMaster EP 제어 가이드

[cite_start]이 문서는 **Ubuntu 22.04 (ROS 2 환경)**에서 **Docker**를 사용하여 호환성 문제 없이 **ROS 1 (Noetic)** 환경을 구축하고, **RoboMaster EP**를 안전하게 제어(Watchdog 포함)하기 위한 전체 절차를 다룹니다. [cite: 1]

---

## 1단계: 도커(Docker) 설치 및 설정 (Host PC)

[cite_start]우분투 노트북(Host)에 도커를 설치합니다. [cite: 2]

1. [cite_start]**도커 설치 및 권한 부여:** [cite: 3]
   ```bash
   # 터미널 실행 (Ctrl+Alt+T)
   sudo apt update
   sudo apt install docker.io -y
   
   # 현재 사용자에게 도커 권한 부여 (sudo 없이 쓰기 위해)
   sudo usermod -aG docker $USER

```

2. **권한 적용:**
로그아웃 후 다시 로그인하거나, 아래 명령어로 그룹 권한을 즉시 적용합니다. 


```bash
newgrp docker

```






## 2단계: ROS 컨테이너 생성 및 초기 세팅

ROS Noetic이 설치된 가상 환경(컨테이너)을 만들고 필수 라이브러리를 설치합니다. 

1. 
**컨테이너 생성 및 접속:** `--net=host`: 노트북의 Wi-Fi를 공유하기 위한 필수 옵션입니다. 


```bash
docker run -it --net=host --name roboros osrf/ros:noetic-desktop-full bash

```


(이제 터미널 프롬프트가 `root@...`로 바뀝니다. 여기는 도커 내부입니다.) 


2. 
**필수 도구 및 라이브러리 설치:** 


```bash
# 1. 패키지 목록 갱신 및 에디터/pip 설치
apt update
apt install python3-pip nano -y

# 2. 호환성 문제 해결을 위한 라이브러리 강제 업데이트 (중요!)
pip3 install --upgrade pip setuptools wheel
pip3 install --upgrade pillow
pip3 install --upgrade --ignore-installed numpy

# 3. RoboMaster SDK 및 키보드 제어 패키지 설치
pip3 install robomaster opencv-python
apt install ros-noetic-teleop-twist-keyboard -y

```



## 3단계: ROS 작업 공간 및 드라이버 작성

로봇과 ROS를 연결하는 '브릿지 프로그램'을 만듭니다. 

1. 
**워크스페이스 및 패키지 생성:** 


```bash
mkdir -p /root/catkin_ws/src
cd /root/catkin_ws/src
catkin_create_pkg robomaster_driver std_msgs rospy geometry_msgs

```


2. 
**드라이버 코드 작성 (driver.py):** 


```bash
cd /root/catkin_ws/src/robomaster_driver/src
nano driver.py

```


아래 코드를 복사해서 붙여넣으세요. **(안전장치 포함 버전)** 


```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import robomaster
from robomaster import robot
import time

ep_robot = None
chassis = None
last_cmd_time = 0  # 마지막 명령 수신 시간

def cmd_vel_callback(msg):
    global last_cmd_time
    last_cmd_time = time.time()  # 시간 갱신

    # ROS 속도(m/s, rad/s) -> 로보마스터 속도(m/s, deg/s) 변환
    x_val = msg.linear.x
    y_val = msg.linear.y
    z_val = msg.angular.z * 57.2958

    if chassis:
        chassis.drive_speed(x=x_val, y=y_val, z=z_val)

def main():
    global ep_robot, chassis, last_cmd_time
    rospy.init_node('robomaster_driver')

    try:
        # 로봇 연결 (공유기 모드)
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="sta")
        chassis = ep_robot.chassis
        rospy.loginfo(">>> Robot Connected! Safe Drive Mode ON.")

        rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

        last_cmd_time = time.time()
        rate = rospy.Rate(10) # 0.1초마다 검사

        while not rospy.is_shutdown():
            # 0.5초 이상 명령이 없으면 강제 정지 (안전장치)
            if time.time() - last_cmd_time > 0.5:
                chassis.drive_speed(x=0, y=0, z=0)
            rate.sleep()

    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        if ep_robot:
            ep_robot.close()

if __name__ == '__main__':
    main()

```


(저장: `Ctrl+O` -> `Enter` -> `Ctrl+X`) 


3. 
**빌드 및 권한 설정:** 


```bash
chmod +x driver.py
cd /root/catkin_ws
catkin_make
source devel/setup.bash

```



## 4단계: 실행 (3개의 터미널 활용)

가장 중요한 단계입니다. 총 3개의 터미널을 열고 순서대로 실행합니다. *(전제 조건: 로봇과 노트북이 동일한 Wi-Fi에 연결되어 있어야 함)* 

### 터미널 1: ROS 마스터 (roscore)

ROS의 중추 신경입니다. 항상 켜져 있어야 합니다. 

1. 새 터미널 열기 (`Ctrl+Alt+T`)
2. 입력:
```bash
docker exec -it roboros bash
source /root/catkin_ws/devel/setup.bash
roscore

```



### 터미널 2: 로봇 드라이버 (driver.py)

로봇과 연결을 담당합니다. 

1. 새 터미널 열기 


2. 입력:
```bash
docker exec -it roboros bash
source /root/catkin_ws/devel/setup.bash
rosrun robomaster_driver driver.py

```


3. 성공 확인: `>>> Robot Connected!` 메시지가 떠야 함. 



### 터미널 3: 키보드 컨트롤러 (teleop)

사용자의 입력을 받는 곳입니다. 

1. 새 터미널 열기
2. 입력:
```bash
docker exec -it roboros bash
source /root/catkin_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

```

## 5단계: 조작 방법

**터미널 3을 클릭해서 활성화한 상태**에서 키보드를 누릅니다. 

* `i`: 전진
* `, (콤마)`: 후진
* `j`: 좌회전 (제자리)
* `l`: 우회전 (제자리)
* `Shift + j`: 왼쪽으로 게걸음 (메카넘 휠)
* `Shift + l`: 오른쪽으로 게걸음
* `k`: 정지 (또는 키보드에서 손을 떼면 0.5초 뒤 자동 정지)
* 
`Ctrl + C`: 프로그램 종료 



> **💡 나중에 다시 실행하려면? (재부팅 후)**
> 컴퓨터를 껐다 켜면 컨테이너가 꺼져 있습니다. 다시 시작하는 법입니다. 
> 
> 
> 1. **컨테이너 깨우기:**
> ```bash
> docker start roboros
> 
> ```
> 
> 
> 
> 
> 2. 그 후 **4단계(실행)**부터 똑같이 터미널 3개를 열어 진행하면 됩니다. 
> 
> 

```

```