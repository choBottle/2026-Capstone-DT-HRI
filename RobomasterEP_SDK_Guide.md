# SDK를 활용한 로보마스터 EP 작동

## [사전 환경 세팅]
* **Ubuntu 22.04 환경이 적용된 노트북** (이하 노트북)
* **VSCode** (Python 설치)
* **로봇과 노트북에 동일하게 연결되어있는 Wi-Fi 네트워크** (로봇에는 전용 앱을 통해 네트워크 연결시킨 후 앱 종료)

---

## [사전 준비]
> 이하 작업은 VSCode 내 Terminal에서 실행

### 1. Python 3.8 설치

**1) 시스템 업데이트 및 필수 도구 설치**
```bash
sudo apt update
sudo apt install software-properties-common -y
```

**2) 구버전 파이썬 저장소(deadsnakes PPA) 추가**
```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
```

**3) Python 3.8 및 개발 도구 설치 (가상환경 모듈 포함)**
```bash
sudo apt install python3.8 python3.8-venv python3.8-dev -y
```

### 2. 프로젝트 생성 및 가상환경 구성

**1) 프로젝트 폴더 생성 및 이동**
```bash
mkdir robomaster_project
cd robomaster_project
```

**2) Python 3.8을 지정하여 가상환경 생성 (중요!)**
> 반드시 `python3.8` 명령어를 사용해야 합니다.
```bash
/usr/bin/python3.8 -m venv venv
```

**3) 가상환경 활성화**
```bash
source venv/bin/activate
```

### 3. 필수 라이브러리 설치

**1) 설치 도구(pip) 최신화 (호환성 확보)**
```bash
pip install --upgrade pip setuptools wheel
```

**2) RoboMaster SDK 설치**
```bash
pip install robomaster
```

**3) OpenCV 설치 (카메라 스트리밍용)**
```bash
pip install opencv-python
```

---

## [로봇 네트워크 연결]
1. 로봇을 **공유기 모드**로 전환
2. 노트북과 로봇에 **같은 Wi-Fi 연결**

---

## [코드 작성 및 실행]

### 1. VSCode 설정 및 방화벽 해제
1. VScode 우측 하단 (Copilot 아이콘과 알림 아이콘 사이)의 **Python 버전을 확인하고 3.8.20으로 변경**
2. 방화벽 해제 명령어 입력:
   ```bash
   sudo ufw disable
   ```
3. 터미널을 유지한 채로 파이썬 코드 파일 생성:
   ```bash
   code main.py
   ```

### 2. main.py 코드 작성
아래 내용을 `main.py`에 붙여넣고 저장합니다.

```python
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
```

### 3. 실행 및 확인
터미널에서 다음 명령어로 실행합니다.
```bash
python main.py
```
---

## [재접속]

종료 후 재접속할 때 입력 순서
아래 명령어는 VSCode 내 터미널에서 입력해야 함

## 1. 프로젝트 폴더로 이동
```bash
cd ~/robomaster_project
```

## 2. 가상환경 활성화
```bash
source venv/bin/activate
```

## 3. 방화벽 끄기
```bash
sudo ufw disable
```

## 4. 코드 실행
```bash
python main.py
```
