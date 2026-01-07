# RoboMaster ROS2 프로젝트 통합 가이드 (STA 모드)
본 가이드는 노트북에서 라즈베리파이로 프로젝트를 전송하고, STA 모드(공유기 연결) 환경에서 Docker를 통해 로봇을 제어하는 모든 과정을 설명합니다.
## 0단계: 라즈베리파이 접속 및 용량 점검 (필수 확인)
파일을 전송하기 전, 라즈베리파이에 충분한 여유 공간이 있는지 먼저 확인해야 합니다.
### 0.1.라즈베리파이 SSH 접속:
```bash
ssh [본인계정]@[호스트네임].local
# 예시: ssh pi@robot1.local
```

### 0.2.디스크 용량 확인:
```bash
df -h /
```

- 확인 사항: Avail(사용 가능 공간)이 최소 5GB 이상이어야 안전합니다. (도커 이미지 및 빌드 파일 용량 고려)
- 용량 부족 시: 불필요한 파일을 지우거나 sudo apt-get clean 명령어로 캐시를 삭제하세요.

### 1단계: 프로젝트 파일 전송 (노트북 → 라즈베리파이)
용량이 확인되었다면, 노트북 터미널에서 프로젝트 폴더를 전송합니다.

```bash
# 노트북 터미널에서 실행
cd ~/physical

# 형식: scp -r robomaster_project [본인계정]@[호스트네임].local:~/
scp -r robomaster_project pi@robot1.local:~/
```

## 2단계: Docker 환경 구축
파일 전송이 완료된 라즈베리파이에서 Docker를 설치하고 권한을 설정합니다.
### 2.1.Docker 설치 (최초 1회):
```bash
sudo apt-get update
sudo apt-get install -y docker.io
sudo systemctl enable --now docker

# 현재 사용자를 docker 그룹에 추가
sudo usermod -aG docker $USER
```
### 2.2.권한 적용: exit으로 접속을 종료한 후, 다시 SSH로 접속해야 Docker 명령어를 사용할 수 있습니다.
## 3단계: Docker 이미지 빌드
제공된 Dockerfile을 사용하여 로봇 제어 이미지를 생성합니다.
```bash
cd ~/robomaster_project
docker build -t rm_node .
```
라즈베리파이 3 기준 약 10분~15분이 소요됩니다.

## 4단계: 네트워크 및 STA 모드 설정 확인
### 4.1.네트워크: 로봇과 라즈베리파이가 동일한 WiFi에 연결되어 있는지 확인합니다.
### 4.2.코드 확인: ros2_ws/src/my_robomaster/my_robomaster/basic_move.py 파일 내 초기화 코드가 다음과 같은지 확인하세요.
```python
ep_robot.initialize(conn_type="sta")
```

## 5단계: 컨테이너 실행 및 노드 구동
도커 컨테이너 안에서 패키지를 빌드하고 실행합니다.
### 5.1.도커 컨테이너 실행:
```bash
docker run -it --rm \
  --net=host \
  -v $(pwd)/ros2_ws:/ros2_ws \
  rm_node /bin/bash
```
### 5.2.컨테이너 내부 작업 (프롬프트 변경 후):
```bash
# 워크스페이스 이동 및 환경 로드
cd /ros2_ws
source /opt/ros/foxy/setup.bash

# 패키지 빌드 (RPi 3 메모리 보호를 위해 병렬 작업 제한)
colcon build --packages-select my_robomaster --parallel-workers 1

# 환경 적용 및 실행
source install/setup.bash
ros2 run my_robomaster basic_move
```
