# 📘 [Manual] Unity-ROS2 Digital Twin 통신 환경 구축 
## 1. 서론 (Introduction)
#### 🔍 프로젝트 목적
현실의 로보마스터(ROS2 Foxy 기반)와 가상의 유니티(Unity) 환경을 실시간으로 연결하여, 로봇의 센서 데이터를 시각화하고 제어 명령을 전달하는 안정적인 데이터 파이프라인을 형성합니다.
#### 📡 통신 방식: Socket.io (WebSocket)
- 방식: 웹 표준 WebSocket을 기반으로 한 이벤트 기반 통신.
- 이유: ROS-TCP의 의존성 문제 해결, 자동 재연결, 쉬운 구현.
- 구성: 라즈베리파이(Server) ↔ 유니티(Client).
## 2. 서버 환경 진입 (Raspberry Pi & Docker)
### 2.1 SSH 원격 접속
노트북 터미널에서 호스트 이름을 사용하여 접속합니다.
```bash
ssh ubuntu@ubuntu.local
```

### 2.2 도커(Docker) 컨테이너 진입
실제 프로젝트가 구동되는 ros-foxy 컨테이너 내부로 접속합니다.
```bash
# 컨테이너 터미널 접속 (컨테이너 이름: ros-foxy)
docker exec -it ros-foxy /bin/bash

# 프로젝트 작업 디렉토리로 이동
cd /ros2_ws/src/my_robomaster
```
## 3. 필수 라이브러리 설치 (Setup)
통신 서버 구동을 위해 컨테이너 내부(root 환경)에서 아래 패키지들을 설치합니다.
```bash
# Socket.io 서버 구동을 위한 파이썬 라이브러리 설치
pip3 install python-socketio eventlet Flask-SocketIO
```
## 4. 서버 코드 작성 및 실행 (Server-side)
### 4.1 서버 파일 생성 (socket_server.py)
```bash
cat <<EOF > socket_server.py
import socketio
import eventlet

sio = socketio.Server(cors_allowed_origins='*')
app = socketio.WSGIApp(sio)

@sio.event
def connect(sid, environ):
    print(f"📡 [접속] 유니티 연결 성공! ID: {sid}")
    sio.emit('chat', {'msg': '라즈베리파이(Foxy) 서버 접속 완료!'})

@sio.event
def disconnect(sid):
    print(f"❌ [해제] 연결 종료 ID: {sid}")

if __name__ == '__main__':
    print("🚀 Socket.io 서버 시작 (Port: 3001)")
    eventlet.wsgi.server(eventlet.listen(('0.0.0.0', 3001)), app)
EOF
```
### 4.2 서버 실행
```bash
python3 socket_server.py
```
## 5. 유니티 클라이언트 설정 (Client-side)
### 5.1 라이브러리 설치
유니티 Window > Package Manager > + 클릭 > Add package from git URL
    URL: https://github.com/itisnajim/SocketIOUnity.git
### 5.2 통신 테스트용 스크립트 작성
유니티 내 Assets/Scripts/Network/SocketController.cs 생성
#### 주의: Uri의 IP 주소를 본인의 라즈베리파이 IP(hostname -I로 확인)로 수정하세요.
```c#

using UnityEngine;
using SocketIOUnity;
using System;

public class SocketController : MonoBehaviour
{
    public SocketIOUnity socket;

    void Start()
    {
        // 서버 주소 (라즈베리파이 IP로 수정 필수)
        var uri = new Uri("http://192.168.50.130:3001");
        socket = new SocketIOUnity(uri, new SocketIOOptions {
            Transport = SocketIOClient.Transport.TransportProtocol.WebSocket
        });

        socket.Connect();
        socket.OnConnected += (sender, e) => Debug.Log("<color=green>[Success]</color> Foxy 서버 연결 성공!");
        socket.On("chat", (data) => Debug.Log("수신 데이터: " + data));
    }
}
```
## 6. 최종 연결 테스트

    라즈베리파이: python3 socket_server.py 실행 확인.

    유니티: Play(▶) 클릭.

    콘솔에 "Foxy 서버 연결 성공!" 메시지 확인.
