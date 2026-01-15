import socket
import os

# 환경 변수에서 IP 가져오기 (docker-compose 설정 연동)
SERVER_IP = os.getenv('SERVER_IP', '192.168.50.39')
ROBOT_IP = os.getenv('ROBOT_IP', '192.168.50.31')  # .31로 일치시킴
UDP_PORTS = [40927, 40928, 40929]

def start_relay():
    sockets = []
    for port in UDP_PORTS:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 호스트 모드이므로 모든 인터페이스에서 해당 포트를 점유
            sock.bind(('0.0.0.0', port))
            sock.setblocking(False)
            sockets.append((sock, port))
            print(f"📡 [Relay] Port {port} 리스닝 중...")
        except Exception as e:
            print(f"❌ 포트 {port} 바인딩 실패: {e}")

    print(f"🚀 Relay 가동: 로봇({ROBOT_IP}) <--> 서버({SERVER_IP})")

    while True:
        for sock, port in sockets:
            try:
                data, addr = sock.recvfrom(4096)
                sender_ip = addr[0]
                
                # 1. 로봇에서 온 데이터 -> 서버로 토스
                if sender_ip == ROBOT_IP:
                    sock.sendto(data, (SERVER_IP, port))
                    # print(f"DEBUG: Robot -> Server (Port {port})") # 확인용 로그
                
                # 2. 서버에서 온 명령 -> 로봇으로 토스
                elif sender_ip == SERVER_IP:
                    sock.sendto(data, (ROBOT_IP, port))
                    # print(f"DEBUG: Server -> Robot (Port {port})")
                
                # 3. 만약 모르는 IP에서 온다면? (디버깅용)
                else:
                    print(f"⚠️ Unknown Packet from {sender_ip} on port {port}")

            except BlockingIOError:
                continue
            except Exception as e:
                print(f"Error during relaying: {e}")

if __name__ == "__main__":
    start_relay()