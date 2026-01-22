import socket
import requests
import time

# --- 설정 ---
# 노트북(Central Hub)의 서비스 주소와 포트 확인 필요
HUB_URL = "http://192.168.50.39:30005/detect"
NODE_ID = "pi-unit-01"
LISTEN_PORT = 40927

def start_detector():
    # UDP 소켓 설정
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind(('', LISTEN_PORT))
    except Exception as e:
        print(f"❌ 소켓 바인딩 실패: {e}")
        return
    
    print(f"=== [{NODE_ID}] 로봇 감지기 가동 중 (포트: {LISTEN_PORT}) ===")

    while True:
        try:
            # 1. 로봇의 브로드캐스트 신호 수신
            data, addr = sock.recvfrom(1024)
            robot_ip = addr[0]
            
            # 2. 데이터 정제 (가장 중요한 부분!)
            try:
                # UTF-8로 디코딩 후, 널 문자(\x00)와 공백을 완전히 제거
                # 이 과정이 없으면 쿠버네티스 파드 생성 시 StartError가 발생합니다.
                raw_msg = data.decode('utf-8', errors='ignore').replace('\x00', '').strip()
            except Exception as e:
                print(f"⚠️ 디코딩 에러: {e}")
                raw_msg = "UNKNOWN_ID"

            # 3. 신호 포착 로그 출력
            if raw_msg:
                print(f"📡 신호 포착! IP: {robot_ip} | Clean ID: {raw_msg}")

                # 4. 허브로 데이터 보고
                # 허브 서버의 Flask 코드가 받을 변수명(ip, raw_data)과 일치시킵니다.
                payload = {
                    "robot_type": "ep01",
                    "ip": robot_ip,
                    "node_id": NODE_ID,
                    "raw_data": raw_msg
                }

                try:
                    response = requests.post(HUB_URL, json=payload, timeout=3)
                    
                    if response.status_code == 200:
                        print(f"✅ 허브 보고 성공! (파드 생성 요청됨)")
                        # 중복 생성 방지를 위해 1분간 대기 (필요에 따라 조절)
                        print("💤 다음 감지까지 60초 대기 중...")
                        time.sleep(60)
                    else:
                        print(f"❌ 허브 보고 실패 (Status: {response.status_code})")
                except requests.exceptions.RequestException as e:
                    print(f"❌ 허브 연결 불가: {e}")
            
        except KeyboardInterrupt:
            print("\n👋 감지기를 종료합니다.")
            break
        except Exception as e:
            print(f"⚠️ 실행 중 오류 발생: {e}")
            time.sleep(1)
    
    sock.close()

if __name__ == "__main__":
    start_detector()
