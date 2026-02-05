import os, socket, requests, time, psutil

# --- 설정 보정 ---
# 내부 DNS 이름 사용 (포트는 5000)
HUB_URL = os.getenv("HUB_URL", "http://central-hub-service.default.svc.cluster.local:5000/detect")
# K8s 배포 시 env에서 NODE_NAME을 주입받아야 함
NODE_ID = os.getenv("NODE_ID", os.getenv("HOSTNAME", "pi-unit-unknown"))
LISTEN_PORT = 40927

last_reports = {}
REPORT_INTERVAL = 30 # 동일 로봇 중복 보고 방지 (하트비트 주기)

def get_node_health():
    try:
        return {"cpu": psutil.cpu_percent(), "mem": psutil.virtual_memory().percent}
    except:
        return {"cpu": 0, "mem": 0}

def start_detector():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) 
    
    try:
        sock.bind(('0.0.0.0', LISTEN_PORT))
        print(f"🚀 [{NODE_ID}] Detector Active (Port: {LISTEN_PORT})")
    except Exception as e:
        print(f"❌ 소켓 바인딩 실패: {e}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            robot_ip = addr[0]
            current_time = time.time()

            # 중복 보고 방지 로직
            if robot_ip in last_reports:
                if current_time - last_reports[robot_ip] < REPORT_INTERVAL:
                    continue

            raw_msg = data.decode('utf-8', errors='ignore').replace('\x00', '').strip()
            
            if raw_msg:
                print(f"📡 신호 포착! IP: {robot_ip} | SN: {raw_msg}")
                payload = {
                    "robot_type": "ep01",
                    "ip": robot_ip,
                    "node_id": NODE_ID,
                    "node_health": get_node_health(),
                    "raw_data": raw_msg 
                }

                try:
                    # 404 에러를 방지하기 위해 정확한 URL로 POST
                    response = requests.post(HUB_URL, json=payload, timeout=2)
                    
                    if response.status_code in [200, 201]:
                        print(f"✅ 허브 보고 성공! (IP: {robot_ip})")
                        last_reports[robot_ip] = current_time
                    else:
                        print(f"❌ 허브 보고 실패 (Status: {response.status_code}) - URL: {HUB_URL}")
                        
                except Exception as e:
                    print(f"🔗 허브 연결 불가: {e}")
            
        except Exception as e:
            print(f"⚠️ 에러 발생: {e}")
            time.sleep(1)

if __name__ == "__main__":
    start_detector()