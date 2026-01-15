import eventlet
eventlet.monkey_patch()

import os
import json
import psycopg2
from flask import Flask, request
from flask_socketio import SocketIO, emit, join_room

app = Flask(__name__)
# async_mode='eventlet' 설정을 통해 실시간 통신 최적화
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# --- [DB 환경 설정: docker-compose와 일치됨] ---
DB_HOST = os.getenv('DB_HOST', 'db')
DB_NAME = os.getenv('DB_NAME', 'robot_db')
DB_USER = os.getenv('DB_USER', 'admin')      # admin으로 수정
DB_PASS = os.getenv('DB_PASS', 'password123') # password123으로 수정

def get_db_connection():
    try:
        conn = psycopg2.connect(
            host=DB_HOST,
            database=DB_NAME,
            user=DB_USER,
            password=DB_PASS,
            connect_timeout=5
        )
        return conn
    except Exception as e:
        print(f"❌ DB Connection Error: {e}")
        return None

# { relay_id: sid } 관리용
relays = {}

@socketio.on('connect')
def handle_connect():
    print(f"🔌 New connection: {request.sid}")

@socketio.on('register')
def handle_register(data):
    role = data.get('role')
    device_id = data.get('id')
    robot_type = data.get('robot_type')

    join_room(role)
    if robot_type:
        join_room(f"{robot_type}_worker")
    
    if role == 'relay':
        relays[device_id] = request.sid
    
    print(f"✅ Registered Role: {role} | ID: {device_id}")

# --- [Flow 1] Relay -> Hub (Raw Data) ---
@socketio.on('relay_to_hub')
def handle_relay_data(data):
    robot_type = "EP01"
    emit('process_raw', data, room=f"{robot_type}_worker")

# --- [Flow 2] Brain Worker -> Hub -> Viewers & DB ---
@socketio.on('worker_to_hub')
def handle_processed_data(data):
    # 1. 수신 로그 출력 (터미널 확인용)
    print(f"📥 [Hub] Received: {data.get('type')} data from {data.get('robot_id')}")
    
    # 2. 실시간 모니터링 클라이언트(Unity/Web)로 전송
    emit('update_data', data, room='viewer', broadcast=True)

    # 3. DB 저장 로직
    conn = get_db_connection()
    if conn:
        try:
            with conn.cursor() as cur:
                sql = """
                INSERT INTO robot_telemetry (robot_id, robot_type, data_type, payload)
                VALUES (%s, %s, %s, %s)
                """
                payload = json.dumps({"value": data.get('val')})
                cur.execute(sql, (
                    data.get('robot_id'),
                    data.get('robot_type', 'EP01'),
                    data.get('type'),
                    payload
                ))
            conn.commit()
            conn.close()
            # print("✅ [DB] Saved") # 너무 잦은 로그 방지를 위해 주석 처리 가능
        except Exception as e:
            print(f"❌ DB Insert Error: {e}")

# --- [Flow 3] 제어 명령 로직 ---
@socketio.on('command_request')
def handle_command_request(data):
    conn = get_db_connection()
    if conn:
        try:
            with conn.cursor() as cur:
                sql = "INSERT INTO robot_commands (robot_id, command, params, sender) VALUES (%s, %s, %s, %s)"
                cur.execute(sql, (
                    data.get('robot_id'),
                    data.get('action'),
                    json.dumps(data.get('params', {})),
                    "viewer"
                ))
            conn.commit()
            conn.close()
        except Exception as e:
            print(f"❌ DB Command Log Error: {e}")

    robot_type = data.get('robot_type')
    emit('generate_hex', data, room=f"{robot_type}_worker")

@socketio.on('hex_to_hub')
def handle_hex_payload(data):
    relay_sid = relays.get(data.get('relay_id'))
    if relay_sid:
        emit('send_to_robot', data, room=relay_sid)

@socketio.on('disconnect')
def handle_disconnect():
    print(f"❌ Disconnected: {request.sid}")

if __name__ == "__main__":
    port = int(os.getenv('PORT', 5000))
    socketio.run(app, host='0.0.0.0', port=port)