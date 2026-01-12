#[Manual] 로봇-서버-DB 데이터 파이프라인 구축 및 연동

이 문서는 캡스톤 디자인 프로젝트의 일환으로 **로봇(Client) - 서버(Flask) - 데이터베이스(MySQL) 간의 양방향 통신 및 로그 저장 시스템** 구축 과정을 기술합니다.  

----------

## 0. 서론

먼저, 이번 프로젝트의 데이터는 성격에 따라 크게 세 가지로 나뉩니다. 이 성격에 따라 담아야 할 그릇(DB)이 달라집니다.

    **1. 관리 및 기록 데이터 (Management Data)**
    - 내용: 사용자 정보, 로봇의 종류(EP01, GO1), 실험 시나리오 설정, 실험이 언제 시작해서 언제 끝났는지에 대한 기록.
    - 특징: 데이터 양이 엄청나게 많지는 않지만, 관계(Relationship)가 중요합니다. (예: "철수가" "GO1 로봇으로" "3번 시나리오를" 돌렸다.)
    - 추천 DB: 관계형 데이터베이스 (RDBMS) - 이게 이번에 설계할 핵심입니다.

    **2. 초고속 시계열 데이터 (Time-Series Data)**
    - 내용: Odometry(20Hz), IMU(20Hz), Wheel Speed(10Hz). 1초에 수십 번씩 쏟아지는 센서 값.
    - 특징: 수정할 일이 없고 계속 쌓이기만 합니다. 나중에 "리플레이"를 위해 필요합니다.
    - 추천 DB: InfluxDB (이미 파일에서 선정되어 있음).

    **3. 실시간 상태/캐시 데이터 (Real-time Data)**
    - 내용: 예측 좌표, 현재 가상 장애물 위치.
    - 특징: 저장이 목적이 아니라, "지금 당장"의 위치를 유니티와 서버가 공유하는 게 목적입니다. 휘발되어도 괜찮습니다.
    - 추천 DB: Redis (이미 파일에서 선정되어 있음).


1번 데이터를 기록할 DB의 스키마를 구성할 예정임. 2번, 3번 데이터 기록용 DB는 추후 구성.

사용 DB 구조는 MySQL. 대중적이고 자료가 많은 DB인데다가 현재 초기 구축을 시도하는 상태에서 알맞은 구조라 판단함.


## 0.1 저장할 개체 정하기



    저장해야 할 개체(Entity) 정하기
    - Robot (로봇 정보): 우리 랩실에 있는 로봇들. (이름: EP01, GO1, IP주소, 상태 등)
    - User (사용자): 실험을 진행하는 학생이나 관리자.
    - Session (실험 기록): "누가, 언제, 어떤 로봇으로 실험을 시작했다"는 이력.
    - EventLog (사건 기록): "몇 시 몇 분에 충돌 발생함", "배터리 부족 경고" 등 중요 이벤트.

**스키마(ERD) 초안 작성**
```
2-1. Robot 테이블 (로봇 명부)
-robot_id (PK, 자동증가 숫자): 로봇 고유 번호
-model_type (문자): "EP01" 또는 "GO1" (어떤 기종인가?)
-ip_address (문자): "192.168.0.101" (통신할 IP)
-status (문자): "Active(활동중)", "Idle(대기)", "Charging(충전)"

2-2. User 테이블 (사용자)
-user_id (PK, 문자): 로그인 ID (예: "student2026")
-name (문자): 사용자 이름
-role (문자): "Admin", "Student"

2-3. Session 테이블 (실험 한 판의 기록)
-session_id (PK, 자동증가 숫자): 이번 실험 번호
-robot_id (FK): 누가(어떤 로봇이) 뛰었나?
-user_id (FK): 누가(어떤 사람이) 시켰나?
-start_time: 실험 시작 시간 (자동 기록)
-end_time: 실험 종료 시간 (끝날 때 업데이트)

2-4. EventLog 테이블 (중요 사건 사고)
-log_id (PK, 숫자): 고유 번호
-session_id (FK): 어느 실험 중에 일어난 일인가?
-event_type (문자): "Collision(충돌)", "Goal(도착)", "Error(에러)"
-message (문자): 상세 내용
-created_at (시간): 발생 시각

2-5. CommandLog 테이블 (명령 기록). 도중에 추가한 테이블.
-command_id (PK, 자동증가 숫자): 명령 번호
-session_id (FK): 몇 번 실험 중 내려진 명령인가?
-command_type (문자): "Velocity", "ModeChange"
-payload (JSON): {"linear": 0.5, "angular": 0.1} (실제 명령값)
-created_at: 명령 수신 시간

센서 데이터는 양이 너무 많아서 스키마 초안에선 제외. 이후 성공적 구동시 SensorLog 테이블 등 추가 가능.
```
----------

## 1. 구현 기능 요약 (Implementation Summary)

본 단계에서는 로봇의 상태 정보와 주행 중 발생하는 이벤트 로그를 원격 서버에 저장하는 백엔드 시스템을 구축하고 연동 테스트를 완료하였습니다.

- **DB 스키마 설계 및 구축**: 사용자, 로봇, 세션, 로그(이벤트/명령) 간의 관계형 데이터베이스(RDBMS) 구축.
- **API 서버 개발**: Flask 기반의 REST API를 통해 로봇의 상태 업데이트 및 로그 저장 요청 처리.
- **연동 검증**: 로봇(Client) -> 서버(Server) -> DB 데이터 흐름 확인 및 Foreign Key 제약 조건에 따른 데이터 무결성 검증 완료.

----------

## 2. 데이터베이스 구축 (Database Setup)

MySQL을 사용하여 로봇 데이터를 저장할 스키마를 정의했습니다. 데이터 무결성을 위해 Users와 Robots 테이블을 먼저 생성하고, 이를 참조하는 Sessions 및 Logs 테이블을 구성했습니다.

**2.0 MySql 데이터베이스 생성**

```
# [STEP 1] DB 프로젝트 폴더 만들기
# 홈 디렉토리에 폴더 생성
mkdir -p ~/capstone-db

# 폴더 안으로 이동
cd ~/capstone-db

#[STEP 2] docker-compose.yml 파일 작성
nano docker-compose.yml

```

```
version: '3.3'

services:
  mysql-db:
    image: mysql:8.0             # MySQL 8.0 버전 사용
    container_name: capstone_db  # 컨테이너 이름 (나중에 찾기 쉽게)
    restart: always              # 서버가 재부팅돼도 DB는 알아서 다시 켜짐
    environment:
      MYSQL_ROOT_PASSWORD: root_password_1234  # [중요] 관리자(root) 비밀번호 (변경하세요!)
      MYSQL_DATABASE: robot_capstone           # 처음에 자동으로 만들 데이터베이스 이름
      MYSQL_USER: robot_user                   # 로봇/유니티가 접속할 때 쓸 아이디
      MYSQL_PASSWORD: robot_password_1234      # [중요] 로봇/유니티용 비밀번호 (변경하세요!)
      TZ: Asia/Seoul                           # 한국 시간대 설정
    ports:
      - "7858:3306"              # 외부(7858) <-> 컨테이너 내부(3306) 연결
    volumes:
      - ./db_data:/var/lib/mysql # [핵심] DB 데이터를 서버 폴더(./db_data)에 영구 저장
      - ./conf:/etc/mysql/conf.d # 설정 파일 저장소
```

```
#[STEP 3] MySQL 실행하기
docker-compose up -d

#[STEP 4] 잘 설치됐는지 확인하기
docker ps

#[STEP 5] 접속 테스트 (서버 내부에서)
# 실행 중인 컨테이너 안으로 들어가서 mysql 접속 시도
docker exec -it capstone_db mysql -u root -p

#Enter password: 가 나오면 아까 docker-compose.yml에 적은 root 비밀번호를 입력하세요.
#mysql> 프롬프트가 뜨면 성공입니다!
#mysql> 프롬프트에서 SHOW DATABASES; robot_capstone이라는 이름이 보이면 성공.
```

**2.1 DB 스키마 (Schema)**
우선, 데이터베이스를 사용하기 위해서는 USE robot_capstone; 와 같이 데이터베이스를 사용하겠다고 선언해야함.

데이터베이스 사용 선언을 생략하고 쿼리 입력시 에러 발생. 

**참고로, 터미널에서 한글 입력시 공백으로 처리되는 이슈 발생. 되도록 영어 사용 권장.**

```
#[STEP 2] 데이터베이스 선택 및 설정 (step1은 docker exec -it capstone_db mysql -u root -p. MySQL 접속 과정)
USE robot_capstone;

-- 한글 저장을 위한 문자셋 설정 (혹시 몰라 안전장치)
ALTER DATABASE robot_capstone CHARACTER SET = utf8mb4 COLLATE = utf8mb4_unicode_ci;

```

```
#[STEP 3] 테이블 생성 SQL 코드
-- 1. 사용자 테이블 (Users)
CREATE TABLE IF NOT EXISTS Users (
    user_id VARCHAR(50) PRIMARY KEY COMMENT '로그인 ID',
    name VARCHAR(50) NOT NULL COMMENT '사용자 이름',
    role VARCHAR(20) DEFAULT 'Student' COMMENT 'Admin/Student',
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP COMMENT '가입일시'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- 2. 로봇 테이블 (Robots)
CREATE TABLE IF NOT EXISTS Robots (
    robot_id INT AUTO_INCREMENT PRIMARY KEY COMMENT '로봇 고유번호',
    model_type VARCHAR(20) NOT NULL COMMENT 'EP01 / GO1',
    ip_address VARCHAR(20) COMMENT '로봇 IP 주소',
    status VARCHAR(20) DEFAULT 'Idle' COMMENT 'Active/Idle/Charging',
    description VARCHAR(100) COMMENT '로봇 설명(별명)'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- 3. 실험 세션 테이블 (Sessions) - 누가, 언제, 어떤 로봇으로?
CREATE TABLE IF NOT EXISTS Sessions (
    session_id INT AUTO_INCREMENT PRIMARY KEY COMMENT '세션 번호',
    user_id VARCHAR(50) NOT NULL COMMENT '수행자 ID',
    robot_id INT NOT NULL COMMENT '사용 로봇 ID',
    start_time DATETIME DEFAULT CURRENT_TIMESTAMP COMMENT '시작 시간',
    end_time DATETIME NULL COMMENT '종료 시간',
    FOREIGN KEY (user_id) REFERENCES Users(user_id) ON DELETE CASCADE,
    FOREIGN KEY (robot_id) REFERENCES Robots(robot_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- 4. 이벤트 로그 테이블 (EventLogs) - 충돌, 에러 등 중요 사건
CREATE TABLE IF NOT EXISTS EventLogs (
    log_id BIGINT AUTO_INCREMENT PRIMARY KEY COMMENT '로그 번호',
    session_id INT NOT NULL COMMENT '관련 세션 번호',
    event_type VARCHAR(50) NOT NULL COMMENT 'Collision/Error/Goal',
    message TEXT COMMENT '상세 내용',
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP COMMENT '발생 시간',
    FOREIGN KEY (session_id) REFERENCES Sessions(session_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- 5. 제어 명령 로그 테이블 (CommandLogs) - 유니티가 보낸 명령 기록
CREATE TABLE IF NOT EXISTS CommandLogs (
    command_id BIGINT AUTO_INCREMENT PRIMARY KEY COMMENT '명령 번호',
    session_id INT NOT NULL COMMENT '관련 세션 번호',
    command_type VARCHAR(50) NOT NULL COMMENT 'Velocity/ModeChange',
    payload JSON COMMENT '명령 데이터 원본 (JSON)',
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP COMMENT '수신 시간',
    FOREIGN KEY (session_id) REFERENCES Sessions(session_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

```

```
#[STEP 4] 잘 만들어졌는지 확인
SHOW TABLES;


+--------------------------+
| Tables_in_robot_capstone |
+--------------------------+
| CommandLogs              |
| EventLogs                |
| Robots                   |
| Sessions                 |
| Users                    |
+--------------------------+
#처럼 나온다면 성공.
```

-----------------

## 3. 서버 어플리케이션 구현 (Server Implementation)

Python Flask 프레임워크와 PyMySQL을 사용하여 RESTful API 서버를 구현했습니다.

- **파일**: server_app.py
- **기능**: DB 커넥션 관리, JSON 요청 파싱, SQL 트랜잭션 처리.

```
from flask import Flask, request, jsonify
import pymysql
import json  # [추가됨]

app = Flask(__name__)

# [설정] DB 포트 확인
DB_CONFIG = {
    'host': '127.0.0.1',
    'port': 7858,
    'user': 'robot_user',
    'password': 'robot_password_1234',
    'db': 'robot_capstone',
    'charset': 'utf8mb4',
    'cursorclass': pymysql.cursors.DictCursor
}

def get_db():
    return pymysql.connect(**DB_CONFIG)

# 1. 로봇 상태 보고
@app.route('/robot/status', methods=['POST'])
def update_status():
    try:
        data = request.json
        serial = data.get('robot_serial', 'Unknown_Serial')
        status = data.get('status', 'Idle')
        ip_addr = request.remote_addr

        conn = get_db()
        cursor = conn.cursor()

        sql_check = "SELECT robot_id FROM Robots WHERE description = %s"
        cursor.execute(sql_check, (serial,))
        result = cursor.fetchone()

        if result:
            sql_update = "UPDATE Robots SET status=%s, ip_address=%s WHERE description=%s"
            cursor.execute(sql_update, (status, ip_addr, serial))
            print(f"🔄 로봇 상태 갱신: {serial} -> {status}")
        else:
            sql_insert = "INSERT INTO Robots (model_type, ip_address, status, description) VALUES ('EP01', %s, %s, %s)"
            cursor.execute(sql_insert, (ip_addr, status, serial))
            print(f"✨ 신규 로봇 발견 및 등록: {serial}")

        conn.commit()
        conn.close()
        return jsonify({"msg": "Status OK"}), 200
    except Exception as e:
        print(f"❌ 상태 업데이트 에러: {e}")
        return jsonify({"error": str(e)}), 500

# 2. 로그 저장
@app.route('/robot/log', methods=['POST'])
def save_log():
    try:
        data = request.json
        cmd_type = data.get('command_type')
        result = data.get('result')

        conn = get_db()
        cursor = conn.cursor()

        # [수정됨] JSON 형식으로 변환해서 저장
        # {"result": "Success"} 형태로 예쁘게 저장됩니다.
        json_payload = json.dumps({"result": result}, ensure_ascii=False)
        
        sql = "INSERT INTO CommandLogs (session_id, command_type, payload) VALUES (1, %s, %s)"
        cursor.execute(sql, (cmd_type, json_payload))
        
        conn.commit()
        conn.close()
        print(f"📝 로그 저장 완료: {cmd_type} - {result}")
        return jsonify({"msg": "Log Saved"}), 200
    except Exception as e:
        print(f"❌ 로그 저장 에러 (세션 1번 있나요?): {e}")
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
```

----------

## 4. 로봇 클라이언트 구현 (Robot Controller)

로봇(또는 로컬 PC)에서 서버로 데이터를 전송하는 클라이언트 스크립트입니다. requests 라이브러리를 사용해 서버 API를 호출합니다.

- **파일**: robot_controller.py

```
import rclpy
from rclpy.node import Node
import requests
import json
from robomaster import robot, conn

SERVER_URL = "http://210.110.250.33:5000" 

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info('>>> 로봇 제어 노드 시작')

        # 1. 로봇 하드웨어 연결 (STA 모드)
        self.ep_robot = robot.Robot()
        try:
            self.ep_robot.initialize(conn_type="sta")
            self.robot_sn = self.ep_robot.get_sn() or "TEST_ROBOT_SN"
            self.get_logger().info(f'>>> 로봇 연결됨 (SN: {self.robot_sn})')
        except Exception as e:
            self.get_logger().error(f'로봇 연결 실패: {e}')
            return

        # 2. 시나리오 실행
        self.run_scenario()

    def send_to_server(self, endpoint, payload):
        try:
            url = f"{SERVER_URL}/robot/{endpoint}"
            headers = {'Content-Type': 'application/json'}
            payload['robot_serial'] = self.robot_sn
            
            res = requests.post(url, data=json.dumps(payload), headers=headers, timeout=3)
            if res.status_code == 200:
                self.get_logger().info(f'[서버 전송 성공] {endpoint}')
            else:
                self.get_logger().warn(f'[서버 응답 에러] {res.status_code}')
        except Exception as e:
            self.get_logger().error(f'[서버 통신 실패] {e}')

    def run_scenario(self):
        # [A] 상태 보고: Active
        self.send_to_server('status', {'status': 'Active', 'model': 'EP01'})

        # [B] 동작 수행: 1m 전진
        try:
            self.get_logger().info('1m 전진 중...')
            self.ep_robot.chassis.move(x=1.0, y=0, z=0, xy_speed=0.5).wait_for_completed()
            
            # [C] 성공 로그 전송
            self.send_to_server('log', {'command_type': 'Move_1m', 'result': 'Success'})
        except Exception as e:
            self.send_to_server('log', {'command_type': 'Move_1m', 'result': f'Error: {e}'})

        # [D] 상태 보고: Idle
        self.send_to_server('status', {'status': 'Idle'})
        self.get_logger().info('시나리오 종료')

    def destroy_node(self):
        try:
            self.ep_robot.close()
        except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin_once(node, timeout_sec=1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---------

## 5. 실행 및 검증 (Verification)

먼저, 랩실 서버는 공용이므로 시스템 전체(sudo pip install)에 라이브러리를 깔면 안되기 때문에 가상환경을 구성해 동작시킴.

```
# 1. 가상환경 생성 (이름은 venv)
python3 -m venv venv

# 2. 가상환경 진입 (이 명령어를 치면 프롬프트 앞에 (venv)가 생깁니다. bash환경이 아닌 sh 환경의 실행 명령어)
. venv/bin/activate

# 3. 이제 맘껏 설치하세요. 이 폴더 안에서만 설치되고 서버에는 영향 안 줍니다.
pip install flask pymysql cryptography
```

**5.0 데이터베이스 기초 설정 (SQL 실행)**

```
#가장 먼저 데이터가 들어갈 "그릇"을 준비해야 합니다. 
#서버 터미널에서 MySQL에 접속(mysql -u robot_user -p ...)한 뒤, 아래 SQL 문을 복사해서 한 방에 실행하세요.

USE robot_capstone;

-- 1. 꼬임 방지를 위해 기존 테스트 데이터 싹 비우기 (순서 중요)
SET FOREIGN_KEY_CHECKS = 0;
TRUNCATE TABLE CommandLogs;
TRUNCATE TABLE Sessions;
TRUNCATE TABLE Robots;
TRUNCATE TABLE Users;
SET FOREIGN_KEY_CHECKS = 1;

-- 2. 테스트용 사용자 생성 (반드시 필요)
INSERT INTO Users (user_id, name, role) 
VALUES ('test_admin', '테스트관리자', 'Admin');

-- 3. 테스트용 로봇 생성 (반드시 필요 - 세션을 만들기 위해)
-- (나중에 실제 로봇이 접속하면 이 정보가 업데이트됩니다)
INSERT INTO Robots (robot_id, model_type, ip_address, status, description) 
VALUES (1, 'EP01', '0.0.0.0', 'Disconnected', 'Test_Bot_Placeholder');

-- 4. 테스트용 세션 생성 (핵심! 이게 있어야 로그가 저장됨)
-- 1번 로봇으로 test_admin이 실험을 한다고 가정
INSERT INTO Sessions (session_id, user_id, robot_id) 
VALUES (1, 'test_admin', 1);

-- 확인
SELECT * FROM Sessions; 
-- 결과에 1줄이 나와야 다음 단계로 넘어갈 수 있습니다.
```

**5.1 서버 프로그램 실행**


서버(server_app.py)를 실행하고 외부 요청을 대기합니다. 이 때, 서버를 실행한 터미널 창은 닫지 않아야 합니다.

```
(venv) $ python3 server_app.py
 * Running on http://0.0.0.0:5000
```

**5.2 로봇 연결**

- [로봇/노트북] 로봇 와이파이 연결: 공식 앱으로 랩실 와이파이에 연결 후 앱 종료.

**5.3 클라이언트 요청 및 결과 확인**


```
python3 robot_controller.py
```

실행 결과는 서버 터미널, 클라이언트 터미널에 출력되고 이후 DB 터미널에서 조회 가능.

명령을 수행중일때는 로봇이 Active 상태. 수행 중이지 않을때는 Idle 상태임을 확인함.

아래는 각 터미널 별 성공적인 로그를 기록하였음.

```
#로봇 제어 클라이언트 
 root@use1-ASUS-TUF-Dash-F15-FX516PR-FX516PR:~/robomaster_project/ros2_ws/src/my_robomaster/my_robomaster# python3 robot_controller.py

[INFO] [1768197748.797025044] [robot_controller]: >>> 로봇 제어 노드 시작

[INFO] [1768197749.103034217] [robot_controller]: >>> 로봇 연결됨 (SN: 3JKCK980030E3K)

[INFO] [1768197749.125363579] [robot_controller]: [서버 전송 성공] status

[INFO] [1768197749.126535932] [robot_controller]: 1m 전진 중...

[INFO] [1768197761.199751482] [robot_controller]: [서버 전송 성공] log

[INFO] [1768197761.215439913] [robot_controller]: [서버 전송 성공] status

[INFO] [1768197761.216054233] [robot_controller]: 시나리오 종료 

#서버 클라이언트
 (venv) $ python3 server_app.py

 * Serving Flask app 'server_app'

 * Debug mode: off

WARNING: This is a development server. Do not use it in a production deployment. Use a production WSGI server instead.

 * Running on all addresses (0.0.0.0)

 * Running on http://127.0.0.1:5000

 * Running on http://210.110.250.33:5000

Press CTRL+C to quit

127.0.0.1 - - [12/Jan/2026 06:00:53] "GET / HTTP/1.1" 404 -

127.0.0.1 - - [12/Jan/2026 06:00:53] "GET /favicon.ico HTTP/1.1" 404 -

🔄 로봇 상태 갱신: 3JKCK980030E3K -> Active

203.230.104.168 - - [12/Jan/2026 06:01:04] "POST /robot/status HTTP/1.1" 200 -

📝 로그 저장 완료: Move_1m - Success

203.230.104.168 - - [12/Jan/2026 06:01:12] "POST /robot/log HTTP/1.1" 200 -

🔄 로봇 상태 갱신: 3JKCK980030E3K -> Idle

203.230.104.168 - - [12/Jan/2026 06:01:12] "POST /robot/status HTTP/1.1" 200 -

🔄 로봇 상태 갱신: 3JKCK980030E3K -> Active

203.230.104.168 - - [12/Jan/2026 06:02:29] "POST /robot/status HTTP/1.1" 200 -

📝 로그 저장 완료: Move_1m - Success

203.230.104.168 - - [12/Jan/2026 06:02:41] "POST /robot/log HTTP/1.1" 200 -

🔄 로봇 상태 갱신: 3JKCK980030E3K -> Idle

203.230.104.168 - - [12/Jan/2026 06:02:41] "POST /robot/status HTTP/1.1" 200 - 

#DB 로그
 mysql> SELECT * FROM Robots;

+----------+------------+-----------------+--------------+----------------------+

| robot_id | model_type | ip_address      | status       | description          |

+----------+------------+-----------------+--------------+----------------------+

|        1 | EP01       | 0.0.0.0         | Disconnected | Test_Bot_Placeholder |

|        2 | EP01       | 203.230.104.168 | Idle         | 3JKCK980030E3K       |

+----------+------------+-----------------+--------------+----------------------+

2 rows in set (0.00 sec)


mysql> SELECT * FROM Robots;

+----------+------------+-----------------+--------------+----------------------+

| robot_id | model_type | ip_address      | status       | description          |

+----------+------------+-----------------+--------------+----------------------+

|        1 | EP01       | 0.0.0.0         | Disconnected | Test_Bot_Placeholder |

|        2 | EP01       | 203.230.104.168 | Active       | 3JKCK980030E3K       |

+----------+------------+-----------------+--------------+----------------------+

2 rows in set (0.01 sec)


mysql> SELECT * FROM Robots;

+----------+------------+-----------------+--------------+----------------------+

| robot_id | model_type | ip_address      | status       | description          |

+----------+------------+-----------------+--------------+----------------------+

|        1 | EP01       | 0.0.0.0         | Disconnected | Test_Bot_Placeholder |

|        2 | EP01       | 203.230.104.168 | Idle         | 3JKCK980030E3K       |

+----------+------------+-----------------+--------------+----------------------+

2 rows in set (0.00 sec)


mysql> SELECT * FROM CommandLogs;

+------------+------------+--------------+-----------------------+---------------------+

| command_id | session_id | command_type | payload               | created_at          |

+------------+------------+--------------+-----------------------+---------------------+

|          1 |          1 | Move_1m      | {"result": "Success"} | 2026-01-12 15:01:12 |

|          2 |          1 | Move_1m      | {"result": "Success"} | 2026-01-12 15:02:41 |

+------------+------------+--------------+-----------------------+---------------------+

2 rows in set (0.00 sec)




```

