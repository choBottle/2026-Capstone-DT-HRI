
# Wlkata MT4 초기 세팅 가이드

## 사전 준비

* 로봇 팔 본체와 집게 체결 : 체결부분에 있는 조그만한 나사 조이기 (드라이버는 EP01 몸체 조립했을 때 썼던 거 쓰면 됨)
* Wlkata Studio와 드라이버를 컴퓨터에 설치하기
([다운로드 링크](https://www.wlkata.com/pages/download-center?srsltid=AfmBOopjzXUAv34lXLxKYnwVsh8p6IMGio8ZRPhKMk-1xZnuvKThTezV))
* 로봇팔 본체 주위를 미리 비워두기 (충돌 대비)

### 1. Power Supply와 로봇팔 본체를 연결한다

### 2. Extender Box의 COMMUNICATION INTERFACE 부분과 로봇팔 본체를 IDC Cable로 연결한다

### 3. Extender Box의 USB 부분과 컴퓨터를 USB Cable로 연결한다

* 여기까지 연결됐다면 전원을 켤 수 있고 컴퓨터 Wlkata Studio에서 인식이 된다
* Wlkata Studio 왼쪽 상단 Device를 MT4/E4로 해야 인식이 되므로 주의

### 4-1. Two-Finger Soft Gripper or Suction Cup을 장착한 경우

* Pnematic Pump 호스와 (Two-Finger Soft Gripper or Suction Cup)을 연결하고, Pnematic Pump와 Extender Box(상단 PUMP/GRIPPER PWM 부분)를 같이 들어있는 얇은 케이블(끝부분이 노란색)로 연결한다

### 4-2. Metal Claw를 장착한 경우

* Metal Claw에 연결되어 있는 선을 Extender Box(상단 PUMP/GRIPPER PWM 부분)에 연결한다

### 5. Wltaka Studio에서 조작을 시도해본다

* COORD MODE는 5도씩 정밀하게 조작이 가능한데, X축, Y축, Z축 각각 해보고 잘 작동하는지 확인한다

### 6-1. Two-Finger Soft Gripper or Suction Cup을 장착한 경우 (테스트 안해봄)

* Wltaka Studio - 오른쪽 Motion Control - Tool에서 Pneumatic Tool을 선택하고 CTRL에서 원하는 작동이 잘 되는지 확인한다

### 6-2. Metal Claw를 장착한 경우

* Wltaka Studio - 오른쪽 Motion Control - Tool에서 Servo Gripper를 선택하고 CTRL에서 원하는 작동이 잘 되는지 확인한다 (집게 열기/닫기)

---

## Ubuntu cmd를 통한 조작

Wlkata Studio 없이 리눅스(Ubuntu) 터미널 환경에서 파이썬 코드로 로봇 팔을 제어하는 방법

### 필수 라이브러리 설치 및 포트 확인

**1. 시리얼 통신 라이브러리 설치**

   터미널을 열고 파이썬 시리얼 라이브러리를 설치합니다.
   
   ```bash
   pip3 install pyserial
   ```

**2. USB 포트 확인 (중요)**

   Ubuntu에서는 로봇 연결 시 `brltty`(점자 디스플레이 드라이버)와 충돌하여 포트가 안 뜨는 경우가 많으므로 포트가 검색되지 않을 시 3번을 참고하시면 됩니다.

   ```bash
   ls /dev/ttyUSB*
   ```
   또는
   ```bash
   ls /dev/ttyACM*
   ```

**3. 포트가 검색되지 않을 경우 (Brltty 삭제):**

   ```bash
   sudo apt remove brltty
   ```

   삭제 후 USB를 뽑았다가 다시 꽂으면 `/dev/ttyUSB0` 또는 `/dev/ttyACM0`이 정상적으로 인식됩니다.


**4. 포트 접근 권한 부여**

   매번 `sudo` 없이 코드를 실행하기 위해 권한을 허용합니다. (재부팅 시 초기화됨)
   
   ```bash
   # 본인의 포트 이름에 맞게 입력 (ttyUSB0 또는 ttyACM0)
   sudo chmod 666 /dev/ttyUSB0
   ```


### 제어 코드 작성 (Python)

아래 코드는 **Servo Gripper**를 장착한 상태에서 안전하게 구동하도록 작성된 테스트 코드입니다.
`robot_control.py` 파일을 만들고 아래 내용을 붙여넣으세요.

**주요 기능:**

* 안전한 호밍 (Homing) 대기
* 좌표 제어 모드(`M20`) 자동 설정 (Soft Limit 에러 방지)
* Servo Gripper 열기/닫기 (`S40`~`S60`)
* 종료 시 안전한 위치로 복귀 (`X200 Y0 Z100`)

```python
import serial
import time

# [설정] 본인의 포트 이름으로 변경 (/dev/ttyUSB0 또는 /dev/ttyACM0)
SERIAL_PORT = '/dev/ttyUSB0'  
BAUD_RATE = 115200

try:
    # 1. 시리얼 연결
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"[{SERIAL_PORT}] 연결 성공! 초기화 중...")
    time.sleep(3) 

    def send_command(command):
        full_command = command + '\r\n'
        ser.write(full_command.encode('utf-8'))
        print(f"Sent: {command}")
        
        # 로봇 응답 대기 (명령 씹힘 방지)
        end_time = time.time() + 1.0 
        while time.time() < end_time:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if line: print(f"Recv: {line}")
                except: pass

    # ================= 동작 시퀀스 시작 =================

    # [1] 호밍 (Homing) - 전원 켜고 최초 1회 필수
    print(">>> 1. 호밍(Homing) 시작... (약 25초 소요)")
    send_command("$H") 
    
    # 호밍이 완료될 때까지 충분히 대기 (중요)
    for i in range(25):
        time.sleep(1)
        while ser.in_waiting > 0: ser.readline() 

    # [2] 모드 설정 (Soft Limit 에러 방지)
    print(">>> 2. 좌표 모드(Cartesian) 설정")
    send_command("M20")  # 좌표 제어 모드 활성화 (필수)
    time.sleep(0.5)
    send_command("G90")  # 절대 좌표 모드
    time.sleep(0.5)

    # [3] 이동 테스트
    print(">>> 3. 이동 및 그리퍼 동작")
    send_command("G0 X180 Y0 Z150") # 테스트 위치로 이동
    time.sleep(2) 

    # [4] 그리퍼 동작 (Servo Gripper 기준)
    # S40: 열기 / S60: 닫기 (값은 미세 조정 가능)
    print(">>> 그리퍼 열기")
    send_command("M3 S40")
    time.sleep(1)
    
    print(">>> 그리퍼 닫기")
    send_command("M3 S60")
    time.sleep(1)

    # [5] 종료 (안전 복귀)
    print(">>> 4. 안전한 위치로 복귀")
    # 주의: X0 Y0 Z0은 로봇 몸통 내부이므로 절대 이동 금지
    send_command("G0 X200 Y0 Z100") 
    time.sleep(2)
    
    send_command("M3 S60") # 그리퍼 닫고 마무리

    ser.close()
    print(">>> 연결 종료")

except Exception as e:
    print(f"에러 발생: {e}")
    print("팁: sudo chmod 666 /dev/ttyUSB0 명령어로 권한을 확인하세요.")

```

### 실행 방법

터미널에서 파일이 있는 경로로 이동하여 실행합니다.

```bash
python3 robot_control.py
```

### 자주 발생하는 에러(Troubleshooting)

| 에러 메시지 / 현상 | 원인 및 해결 방법 |
| --- | --- |
| **Permission denied** | 포트 접근 권한이 없습니다. `sudo chmod 666 /dev/ttyUSB0`을 입력하세요. |
| **Error, Soft limit: X (or Y)** | 1. `M20` 명령어가 누락되어 로봇이 각도 모드로 인식한 경우입니다.<br>2. 이동하려는 좌표가 로봇의 물리적 한계를 벗어났습니다. (예: `G0 X0 Y0 Z0`) |
| **Device not found** | `brltty` 프로그램 충돌입니다. `sudo apt remove brltty` 후 재연결하세요. |
| **그리퍼가 움직이지 않음** | 코드의 `M3 S` 값이 올바른지 확인하세요.<br>(Pneumatic: S0/S1000, Servo: S40/S60) |
---

### 참고사항

* 간단 테스트 영상 : https://youtu.be/aABIZw-NVqQ
* 문제가 생기지 않도록 한 동작을 완료하고 다음 동작을 테스트하기 전에 HOMING을 통해 로봇팔의 위치를 초기화하고 하는 것이 좋다
* 부품 명칭은 WLKATA MT4 Edu Kit Shipping List 종이에 있는 명칭을 그대로 사용하였으므로 해당 가이드 참고 시 해당 종이를 참고하여 수행하면 된다
