
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


## 제어 코드 작성 (Python)

### 1. 기본 동작 코드
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

### 2. 키보드 제어 코드
아래 코드는 **Servo Gripper**를 장착한 상태에서 키보드로 로봇 팔을 제어할 수 있는 코드입니다.
`keyboard_control.py` 파일을 만들고 아래 내용을 붙여넣으세요.

**조작 키**

* W / S: 앞으로 뻗기 / 몸 쪽으로 당기기 (X축)

* A / D: 왼쪽 / 오른쪽 (Y축)

* Q / E: 위로 들기 / 아래로 내리기 (Z축)

* U / J: 집게 열기 / 잡기

* ESC: 프로그램 종료

```python
import serial
import time
import sys
import tty
import termios

# ================= [설정 구간] =================
SERIAL_PORT = '/dev/ttyUSB0'  # 본인 포트 이름으로 변경 (/dev/ttyACM0 일 수도 있음)
BAUD_RATE = 115200

# 이동 단위 (mm) - 한 번 누를 때마다 이동할 거리
STEP_SIZE = 10  
# Z축 이동 단위 (mm)
Z_STEP_SIZE = 10 

# [안전 장치] 소프트웨어 좌표 제한 (기구적 한계 보호)
# 이 범위를 넘어가려 하면 코드가 명령을 보내지 않습니다.
LIMITS = {
    'min_x': 100, 'max_x': 280,  # 앞뒤 (너무 가까우면 몸통 충돌)
    'min_y': -150, 'max_y': 150, # 좌우
    'min_z': 0,   'max_z': 180   # 높이
}

# 그리퍼 설정 (Servo Gripper)
GRIPPER_OPEN = 40
GRIPPER_CLOSE = 60
# ==============================================

# 리눅스 터미널에서 키 입력 한 글자씩 받아오는 함수 (Enter 없이 즉시 반응)
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    try:
        # 1. 시리얼 연결
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"[{SERIAL_PORT}] 연결 성공! 초기화 중...")
        time.sleep(3)

        def send_gcode(cmd):
            ser.write((cmd + '\r\n').encode())
            # 빠르게 연속 입력 받기 위해 응답 대기는 최소화하거나 생략
            time.sleep(0.05) 

        # 2. 초기화 (호밍 & 모드 설정)
        print(">>> 호밍(Homing) 시작... (잠시 대기)")
        send_gcode("$H")
        time.sleep(20) # 호밍 대기

        print(">>> 좌표 모드 설정")
        send_gcode("M20")
        time.sleep(0.5)
        send_gcode("G90") # 절대 좌표 모드
        time.sleep(0.5)

        # 3. 시작 위치로 이동
        # 현재 로봇의 위치를 변수에 저장해두고 관리합니다.
        curr_x, curr_y, curr_z = 200, 0, 100
        curr_gripper = GRIPPER_OPEN
        
        print(f">>> 시작 위치로 이동: X{curr_x} Y{curr_y} Z{curr_z}")
        send_gcode(f"G0 X{curr_x} Y{curr_y} Z{curr_z}")
        send_gcode(f"M3 S{curr_gripper}") # 그리퍼 열기
        time.sleep(2)

        # 4. 키보드 제어 루프
        print("\n" + "="*40)
        print("   [MT4 키보드 제어 모드]")
        print("   W / S : 앞 / 뒤 (X축)")
        print("   A / D : 좌 / 우 (Y축)")
        print("   Q / E : 위 / 아래 (Z축)")
        print("   U / J : 그리퍼 열기 / 닫기")
        print("   ESC   : 종료")
        print("="*40 + "\n")

        while True:
            key = getch() # 키 입력 대기 (블로킹)

            # 변경 전 좌표 기억
            next_x, next_y, next_z = curr_x, curr_y, curr_z
            moved = False
            gripper_action = False

            # 키 매핑 확인
            if key == 'w': # 앞
                next_x += STEP_SIZE
                moved = True
            elif key == 's': # 뒤
                next_x -= STEP_SIZE
                moved = True
            elif key == 'a': # 좌 (Y+)
                next_y += STEP_SIZE
                moved = True
            elif key == 'd': # 우 (Y-)
                next_y -= STEP_SIZE
                moved = True
            elif key == 'q': # 위
                next_z += Z_STEP_SIZE
                moved = True
            elif key == 'e': # 아래
                next_z -= Z_STEP_SIZE
                moved = True
            
            # 그리퍼 제어
            elif key == 'u': # 열기
                curr_gripper = GRIPPER_OPEN
                gripper_action = True
                print("   [Gripper] OPEN")
            elif key == 'j': # 닫기
                curr_gripper = GRIPPER_CLOSE
                gripper_action = True
                print("   [Gripper] CLOSE")

            # 종료
            elif ord(key) == 27: # ESC 키
                print("\n>>> 종료합니다.")
                break

            # 5. 유효성 검사 및 명령 전송
            if moved:
                # 좌표 제한 확인 (Safety Check)
                if (LIMITS['min_x'] <= next_x <= LIMITS['max_x'] and
                    LIMITS['min_y'] <= next_y <= LIMITS['max_y'] and
                    LIMITS['min_z'] <= next_z <= LIMITS['max_z']):
                    
                    # 유효하면 좌표 업데이트 및 전송
                    curr_x, curr_y, curr_z = next_x, next_y, next_z
                    cmd = f"G0 X{curr_x} Y{curr_y} Z{curr_z}"
                    send_gcode(cmd)
                    print(f"MOVED -> X:{curr_x} Y:{curr_y} Z:{curr_z}\r", end='') # \r로 줄바꿈 없이 갱신
                else:
                    print(f"\n[WARNING] 제한 범위 도달! ({next_x}, {next_y}, {next_z})")
            
            if gripper_action:
                send_gcode(f"M3 S{curr_gripper}")

    except Exception as e:
        print(f"\n에러 발생: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
```


## 3. 실행 방법 (공통)

터미널에서 파일이 있는 경로로 이동하여 실행합니다.

```bash
python3 robot_control.py
```

```bash
python3 keyboard_control.py
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
