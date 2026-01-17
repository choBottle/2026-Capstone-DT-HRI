
# Wlkata MT4 초기 세팅 가이드

## 사전 준비

* 로봇 팔 본체와 집게 체결 : 체결부분에 있는 조그만한 나사 조이기 (드라이버는 EP01 몸체 조립했을 때 썼던 거 쓰면 됨)
* Wlkata Studio와 드라이버를 컴퓨터에 설치하기
([다운로드 링크](https://www.wlkata.com/pages/download-center?srsltid=AfmBOopjzXUAv34lXLxKYnwVsh8p6IMGio8ZRPhKMk-1xZnuvKThTezV))
* 로봇팔 본체 주위를 미리 비워두기 (충돌 대비)

### 1. Power Supply와 로봇팔 본체를 연결한다

### 2. Extender Box 상단과 로봇팔 본체를 IDC Cable로 연결한다

### 3. Power Supply의 COMMUNICATION INTERFACE 부분과 컴퓨터를 USB Cable로 연결한다

* 여기까지 연결됐다면 전원을 켤 수 있고 컴퓨터 Wlkata Studio에서 인식이 된다
* Wlkata Studio 왼쪽 상단 Device를 MT4/E4로 해야 인식이 되므로 주의

### 4-1. Two-Finger Soft Gripper or Suction Cup을 장착한 경우

* Pnematic Pump 호스와 (Two-Finger Soft Gripper or Suction Cup)을 연결하고, Pnematic Pump와 Extender Box(상단 PUMP/GRIPPER PWM 부분)를 같이 들어있는 얇은 케이블(끝부분이 노란색)로 연결한다

### 4-2. Metal Claw를 장착한 경우

* Metal Claw에 있는 선을 Extender Box(상단 PUMP/GRIPPER PWM 부분)에 연결한다

### 5. Wltaka Studio에서 조작을 시도해본다

* COORD MODE는 5도씩 정밀하게 조작이 가능한데, X축, Y축, Z축 각각 해보고 잘 작동하는지 확인한다

### 6-1. Two-Finger Soft Gripper or Suction Cup을 장착한 경우 (테스트 안해봄)

* Wltaka Studio - 오른쪽 Motion Control - Tool에서 Pneumatic Tool을 선택하고 CTRL에서 원하는 작동이 잘 되는지 확인한다

### 6-2. Metal Claw를 장착한 경우

* Wltaka Studio - 오른쪽 Motion Control - Tool에서 Servo Gripper를 선택하고 CTRL에서 원하는 작동이 잘 되는지 확인한다 (집게 열기/닫기)

---

### 간단 테스트 영상

* https://youtu.be/aABIZw-NVqQ

* ※ 문제가 생기지 않도록 한 동작을 완료하고 다음 동작을 테스트하기 전에 HOMING을 통해 로봇팔의 위치를 초기화하고 하는 것이 좋다