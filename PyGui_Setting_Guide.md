# PyGui 접속 가이드

## [MobaXterm 설치 및 실행] - Windows 한정

### 1. Windows에서 설치 시

1. [MobaXterm Download Page](https://mobaxterm.mobatek.net/download-home-edition.html) 링크로 들어가 **Portable edition** 다운로드
2. 설치 후 압축을 풀고 `.exe` 파일 실행

### 2. Linux(Ubuntu)에서 설치 시

별도의 프로그램 설치가 필요 없습니다. 기본 터미널을 사용하세요.

명령어 입력: (반드시 대문자 -X 옵션을 포함해야 합니다)

예시
```bash
ssh -X physical@pi2.local
```
비밀번호를 입력하면 바로 접속됩니다. [PyGui 세팅] 단계로 넘어가세요.

---

## [MobaXterm 세팅] - Windows 한정

1. 좌측 상단 **Session** → 좌측 상단 **SSH** 탭 클릭
2. **Remote host**에 라즈베리파이의 IP 주소 입력
3. **Specify username** 체크박스를 체크하고, username (`physical`) 입력
4. **Advanced SSH settings** 탭에서 **X11-Forwarding**이 체크되어 있는지 확인
5. **OK** 버튼을 눌러 설정 완료
6. 설정 완료 후 뜨는 창에서 **Accept** 클릭
7. `cmd` 창에서 password 입력
8. 입력 후 뜨는 창에서 **Yes** 클릭
9. **Master Password** 설정 후 **OK** 클릭 (MobaXterm 접속 시 비밀번호 설정, Raspi와 연관 없음)
10. 이후 MobaXterm 실행할 때마다 왼쪽 **User sessions**에서 라즈베리파이 서버를 찾아 바로 접속할 수 있음

---

## [PyGui 세팅]

### 기본 테스트 코드 (`test_gui.py`)

```python
import dearpygui.dearpygui as dpg

def button_callback(sender, app_data):
    print(f"Button Clicked! (Sender: {sender})")
    dpg.set_value("text_output", "Status: Button Clicked! (Check Terminal)")

dpg.create_context()

with dpg.window(label="System Check Window", width=580, height=380, pos=[10, 10]):
    dpg.add_text("PyGui Display Test", color=(0, 255, 0))
    dpg.add_separator()
    dpg.add_text("If you see this window, X11 Forwarding is working correctly.")
    dpg.add_spacer(height=10)
    
    dpg.add_button(label="Test Button (Click Me)", callback=button_callback)
    dpg.add_spacer(height=10)
    
    dpg.add_text("Waiting...", tag="text_output", color=(255, 255, 0))

dpg.create_viewport(title='MobaXterm PyGui Test', width=600, height=400)
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
```

---

## [PyGui 세팅 및 테스트 실행]

### 1. 파이썬 가상환경 설정 (최초 1회만 수행)

라즈베리파이 시스템 보호를 위해 가상환경 내에서 라이브러리를 설치합니다.

1. 터미널(MobaXterm) 접속 후 홈 디렉토리로 이동
```bash
cd ~

```


2. 가상환경 생성 (`robot_env`라는 이름의 방 만들기)
```bash
python3 -m venv robot_env

```


3. 가상환경 활성화 (필수)
```bash
source robot_env/bin/activate

```


*(터미널 명령어 입력줄 맨 앞에 `(robot_env)` 표시가 뜨면 성공)*
4. 필수 라이브러리 설치
```bash
pip install --upgrade pip
pip install dearpygui

```



---

### 2. 테스트 코드 작성

화면이 정상적으로 출력되는지 확인하기 위해 테스트 파일을 만듭니다.

1. 파일 생성 및 편집기 열기
```bash
nano test_gui.py

```


2. 기본 테스트 코드를 복사하여 붙여넣기
*(MobaXterm에서는 우클릭 또는 `Shift`+`Insert`로 붙여넣기 가능)*
3. 저장하고 나가기
* `Ctrl` + `O` (저장) → `Enter`
* `Ctrl` + `X` (나가기)



---

### 3. 실행 및 검증 (매번 수행)

실제로 내 PC 화면에 GUI 창이 뜨는지 확인합니다.

1. **가상환경 활성화** (접속 시마다 입력)
```bash
source ~/robot_env/bin/activate

```


2. **테스트 코드 실행**
```bash
python3 test_gui.py

```


3. **검증 포인트 (Check Point)**
* **성공:** 약 1~3초 뒤 PC 화면에 **"System Check Window"**라는 회색 창이 팝업으로 뜹니다.
* **기능 확인:** 창 안의 "Test Button"을 누르면 터미널에 메시지가 출력되는지 확인하세요.



---

## [문제 해결 (Troubleshooting)]

### Q1. "Glfw Error: The DISPLAY environment variable is missing" 에러 발생
> **원인:** MobaXterm의 X11-Forwarding 기능이 꺼져있거나 세션이 꼬인 경우입니다.
>
> **해결:**
> 1. MobaXterm 접속 설정(Session Settings)에서 **`X11-Forwarding`** 체크 여부를 확인하세요.
> 2. 접속을 종료(`exit`)하고 MobaXterm을 완전히 껐다가 다시 실행하여 재접속하세요.

---

### Q2. 창이 너무 작거나 검은 화면만 나와요.
> **해결:** MobaXterm이 화면 정보를 늦게 받아서 그럴 수 있습니다.
> 창을 마우스로 잡고 **살짝 흔들거나 크기를 조절**하면 화면이 정상적으로 나타납니다.

---

### Q3. 한글이 네모(□□□)로 나오거나 깨져요.
> **원인:** 라즈베리파이에 한글 폰트가 설치되어 있지 않아서 발생하는 문제입니다.
>
> **해결:** 터미널에 아래 명령어를 입력하여 한글 폰트(나눔 폰트)를 설치하면 해결됩니다.
> ```bash
> sudo apt update
> sudo apt install fonts-nanum -y
> ```
> *(설치 완료 후 프로그램을 다시 실행해 보세요.)*
