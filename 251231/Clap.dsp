<dji><attribute><creation_date>2025/12/31</creation_date><sign>4a25913885988ad7</sign><modify_time>12/31/2025 2:47:34 PM</modify_time><guid>eeb4138d51de44b394b8807a5ceb0326</guid><creator>Anonymous</creator><firmware_version_dependency>00.00.0000</firmware_version_dependency><title>Clap</title><code_type>python</code_type><app_min_version></app_min_version><app_max_version></app_max_version></attribute><audio-list /><code><python_code><![CDATA[def start():
# 박수 소리를 인식하여 특정 동작을 수행하는 코드, 박수 두 번은 전진, 세 번은 발사    
# 1. 초기 설정
    robot_ctrl.set_mode(rm_define.robot_mode_free)
    
    # [중요] 박수 인식 기능 켜기
    # 이 함수가 실행되어야 로봇이 소리를 듣기 시작합니다.
    media_ctrl.enable_sound_recognition(rm_define.sound_detection_applause)
    
    # 평상시 LED: 파란색
    led_ctrl.set_top_led(rm_define.armor_top_all, 0, 0, 255, rm_define.effect_always_on)
    
    print("박수 인식 대기 중... (2번: 전진, 3번: 발사)")

    # [핵심] 프로그램이 종료되지 않고 계속 귀를 열어두기 위한 무한 루프
    # 스크래치의 'wait' 블록 역할을 합니다.
    while True:
        time.sleep(1)

# -----------------------------------------------------------------
# [이벤트 1] 박수 2번이 감지되면 자동으로 실행되는 함수
# -----------------------------------------------------------------
def sound_recognized_applause_twice(msg):
    print(">>> [2번] 전진!")
    
    # 1. 알림 (보라색)
    led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 255, rm_define.effect_always_on)
    
    # 2. 전진 동작
    chassis_ctrl.set_trans_speed(0.5)
    chassis_ctrl.move(0) # 0도: 전진
    time.sleep(0.5)
    chassis_ctrl.stop()
    
    # 3. 복귀 (파란색)
    led_ctrl.set_top_led(rm_define.armor_top_all, 0, 0, 255, rm_define.effect_always_on)

# -----------------------------------------------------------------
# [이벤트 2] 박수 3번이 감지되면 자동으로 실행되는 함수
# -----------------------------------------------------------------
def sound_recognized_applause_thrice(msg):
    print(">>> [3번] 발사!")
    
    # 1. 알림 (빨간색 점멸)
    led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_flash)
    
    # 2. 발사 동작
    gun_ctrl.fire_once()
    time.sleep(0.5)
    
    # 3. 복귀 (파란색)

    led_ctrl.set_top_led(rm_define.armor_top_all, 0, 0, 255, rm_define.effect_always_on)]]></python_code><scratch_description><![CDATA[]]></scratch_description></code></dji>
