<dji><attribute><creation_date>2025/12/30</creation_date><sign>3f0d917349426d69</sign><modify_time>12/31/2025 9:43:04 AM</modify_time><guid>bc6c63c730ad455ea5321cbe4b1c619d</guid><creator>Anonymous</creator><firmware_version_dependency>00.00.0000</firmware_version_dependency><title>CounterAttack</title><code_type>python</code_type><app_min_version></app_min_version><app_max_version></app_max_version></attribute><audio-list /><code><python_code><![CDATA[def start():
    
# 로봇이 공격을 당했을 때 어디서 맞았는지 감지하고 적을 향해 총구를 돌려 반격하며 반대 방향으로 후퇴하는 로직

# 로봇 모드 설정 (Free 모드여야 짐벌과 차체가 따로 놉니다)
    robot_ctrl.set_mode(rm_define.robot_mode_free)
    
    # 짐벌 회전 속도 (도/초)
    gimbal_ctrl.set_rotate_speed(540)
    
    # [수정됨] 이동 속도 조절 (단위: m/s)
    # 기존 1.0에서 0.5로 줄였습니다. 더 느리게 하려면 0.3 정도로 낮추세요.
    chassis_ctrl.set_trans_speed(0.5)
    
    # 1. 초기 상태: 짐벌 LED와 모든 장갑판 LED를 파란색으로 설정
    led_ctrl.set_top_led(rm_define.armor_top_all, 0, 0, 255, rm_define.effect_always_on)
    led_ctrl.set_bottom_led(rm_define.armor_bottom_all, 0, 0, 255, rm_define.effect_always_on)
    
    # 시작할 때 정면 초기화
    gimbal_ctrl.recenter()
    current_yaw = 0 

    while True:
        target_angle = 0
        evasive_angle = 0 # 도망갈 방향 (차체 기준)
        is_hit = False
        hit_armor_id = 0 

        # ---------------------------------------------------
        # 1. 맞은 부위 확인 -> 반격 각도 및 도망갈 방향 설정
        # ---------------------------------------------------
        
        # [정면 피격] -> 적은 앞(0도) -> 뒤(180도)로 도망
        if armor_ctrl.check_condition(rm_define.cond_armor_bottom_front_hit):
            target_angle = 0
            evasive_angle = 180 
            hit_armor_id = rm_define.armor_bottom_front
            is_hit = True
            
        # [우측 피격] -> 적은 오른쪽(90도) -> 왼쪽(-90도)으로 도망
        elif armor_ctrl.check_condition(rm_define.cond_armor_bottom_right_hit):
            target_angle = 90
            evasive_angle = -90
            hit_armor_id = rm_define.armor_bottom_right
            is_hit = True
            
        # [좌측 피격] -> 적은 왼쪽(-90도) -> 오른쪽(90도)으로 도망
        elif armor_ctrl.check_condition(rm_define.cond_armor_bottom_left_hit):
            target_angle = -90
            evasive_angle = 90
            hit_armor_id = rm_define.armor_bottom_left
            is_hit = True
            
        # [후면 피격] -> 적은 뒤(180도) -> 앞(0도)으로 도망
        elif armor_ctrl.check_condition(rm_define.cond_armor_bottom_back_hit):
            # 짐벌 회전 최단거리 계산 로직
            if current_yaw >= 0:
                target_angle = 180
            else:
                target_angle = -180
            
            evasive_angle = 0 # 앞으로 도망
            hit_armor_id = rm_define.armor_bottom_back
            is_hit = True

        # ---------------------------------------------------
        # 2. 반응 행동 (LED -> 회전 -> 사격 -> 회피기동)
        # ---------------------------------------------------
        if is_hit:
            # 소리 재생 및 LED 효과 (짐벌: 전체 점멸, 바디: 맞은 곳만 켜짐)
            media_ctrl.play_sound(rm_define.media_sound_attacked)
            led_ctrl.set_top_led(rm_define.armor_top_all, 255, 0, 0, rm_define.effect_flash)
            led_ctrl.set_bottom_led(hit_armor_id, 255, 0, 0, rm_define.effect_always_on)
            
            # 짐벌 회전 각도 계산 및 실행
            diff = target_angle - current_yaw
            
            if diff > 0:
                gimbal_ctrl.rotate_with_degree(rm_define.gimbal_right, diff)
            elif diff < 0:
                gimbal_ctrl.rotate_with_degree(rm_define.gimbal_left, abs(diff))
            
            current_yaw = target_angle # 현재 각도 업데이트
            
            # [반격 사격]
            gun_ctrl.fire_once()
            
            # [회피 기동] 설정된 속도(0.5m/s)로 1초간 이동
            chassis_ctrl.move_with_time(evasive_angle, 1.0)
            
            # ---------------------------------------------------
            # [복귀] LED 색상 원상복구 (파란색)
            # ---------------------------------------------------
            led_ctrl.set_top_led(rm_define.armor_top_all, 0, 0, 255, rm_define.effect_always_on)
            led_ctrl.set_bottom_led(hit_armor_id, 0, 0, 255, rm_define.effect_always_on)


        time.sleep(0.01)]]></python_code><scratch_description><![CDATA[]]></scratch_description></code></dji>
