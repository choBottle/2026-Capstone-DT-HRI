-- 1. 로봇 통합 상태 로그 테이블 (EP01, Go1 공용)
CREATE TABLE IF NOT EXISTS robot_telemetry (
    id SERIAL PRIMARY KEY,
    robot_id VARCHAR(50) NOT NULL,    -- 예: EP01_SN_001
    robot_type VARCHAR(20) NOT NULL,  -- 예: EP01, GO1
    data_type VARCHAR(30) NOT NULL,   -- 예: battery, position, impact, armor_hit
    payload JSONB NOT NULL,           -- 다양한 센서 데이터를 JSON 형태로 유연하게 저장
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- 2. 로봇 제어 명령 기록 테이블 (Audit Log)
CREATE TABLE IF NOT EXISTS robot_commands (
    id SERIAL PRIMARY KEY,
    robot_id VARCHAR(50) NOT NULL,
    command VARCHAR(50) NOT NULL,
    params JSONB,
    sender VARCHAR(50),               -- 명령 주체 (Unity, AI_Model 등)
    status VARCHAR(20) DEFAULT 'sent',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- 특정 로봇의 최신 데이터를 타입별로 빠르게 조회하기 위한 인덱스
CREATE INDEX idx_robot_status_latest ON robot_telemetry (robot_id, data_type, created_at DESC);

-- status 외에 로봇으로부터의 리턴값을 저장할 필드 추가 (선택 사항)
ALTER TABLE robot_commands ADD COLUMN result_payload JSONB;