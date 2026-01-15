# [DB Team] Integration & Research Guide

이 문서는 DB팀이 연구한 스키마 설계를 `physical_ai` 프로젝트에 효율적으로 통합하기 위한 지침입니다.

## 1. 프로젝트 내 DB 구조 이해

현재 DB는 Docker 컨테이너로 실행되며, 초기 설정은 아래 파일들에 의존합니다.

- **`docker-compose.yml`**: DB 컨테이너 설정 (ID/PW, 포트, 볼륨 마운트)
- **`database/init.sql`**: 컨테이너가 처음 생성될 때 실행되는 테이블 생성 스크립트
- **`postgres_data/`**: 실제 데이터가 저장되는 물리적 공간 (깃 업로드 제외 대상)

## 2. DB 팀의 주요 작업 영역 (Integration Points)

### **A. 스키마 수정 및 추가 (`init.sql`)**

연구하신 새로운 테이블 구조나 인덱스, 뷰(View) 등을 적용하려면 `database/init.sql` 파일을 수정하세요.

- **주의:** 이미 컨테이너가 생성된 상태에서 `init.sql`만 수정하면 반영되지 않습니다. 반드시 아래 명령어로 볼륨을 지우고 다시 생성해야 합니다.

Bash

`docker compose down -v  # 볼륨(데이터)까지 완전히 삭제
docker compose up -d    # 수정된 init.sql로 다시 생성`

### **B. JSONB 데이터 최적화 연구**

현재 로봇 데이터는 `payload`라는 **JSONB** 컬럼에 저장됩니다.

- **과제:** 특정 센서 데이터(예: 배터리, 가속도)를 추출하는 쿼리 성능 최적화.
- **가이드:** `GIN 인덱스` 적용 등 연구 내용을 `init.sql`에 반영하여 배포하세요.

---

## 3. 실수가 없기 위한 협업 워크플로우

### **Step 1: 브랜치 전략**

본인의 연구 내용을 적용할 때는 반드시 새 브랜치를 생성하세요.

Bash

`git checkout -b feature/db-optimization-research`

### **Step 2: 로컬 환경 테스트**

`main_server/hub_server.py`의 DB 연동 코드를 확인하여, 본인이 수정한 테이블 이름이나 컬럼명과 일치하는지 검증해야 합니다.

- **연동 파일:** `hub_server.py` 내의 `handle_processed_data` 함수 확인.

### **Step 3: 데이터 마이그레이션 전략**

만약 서비스 중단 없이 테이블 구조를 바꿔야 한다면, `init.sql` 대신 별도의 마이그레이션 스크립트를 어떻게 실행할 것인지 연구가 필요합니다.

---

## 4. DB팀을 위한 체크리스트 (Best Practices)

- **성능 모니터링:** 많은 로봇 데이터가 들어올 때 쓰기(Write) 속도 지연이 없는지 확인.
- **데이터 정합성:** `robot_id`와 `robot_type`이 올바르게 매칭되어 저장되는지 검증.
- **환경 변수 관리:** `docker-compose.yml`의 `POSTGRES_USER` 등은 민감 정보이므로 직접 하드코딩하지 않고 `.env` 파일 활용 권장.

---

**DB팀에게 전달할 핵심 메시지:**

> "우리는 현재 실시간으로 초당 수십 개의 로봇 패킷을 받고 있습니다. 이 데이터를 가장 효율적으로 조회하고, 향후 AI 학습팀이 데이터를 쉽게 뽑아갈 수 있도록 스키마를 설계해 주시는 것이 DB팀의 핵심 목표입니다."
>
