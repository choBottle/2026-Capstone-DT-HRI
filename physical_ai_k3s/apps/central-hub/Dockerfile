FROM python:3.11-slim

# 인터랙티브 모드 방지 및 로그 즉시 출력을 위한 설정
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# 필수 시스템 라이브러리 (K8s API 및 Python 패키지 빌드용)
RUN apt-get update && apt-get install -y \
    gcc \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# 1. 의존성 파일 복사 및 설치
COPY requirements.txt .
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# 2. 소스 코드 및 템플릿/정책 파일 복사
# 정책 파일(mission_policy.yaml)이 있어야 LLM이 정확한 가이드를 읽습니다.
COPY mission_orchestrator.py .
COPY mission_policy.yaml .
COPY templates/ ./templates/

# Flask API 포트 개방
EXPOSE 5000

# 실행 시 API_KEY와 REDIS_HOST는 K8s Deployment의 env에서 주입받음
CMD ["python", "mission_orchestrator.py"]