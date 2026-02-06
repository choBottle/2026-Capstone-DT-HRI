# 라즈베리파이(ARM)와 메인PC(x86) 호환을 위한 slim 버전
FROM python:3.8-slim

# 로그 버퍼링 방지 및 파이썬 경로 설정
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

# 필수 시스템 라이브러리 설치
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    gcc \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# 1. 의존성 파일 복사 및 설치
COPY requirements.txt .
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir setuptools==58.2.0 && \
    pip install --no-cache-dir -r requirements.txt

# 2. 소스 코드 복사 (현재 폴더의 모든 파일을 복사)
COPY . /app/

# 3. 실행 권한 부여
RUN chmod +x /app/link_proxy.py

# 4. 실행 (절대 경로 사용으로 경로 문제 원천 차단)
CMD ["python", "-u", "/app/link_proxy.py"]