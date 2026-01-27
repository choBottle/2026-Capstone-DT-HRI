# 1. SDK 호환성을 위해 Python 3.8 Full 버전 유지
FROM python:3.8

# 2. OpenCV 및 SDK 구동에 필요한 시스템 라이브러리 설치
# libgl1-mesa-glx와 libglib2.0-0이 cv2 로드 시 가장 중요합니다.
RUN apt-get update && apt-get install -y \
    gcc \
    python3-dev \
    libffi-dev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# 3. 라이브러리 설치 순서 (의존성 꼬임 방지)
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir numpy==1.23.5 && \
    pip install --no-cache-dir opencv-python-headless && \
    pip install --no-cache-dir robomaster && \
    pip install --no-cache-dir \
    prometheus-client \
    opentelemetry-api \
    opentelemetry-sdk \
    opentelemetry-exporter-otlp-proto-grpc

# 4. 소스 복사 및 설정
COPY control.py .
ENV PYTHONUNBUFFERED=1

CMD ["python", "control.py"]