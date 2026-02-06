FROM python:3.8-slim

WORKDIR /app

# psutil 설치를 위한 gcc 등 최소 빌드 도구 포함
RUN apt-get update && apt-get install -y \
    gcc \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY detector.py .

ENV PYTHONUNBUFFERED=1

# 탐지기는 항상 실행되어야 함
CMD ["python", "detector.py"]