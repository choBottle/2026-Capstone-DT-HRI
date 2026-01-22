# 1. 베이스 이미지: 라즈베리 파이(ARM64) 최적화 버전
FROM arm64v8/python:3.9-slim

# 2. 작업 디렉토리 설정
WORKDIR /app

# 3. 필수 시스템 패키지 설치 (OpenCV 및 빌드 도구)
# gcc, python3-dev: SocketIO 및 일부 라이브러리 컴파일용
# libglib...: OpenCV 실행용
RUN apt-get update && apt-get install -y \
    gcc \
    python3-dev \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

# 4. 라이브러리 설치
# (requirements.txt가 같은 폴더에 있어야 합니다!)
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 5. 소스 코드 복사
COPY . .

# 6. 실행 명령어
# -u 옵션: 파이썬 로그가 버퍼링 없이 즉시 출력되게 함 (디버깅 필수)
CMD ["python", "-u", "main.py"]
