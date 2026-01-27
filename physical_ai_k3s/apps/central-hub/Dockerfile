FROM python:3.9-slim

# 1. 파이썬 로그가 버퍼링 없이 즉시 출력되도록 설정 (Loki 실시간 로그 확인용)
ENV PYTHONUNBUFFERED=1
# 2. OTEL이 라이브러리를 찾을 수 있도록 환경 변수 기본값 설정 (필요 시 YAML에서 덮어쓰기 가능)
ENV OTEL_SERVICE_NAME=central-hub

WORKDIR /app

# 3. 라이브러리 설치 (레이어 캐싱 활용)
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 4. 소스 코드 및 템플릿 복사
COPY main.py .
# manifests 폴더 구조 유지 (base, templates 등)
COPY manifests/ ./manifests/

# 5. Flask 기본 포트 및 OTEL 데이터 수집 포트 노출
EXPOSE 5000

CMD ["python", "main.py"]