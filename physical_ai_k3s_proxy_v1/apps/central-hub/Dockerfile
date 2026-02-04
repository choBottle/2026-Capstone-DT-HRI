FROM python:3.11-slim

RUN apt-get update && apt-get install -y gcc python3-dev && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY mission_orchestrator.py .
COPY templates/ ./templates/

ENV PYTHONUNBUFFERED=1
# 실행 시 API_KEY와 REDIS_HOST는 K8s Deployment에서 주입받습니다.

EXPOSE 5000
CMD ["python", "mission_orchestrator.py"]