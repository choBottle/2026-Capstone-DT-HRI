FROM python:3.9-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY main.py .
# manifests 폴더(templates 포함)를 통째로 복사
COPY manifests/ ./manifests/

EXPOSE 5000
CMD ["python", "main.py"]