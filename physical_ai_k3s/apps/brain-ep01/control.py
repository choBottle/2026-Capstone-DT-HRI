import os
import time
import logging
import json
from multiprocessing import Queue
from robomaster import robot
from prometheus_client import start_http_server, Gauge

# OpenTelemetry 라이브러리 (프로젝트 정체성 유지)
from opentelemetry import trace, metrics
from opentelemetry.sdk.resources import Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.exporter.otlp.proto.grpc.metric_exporter import OTLPMetricExporter
from opentelemetry.sdk.metrics import MeterProvider
from opentelemetry.sdk.metrics.export import PeriodicExportingMetricReader

# [변경점 1] Loki 직접 전송 핸들러 (인프라 에러 우회용)
from logging_loki import LokiQueueHandler

# 1. 설정 정보
robot_ip = os.getenv("ROBOT_IP", "192.168.2.1")
robot_sn = os.getenv("ROBOT_SN", "UNKNOWN")
COLLECTOR_ENDPOINT = "10.43.4.211:4317" 
LOKI_URL = "http://10.43.4.211:3100/loki/api/v1/push" # 내부 주소

resource = Resource.create({
    "service.name": "brain-ep01",
    "robot.id": robot_sn,
    "deployment.environment": "k3s-production"
})

# [변경점 2] 로그 핸들러 설정
# AI 판단 내용을 담을 JSON 로그 전송용
logger = logging.getLogger("robot-ai")
logger.setLevel(logging.INFO)
loki_handler = LokiQueueHandler(
    Queue(-1),
    url=LOKI_URL,
    tags={"target_robot": robot_sn, "robot_family": "EP01"},
    version="1",
)
logger.addHandler(loki_handler)

# 2. OTel 메트릭/트레이스 (이 부분은 OTel을 그대로 사용함!)
metric_reader = PeriodicExportingMetricReader(OTLPMetricExporter(endpoint=COLLECTOR_ENDPOINT, insecure=True))
meter_provider = MeterProvider(metric_readers=[metric_reader], resource=resource)
metrics.set_meter_provider(meter_provider)
meter = metrics.get_meter(__name__)
battery_gauge = meter.create_gauge("robot_battery_percent", unit="%", description="배터리 잔량")

trace_provider = TracerProvider(resource=resource)
trace_provider.add_span_processor(BatchSpanProcessor(OTLPSpanExporter(endpoint=COLLECTOR_ENDPOINT, insecure=True)))
trace.set_tracer_provider(trace_provider)
tracer = trace.get_tracer(__name__)

def battery_info_handler(level):
    # OTel 메트릭 업데이트
    battery_gauge.set(level, {"robot_id": robot_sn})

def start_control():
    start_http_server(8000)
    ep_robot = robot.Robot()
    
    # [AI 로그] 가동 시작 알림
    logger.info(json.dumps({"level": "INFO", "message": "AI System Started", "robot_id": robot_sn}))

    with tracer.start_as_current_span("robot-mission-root") as span:
        try:
            res = ep_robot.initialize(conn_type="sta", proto_type="tcp")
            if res == 0 or res is True:
                ep_robot.battery.sub_battery_info(freq=5, callback=battery_info_handler)
                
                for i in range(5):
                    # [변경점 3] AI 데이터 수집 및 로그 생성
                    # AI 모델이 나중에 학습할 수 있도록 로그에 JSON 형태로 저장
                    log_data = {
                        "level": "INFO",
                        "message": f"Step {i} Data",
                        "battery": 100 - (i*5), # 가상 배터리 수치
                        "cpu_load": 15.5
                    }
                    logger.info(json.dumps(log_data)) # Loki로 직접 쏨
                    
                    span.add_event(f"Step-{i}", {"battery": 100-(i*5)}) # Tempo로 쏨
                    time.sleep(1)
                
                trace_provider.shutdown()
        finally:
            ep_robot.close()
            logger.info(json.dumps({"level": "INFO", "message": "Mission Finished"}))

if __name__ == "__main__":
    start_control()