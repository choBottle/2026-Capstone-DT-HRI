import os
import yaml
import time
import logging
from flask import Flask, request, jsonify
from kubernetes import client, config

# --- OpenTelemetry 관련 라이브러리 ---
from opentelemetry import trace, _logs
from opentelemetry.sdk.resources import Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor
from opentelemetry.sdk._logs import LoggerProvider, LoggingHandler
from opentelemetry.sdk._logs.export import BatchLogRecordProcessor
from opentelemetry.exporter.otlp.proto.grpc.trace_exporter import OTLPSpanExporter
from opentelemetry.exporter.otlp.proto.grpc._log_exporter import OTLPLogExporter
from opentelemetry.instrumentation.flask import FlaskInstrumentor

# 1. 공통 리소스 설정 (서비스 이름 및 식별자)
resource = Resource.create({
    "service.name": "central-hub",
    "node.id": "central-laptop"
})

# 2. OpenTelemetry Tracing(Tempo) 설정
trace_provider = TracerProvider(resource=resource)
otlp_trace_exporter = OTLPSpanExporter(
    endpoint="monitoring-service.monitoring:4317", 
    insecure=True
)
trace_provider.add_span_processor(BatchSpanProcessor(otlp_trace_exporter))
trace.set_tracer_provider(trace_provider)
tracer = trace.get_tracer(__name__)

# 3. OpenTelemetry Logging(Loki) 설정 [추가됨]
logger_provider = LoggerProvider(resource=resource)
_logs.set_logger_provider(logger_provider)
otlp_log_exporter = OTLPLogExporter(
    endpoint="monitoring-service.monitoring:4317", 
    insecure=True
)
logger_provider.add_log_record_processor(BatchLogRecordProcessor(otlp_log_exporter))

# 파이썬 표준 logging을 OTEL 로깅과 연결
handler = LoggingHandler(level=logging.INFO, logger_provider=logger_provider)
logging.getLogger().addHandler(handler)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# 4. Flask 앱 설정 및 자동 계측
app = Flask(__name__)
FlaskInstrumentor().instrument_app(app)

# K8s 설정 로드
try:
    config.load_incluster_config()
except Exception:
    config.load_kube_config()

v1 = client.CoreV1Api()
TEMPLATE_DIR = "/app/manifests/templates" 

@app.route('/detect', methods=['POST'])
def detect_robot():
    # 전체 탐지 프로세스를 하나의 큰 트레이스로 묶음
    with tracer.start_as_current_span("robot-detection-pipeline") as root_span:
        data = request.json
        logger.info(f"📡 수신 데이터: {data}")

        # 1. 데이터 추출 및 노드 매핑
        with tracer.start_as_current_span("process-metadata"):
            robot_type = data.get('robot_type', 'ep01')
            robot_ip = data.get('ip')
            raw_node_id = data.get('node_id', 'pi1')
            raw_data = data.get('raw_data', 'unknown').replace('\x00', '').strip()

            target_node = "pi1" if "pi-unit" in raw_node_id or "pi1" in raw_node_id else raw_node_id
            
            # 스팬에 라벨 추가 (Grafana에서 검색 가능)
            root_span.set_attribute("robot.type", robot_type)
            root_span.set_attribute("target.node", target_node)

        # 2. 템플릿 파일 경로 확인 및 로드
        with tracer.start_as_current_span("load-template"):
            template_path = os.path.join(TEMPLATE_DIR, f"{robot_type}-pod-tpl.yaml")
            if not os.path.exists(template_path):
                logger.error(f"❌ 템플릿 없음: {template_path}")
                return jsonify({"status": "error", "message": f"Template not found"}), 404
            
            with open(template_path, 'r') as f:
                pod_manifest = yaml.safe_load(f)

        # 3. 파드 명세 구성
        with tracer.start_as_current_span("build-manifest"):
            timestamp = int(time.time())
            pod_name = f"robot-{robot_type}-{timestamp}"
            pod_manifest['metadata']['name'] = pod_name
            
            # nodeSelector 및 환경변수 주입
            if 'spec' not in pod_manifest: pod_manifest['spec'] = {}
            pod_manifest['spec']['nodeSelector'] = {'kubernetes.io/hostname': target_node}

            container = pod_manifest['spec']['containers'][0]
            container['env'] = [
                {'name': 'ROBOT_IP', 'value': robot_ip},
                {'name': 'ROBOT_SN', 'value': raw_data}
            ]

        # 4. K8s API 호출 (실제 파드 생성)
        with tracer.start_as_current_span("k8s-api-create-pod"):
            try:
                v1.create_namespaced_pod(namespace="default", body=pod_manifest)
                logger.info(f"🚀 [성공] 노드 '{target_node}'에 파드 생성 완료: {pod_name}")
                return jsonify({"status": "success", "pod_name": pod_name}), 200
            except Exception as e:
                root_span.record_exception(e) # 트레이스에 에러 기록
                logger.error(f"❌ 파드 생성 실패: {str(e)}") # Loki에 에러 로그 전송
                return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == "__main__":
    logger.info("Starting Central Hub server...")
    app.run(host='0.0.0.0', port=5000)