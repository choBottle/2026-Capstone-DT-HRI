import os
import yaml
import time
import json
import redis
import logging
import threading
from flask import Flask, request, jsonify
from kubernetes import client, config, watch
import google.generativeai as genai

app = Flask(__name__)
# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("Central-Hub")

# 1. 인프라 및 환경 설정
REDIS_HOST = os.getenv("REDIS_HOST", "redis-service")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

try:
    r = redis.Redis(host=REDIS_HOST, port=6379, decode_responses=True)
    # K8s 내부에서 실행될 때의 설정 로드
    config.load_incluster_config()
    v1 = client.CoreV1Api()
    
    # [수정] Gemini LLM 초기화 - 모델명 앞에 models/ 추가하여 404 방지
    genai.configure(api_key=GEMINI_API_KEY)
    model = genai.GenerativeModel('models/gemini-2.0-flash')
    
    logger.info("🎯 Selected Model: models/gemini-2.0-flash")
except Exception as e:
    logger.error(f"❌ 초기화 중 에러 발생: {e}")

TEMPLATE_PATH = "templates/task-pod-tpl.yaml"
POLICY_PATH = "mission_policy.yaml"

# --- [비동기 모니터링 및 뒷정리 함수] ---
def monitor_and_cleanup(pod_name, robot_sn):
    logger.info(f"👀 [Monitor] Starting watch for pod: {pod_name}")
    w = watch.Watch()
    try:
        for event in w.stream(v1.list_namespaced_pod, namespace="default", label_selector=f"mission_id={pod_name}"):
            pod_status = event['object'].status.phase
            if pod_status in ["Succeeded", "Failed"]:
                logger.info(f"🏁 [Finish] Pod {pod_name} terminated with status: {pod_status}")
                log_data = {
                    "pod_name": pod_name,
                    "status": pod_status,
                    "completed_at": time.strftime('%Y-%m-%d %H:%M:%S')
                }
                r.hset(f"mission_history:{robot_sn}", pod_name, json.dumps(log_data))
                v1.delete_namespaced_pod(name=pod_name, namespace="default")
                logger.info(f"🗑️ [Cleanup] Pod {pod_name} has been removed.")
                w.stop()
                break
    except Exception as e:
        logger.error(f"❌ [Monitor Error] Error during {pod_name} cleanup: {e}")

# --- [시스템 프롬프트 구성] ---
def get_system_prompt():
    try:
        with open(POLICY_PATH, 'r') as f:
            policy = f.read()
    except FileNotFoundError:
        policy = "Mission policy file not found."

    return f"""
    너는 RoboMaster EP01의 미션 제어관이다. 사용자의 자연어 명령을 아래 규격에 맞는 Python 코드로 번역하라.
    [규칙]
    - 반드시 실행 가능한 '완전한 Python 코드'만 출력하라. (설명 금지, 마크다운 기호 금지)
    - Redis 'robot:<SN>:commands' 키에 JSON 명령어를 LPUSH 한다.
    - 로직 마지막에는 반드시 print("MISSION_FINISHED")를 포함하라.
    [사양서 가이드]
    {policy}
    """

# --- [API 엔드포인트: 로봇 감지 및 Link Proxy 배포] ---
@app.route('/detect', methods=['POST'])
def robot_detected():
    data = request.json
    robot_ip = data.get("ip")
    robot_sn = data.get("raw_data")
    node_id = data.get("node_id")

    if r.exists(f"link_status:{robot_sn}"):
        logger.info(f"ℹ️ Robot {robot_sn} already has a Link Proxy.")
        return jsonify({"status": "ignored", "message": "Already linked"}), 200

    link_id = f"link-{robot_sn.lower()[:8]}"
    
    try:
        with open(TEMPLATE_PATH) as f:
            pod_manifest = yaml.safe_load(f)

        pod_manifest['metadata']['name'] = link_id
        # [수정] 브로드캐스트 및 DNS 해결을 위한 설정
        pod_manifest['spec']['hostNetwork'] = True
        pod_manifest['spec']['dnsPolicy'] = "ClusterFirstWithHostNet"
        pod_manifest['spec']['nodeSelector'] = {"kubernetes.io/hostname": node_id}
        
        container = pod_manifest['spec']['containers'][0]
        container['image'] = "jny123/ep01-link:v11"
        
        if 'command' in container: del container['command']
        if 'args' in container: del container['args']

        container['env'] = [
            {"name": "ROBOT_IP", "value": robot_ip},
            {"name": "ROBOT_SN", "value": robot_sn},
            {"name": "REDIS_HOST", "value": REDIS_HOST}
        ]

        v1.create_namespaced_pod(namespace="default", body=pod_manifest)
        r.set(f"link_status:{robot_sn}", node_id)
        logger.info(f"🔗 [Link-Up] Successfully deployed Link Proxy for {robot_sn} on {node_id}")
        return jsonify({"status": "success", "action": "link_deployed"}), 201

    except Exception as e:
        logger.error(f"❌ [Detect Error] Failed to deploy Link Proxy: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

# --- [API 엔드포인트: 자연어 미션 요청 처리] ---
@app.route('/ask', methods=['POST'])
def handle_natural_language():
    data = request.json
    user_prompt = data.get("prompt")
    robot_sn = data.get("robot_sn", "ep01")

    try:
        full_prompt = f"{get_system_prompt()}\n\n사용자 명령: {user_prompt}"
        response = model.generate_content(full_prompt)
        generated_code = response.text.replace("```python", "").replace("```", "").strip()
    except Exception as e:
        logger.error(f"❌ [LLM Error] Failed to translate prompt: {e}")
        return jsonify({"status": "error", "message": f"LLM Error: {e}"}), 500

    task_id = f"mission-{int(time.time())}"
    try:
        with open(TEMPLATE_PATH) as f:
            pod_manifest = yaml.safe_load(f)

        pod_manifest['metadata']['name'] = task_id
        pod_manifest['metadata']['labels'] = {"app": "mission-task", "mission_id": task_id}
        
        current_node = r.get(f"link_status:{robot_sn}")
        if current_node:
            pod_manifest['spec']['nodeSelector'] = {"kubernetes.io/hostname": current_node}

        container = pod_manifest['spec']['containers'][0]
        container['image'] = "python:3.9-slim"
        
        if 'args' in container: del container['args']
        
        wrapped_code = f"""
import os, redis, json, time
try:
    {generated_code}
    print("LOG: Mission Logic Executed Successfully")
except Exception as e:
    print(f"ERROR: Runtime exception occurred: {{e}}")
    exit(1)
"""
        container['command'] = ["/bin/sh", "-c", f"pip install redis && python -c '{wrapped_code}'"]
        container['env'] = [{"name": "REDIS_HOST", "value": REDIS_HOST}, {"name": "ROBOT_SN", "value": robot_sn}]

        v1.create_namespaced_pod(namespace="default", body=pod_manifest)
        logger.info(f"🚀 [Mission-Start] Deployed task {task_id} for {robot_sn}")

        monitor_thread = threading.Thread(target=monitor_and_cleanup, args=(task_id, robot_sn), daemon=True)
        monitor_thread.start()

        return jsonify({"status": "success", "mission_id": task_id, "node": current_node, "generated_logic": generated_code})

    except Exception as e:
        logger.error(f"❌ [K8s Error] Failed to deploy mission pod: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)