import os
import yaml
import time
import json
import redis
import logging
import threading
import requests
from flask import Flask, request, jsonify
from kubernetes import client, config, watch

app = Flask(__name__)
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("Central-Hub")

REDIS_HOST = os.getenv("REDIS_HOST", "redis-service")
OLLAMA_HOST = os.getenv("OLLAMA_HOST", "192.168.50.39") 
OLLAMA_URL = f"http://{OLLAMA_HOST}:11434/api/generate"

try:
    r = redis.Redis(host=REDIS_HOST, port=6379, decode_responses=True)
    config.load_incluster_config()
    v1 = client.CoreV1Api()
    logger.info(f"🎯 Local LLM Mode: Ollama at {OLLAMA_URL}")
except Exception as e:
    logger.error(f"❌ 초기화 중 에러 발생: {e}")

TEMPLATE_PATH = "templates/task-pod-tpl.yaml"

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

def get_system_prompt():
    return """
Role: Python Code Generator for RoboMaster EP01.
Task: Write MINIMAL Python code to send Redis LPUSH commands.

[STRICT RULES - DO NOT VIOLATE]
1. NO Markdown (NO ```python).
2. NO explanations. NO comments.
3. NO functions (def) or classes (class). Just sequential lines of code.
4. ONLY use 'os', 'redis', 'json', 'time'.
5. Use this EXACT Redis setup:
r = redis.Redis(host=os.environ['REDIS_HOST'], port=6379, decode_responses=True)
k = f"robot:{os.environ['ROBOT_SN']}:commands"

[COMMAND PROTOCOL]
Each command MUST be: r.lpush(k, json.dumps({"target": "TARGET", "action": "ACTION", "params": {PARAMS}}))

[TARGET/ACTION LIST]
- sensor: SUB_BATTERY, SUB_POS, SUB_ARMOR, SUB_SPEED, SUB_IMU, UNSUB
- chassis: MOVE (x,y,z,speed), ROTATE (yaw,v_speed)
- actuator: ARM_MOVE (arm_x, arm_y), GRIPPER (grip:open/close)
- led: SET (r,g,b,eff:"on"/"off"/"flash")
- vision: START, STOP (type:marker/line)

[ARMOR ID] 1:Back, 2:Front, 3:Left, 4:Right.

[EXAMPLE OUTPUT]
import os, redis, json, time
r = redis.Redis(host=os.environ['REDIS_HOST'], port=6379, decode_responses=True)
k = f"robot:{os.environ['ROBOT_SN']}:commands"
r.lpush(k, json.dumps({"target": "sensor", "action": "SUB_BATTERY", "params": {}}))
r.lpush(k, json.dumps({"target": "led", "action": "SET", "params": {"r":0,"g":255,"b":0,"eff":"on"}}))
print("MISSION_FINISHED")
"""

# --- [기존에 잘 작동하던 detect 함수 유지] ---
@app.route('/detect', methods=['POST'])
def robot_detected():
    data = request.json
    robot_ip = data.get("ip")
    robot_sn = data.get("raw_data")
    node_id = data.get("node_id")

    if r.exists(f"link_status:{robot_sn}"):
        return jsonify({"status": "ignored", "message": "Already linked"}), 200

    link_id = f"link-{robot_sn.lower()[:8]}"
    try:
        with open(TEMPLATE_PATH) as f:
            pod_manifest = yaml.safe_load(f)

        pod_manifest['metadata']['name'] = link_id
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
        return jsonify({"status": "success", "action": "link_deployed"}), 201
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

# --- [인용구 문제만 해결한 ask 함수] ---
@app.route('/ask', methods=['POST'])
def handle_natural_language():
    data = request.json
    user_prompt = data.get("prompt")
    robot_sn = data.get("robot_sn", "ep01")

    try:
        full_prompt = f"{get_system_prompt()}\n\nUser Question: {user_prompt}\n\nPython Code:"
        payload = {"model": "llama3.2:3b", "prompt": full_prompt, "stream": False}
        response = requests.post(OLLAMA_URL, json=payload, timeout=300)
        generated_code = response.json().get('response', '').replace("```python", "").replace("```", "").strip()
    except Exception as e:
        return jsonify({"status": "error", "message": f"Ollama Error: {e}"}), 500

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
        
        # [핵심 수정] 코드를 쉘 명령줄이 아닌 환경변수에 넣어 인용구 충돌 차단
        container['env'] = [
            {"name": "REDIS_HOST", "value": REDIS_HOST},
            {"name": "ROBOT_SN", "value": robot_sn},
            {"name": "MISSION_CODE", "value": generated_code} 
        ]
        
        # 쉘 명령줄에는 고정된 홑따옴표가 없는 코드만 사용
        launcher = "import os; exec(os.environ.get('MISSION_CODE', ''))"
        container['command'] = ["/bin/sh", "-c", f"pip install redis && python -c \"{launcher}\""]

        v1.create_namespaced_pod(namespace="default", body=pod_manifest)
        
        monitor_thread = threading.Thread(target=monitor_and_cleanup, args=(task_id, robot_sn), daemon=True)
        monitor_thread.start()

        return jsonify({"status": "success", "mission_id": task_id, "generated_logic": generated_code})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)