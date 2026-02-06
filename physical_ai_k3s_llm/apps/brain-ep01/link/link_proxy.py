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

# 1. 인프라 및 환경 설정
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

# --- [시스템 프롬프트 구성: Few-shot 및 Armor ID 포함] ---
def get_system_prompt():
    return """
Role: RoboMaster EP01 Mission Controller.
Task: Generate Python code to control the robot via Redis LPUSH.

[Strict Rules]
1. Format: NO Markdown, NO explanation, ONLY Python code.
2. Target Key: f"robot:{os.environ['ROBOT_SN']}:commands"
3. Command Structure: Each command MUST be a single JSON object. DO NOT use .append() on a dict.
4. Armor ID Reference: 1:Back, 2:Front, 3:Left, 4:Right.

[Target/Action List]
- sensor: SUB_BATTERY, SUB_POS, SUB_ARMOR, SUB_SPEED, SUB_IMU, UNSUB
- chassis: MOVE (params: x,y,z,speed), ROTATE (params: yaw,v_speed)
- actuator: ARM_MOVE (params: arm_x, arm_y), GRIPPER (params: grip:open/close)
- led: SET (params: r,g,b,eff)
- vision: START, STOP (params: type:marker/line)

[Code Example]
import os, redis, json, time
r = redis.Redis(host=os.environ['REDIS_HOST'], port=6379, decode_responses=True)
key = f"robot:{os.environ['ROBOT_SN']}:commands"

# Multiple commands example:
cmds = [
    {"target": "sensor", "action": "SUB_BATTERY", "params": {}},
    {"target": "led", "action": "SET", "params": {"r": 0, "g": 255, "b": 0, "eff": "on"}}
]
for c in cmds:
    r.lpush(key, json.dumps(c))
print("MISSION_FINISHED")
"""

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

@app.route('/ask', methods=['POST'])
def handle_natural_language():
    data = request.json
    user_prompt = data.get("prompt")
    robot_sn = data.get("robot_sn", "ep01")

    try:
        full_prompt = f"{get_system_prompt()}\n\nUser Question: {user_prompt}\n\nPython Code:"
        payload = {"model": "llama3.2:3b", "prompt": full_prompt, "stream": False}
        
        response = requests.post(OLLAMA_URL, json=payload, timeout=30)
        raw_response = response.json().get('response', '')
        generated_code = raw_response.replace("```python", "").replace("```", "").strip()
        
    except Exception as e:
        logger.error(f"❌ [Ollama Error] {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

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
        
        wrapped_code = f"""
import os, redis, json, time
try:
    os.environ['ROBOT_SN'] = '{robot_sn}'
    os.environ['REDIS_HOST'] = '{REDIS_HOST}'
    {generated_code}
    print("LOG: Mission Logic Executed Successfully")
except Exception as e:
    print(f"ERROR: {{e}}")
    exit(1)
"""
        container['command'] = ["/bin/sh", "-c", f"pip install redis && python -c \"{wrapped_code}\""]
        v1.create_namespaced_pod(namespace="default", body=pod_manifest)

        monitor_thread = threading.Thread(target=monitor_and_cleanup, args=(task_id, robot_sn), daemon=True)
        monitor_thread.start()

        return jsonify({"status": "success", "mission_id": task_id, "generated_logic": generated_code})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)