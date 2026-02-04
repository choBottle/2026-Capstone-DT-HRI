import os, yaml, time, json, redis, logging
from flask import Flask, request, jsonify
from kubernetes import client, config

app = Flask(__name__)
# Redis 연결: 디코딩 설정을 추가하여 데이터 읽기 편하게 설정
r = redis.Redis(host=os.getenv("REDIS_HOST", "redis-service"), port=6379, decode_responses=True)

# K8s 클러스터 내부 설정 로드
config.load_incluster_config()
v1 = client.CoreV1Api()

TEMPLATE_PATH = "templates/task-pod-tpl.yaml"

@app.route('/detect', methods=['POST'])
def robot_detected():
    """
    [고민 1 해결] Detector로부터 로봇 감지 보고를 받아 Link Pod를 자동 배포함
    """
    data = request.json
    robot_ip = data.get("ip")
    robot_sn = data.get("raw_data") # Detector가 보낸 SN
    node_id = data.get("node_id")   # 감지된 라즈베리파이 호스트네임

    # 1. 중복 연결 방지 (이미 해당 로봇에 대한 Link Pod가 있다면 무시)
    if r.exists(f"link_status:{robot_sn}"):
        return jsonify({"status": "ignored", "message": "Already linked"}), 200

    # 2. Link Pod 이름 설정 (소문자 및 8자리 SN 사용)
    link_id = f"link-{robot_sn.lower()[:8]}"

    with open(TEMPLATE_PATH) as f:
        pod_manifest = yaml.safe_load(f)

    # 3. 매니페스트 동적 수정 (Link Pod 전용)
    pod_manifest['metadata']['name'] = link_id
    # [핵심] 감지된 노드(pi1 등)로 배포 위치 고정
    pod_manifest['spec']['nodeSelector'] = {"kubernetes.io/hostname": node_id}
    
    container = pod_manifest['spec']['containers'][0]
    container['image'] = "jny123/ep01-link:v6" # Link 전용 이미지
    container['env'] = [
        {"name": "ROBOT_IP", "value": robot_ip},
        {"name": "ROBOT_SN", "value": robot_sn},
        {"name": "REDIS_HOST", "value": "redis-service"}
    ]

    try:
        v1.create_namespaced_pod(namespace="default", body=pod_manifest)
        r.set(f"link_status:{robot_sn}", node_id) # Redis에 현재 연결 노드 기록
        print(f"🔗 [Link-Up] Robot {robot_sn} connected to node {node_id}")
        return jsonify({"status": "success", "action": "link_deployed"}), 201
    except Exception as e:
        print(f"❌ Link Pod 배포 실패: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500

@app.route('/test-deploy', methods=['POST'])
def deploy_task():
    """
    기존 임무 배포 로직: Link Pod가 이미 있다는 가정하에 명령만 전달하는 Task Pod 배포
    """
    data = request.json
    task_type = data.get("task_type")
    robot_sn = data.get("robot_sn", "ep01")
    params = data.get("params", {})

    start_time = time.time()
    task_id = f"{task_type}-{int(start_time * 1000)}"

    r.set(f"task_config:{task_id}", json.dumps(params))

    with open(TEMPLATE_PATH) as f:
        pod_manifest = yaml.safe_load(f)

    pod_manifest['metadata']['name'] = task_id
    
    # [핸드오버 준비] 현재 로봇이 연결된 노드를 Redis에서 찾아 해당 노드에 임무 파드 배포
    current_node = r.get(f"link_status:{robot_sn}")
    if current_node:
        pod_manifest['spec']['nodeSelector'] = {"kubernetes.io/hostname": current_node}

    container = pod_manifest['spec']['containers'][0]
    image_map = {
        "vision": "jny123/vision-worker:latest",
        "navigation": "jny123/nav-worker:latest"
    }
    container['image'] = image_map.get(task_type, "jny123/ep01-link:latest")

    container['env'] = [
        {"name": "REDIS_HOST", "value": "redis-service"},
        {"name": "ROBOT_SN", "value": robot_sn},
        {"name": "TASK_ID", "value": task_id}
    ]

    try:
        v1.create_namespaced_pod(namespace="default", body=pod_manifest)
        dispatch_latency = (time.time() - start_time) * 1000
        return jsonify({"status": "success", "task_id": task_id, "latency_ms": dispatch_latency})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)