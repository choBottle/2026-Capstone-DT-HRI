import os
import yaml
import time
from flask import Flask, request, jsonify
from kubernetes import client, config

app = Flask(__name__)

# K8s 설정 로드
try:
    config.load_incluster_config()
except Exception:
    config.load_kube_config()

v1 = client.CoreV1Api()
TEMPLATE_DIR = "/app/manifests/templates" 

@app.route('/detect', methods=['POST'])
def detect_robot():
    data = request.json
    print(f"📡 수신 데이터: {data}")

    # 1. 데이터 추출
    robot_type = data.get('robot_type', 'ep01')
    robot_ip = data.get('ip')
    raw_node_id = data.get('node_id', 'pi1') # 예: 'pi-unit-01'
    raw_data = data.get('raw_data', 'unknown').replace('\x00', '').strip()

    # [핵심 수정] 실제 K8s 노드 이름(pi1)으로 매핑
    # detector가 'pi-unit-01'이라고 보내도 실제 노드 이름인 'pi1'으로 변경합니다.
    target_node = "pi1" if "pi-unit" in raw_node_id or "pi1" in raw_node_id else raw_node_id

    # 2. 템플릿 파일 경로 확인
    template_path = os.path.join(TEMPLATE_DIR, f"{robot_type}-pod-tpl.yaml")
    if not os.path.exists(template_path):
        print(f"❌ 템플릿 없음: {template_path}")
        return jsonify({"status": "error", "message": f"Template not found: {template_path}"}), 404

    try:
        # 3. 템플릿 로드
        with open(template_path, 'r') as f:
            pod_manifest = yaml.safe_load(f)

        # 4. 동적 설정 주입
        # 파드 이름은 중복 방지를 위해 타임스탬프를 포함하고 소문자로 유지
        timestamp = int(time.time())
        pod_name = f"robot-{robot_type}-{timestamp}"
        pod_manifest['metadata']['name'] = pod_name
        
        # [중요] nodeSelector를 실제 노드 이름인 pi1으로 강제 지정
        if 'spec' not in pod_manifest: pod_manifest['spec'] = {}
        pod_manifest['spec']['nodeSelector'] = {
            'kubernetes.io/hostname': target_node
        }

        # 5. 환경변수 주입 (로봇 IP 및 SN)
        container = pod_manifest['spec']['containers'][0]
        container['env'] = [
            {'name': 'ROBOT_IP', 'value': robot_ip},
            {'name': 'ROBOT_SN', 'value': raw_data}
        ]

        # 6. 파드 생성
        v1.create_namespaced_pod(namespace="default", body=pod_manifest)
        print(f"🚀 [성공] 노드 '{target_node}'에 파드 생성 완료: {pod_name}")
        return jsonify({"status": "success", "pod_name": pod_name, "assigned_node": target_node}), 200

    except Exception as e:
        print(f"❌ 파드 생성 실패: {str(e)}")
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)