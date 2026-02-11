## 라즈베리파이 노드 추가 가이드

이 과정은 **새로운 라즈베리파이에 OS를 막 설치한 직후**부터 시작한다고 가정합니다.

### 1. 시스템 기본 최적화 (로케일 & 타임존)

접속하자마자 터미널이 지저분해지는 것을 방지하고 시간을 맞춥니다.

```bash
# 로케일 일괄 설치 및 설정
sudo apt-get update && sudo apt-get install -y locales-all
sudo localectl set-locale LANG=en_US.UTF-8

# 타임존 설정 (한국 시간)
sudo timedatectl set-timezone Asia/Seoul
```

### 2. K8s 필수 설정 (cgroup 활성화)

이 설정을 안 하면 K3s Agent가 실행되지 않습니다.

```bash
# 부팅 옵션 수정
# /boot/firmware/cmdline.txt (신규 OS) 또는 /boot/cmdline.txt (구형 OS)
sudo sed -i '$ s/$/ cgroup_memory=1 cgroup_enable=memory/' /boot/firmware/cmdline.txt

# 설정 적용을 위한 재부팅
sudo reboot
```

### 3. K3s Agent 설치 및 클러스터 연결

재부팅 후, **중앙 서버(Server)**에서 토큰과 IP를 확인한 뒤 새 노드에서 실행합니다.

- **중앙 서버에서 확인:**

```bash
 sudo cat /var/lib/rancher/k3s/server/node-token
```

- **새 노드(Agent)에서 실행:**

```bash
# 변수 설정 (본인 환경에 맞게 수정)
export K3S_URL="https://<SERVER_IP>:6443"
export K3S_TOKEN="<NODE_TOKEN_VALUE>"

# 에이전트 설치
curl -sfL https://get.k3s.io | K3S_URL=$K3S_URL K3S_TOKEN=$K3S_TOKEN sh -
```

### 4. 연결 완료 확인 노드 역할 부여 (라벨링)

연결이 완료되면 **중앙 서버(PC)**에서 노드가 어떤 역할을 할지 명시합니다. 이 라벨을 기반으로 우리가 짠 스케줄러가 파드를 배정합니다.

```bash
# 노드 이름 확인 (예: pi3)
kubectl get nodes

# 라벨 부여(선택, 현재는 없음)
kubectl label node <노드이름> role=edge
kubectl label node <노드이름> device-type=raspberrypi5
```
