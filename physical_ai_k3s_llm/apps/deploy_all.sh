#!/bin/bash

# 색상 설정
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=== [1/4] Redis 서비스 배포 ===${NC}"
kubectl apply -f central-hub/manifests/redis-deployment.yaml
sleep 3  # Redis 안정화 대기

echo -e "${BLUE}=== [2/4] RBAC 및 Central-Hub 배포 ===${NC}"
kubectl apply -f central-hub/manifests/rbac.yaml
kubectl apply -f central-hub/manifests/central-hub.yaml
# Central Hub가 뜰 때까지 대기
kubectl wait --for=condition=ready pod -l app=central-hub --timeout=60s

echo -e "${BLUE}=== [3/4] Robot Detector(DaemonSet) 배포 ===${NC}"
kubectl apply -f robot-detector/manifests/detector-daemonset.yaml

echo -e "${GREEN}=== [4/4] 클러스터 배포 완료! ===${NC}"
echo -e "현재 파드 상태를 확인합니다..."
kubectl get pods -o wide