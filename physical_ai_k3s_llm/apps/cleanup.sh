#!/bin/bash

RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${RED}=== [1/4] 모든 관련 파드 강제 삭제 (패턴 매칭) ===${NC}"
# 1. 레이블 기준 삭제
kubectl delete pods -l type=robot-task,app=mission-task --grace-period=0 --force 2>/dev/null

# 2. 이름 패턴(link-, mission-) 기준 삭제 (레이블이 없는 경우 대비)
PODS_TO_KILL=$(kubectl get pods -o custom-columns=:.metadata.name --no-headers | grep -E "^link-|^mission-")
if [ ! -z "$PODS_TO_KILL" ]; then
    echo -e "${YELLOW}발견된 좀비 파드 삭제 중: $PODS_TO_KILL${NC}"
    kubectl delete pods $PODS_TO_KILL --grace-period=0 --force 2>/dev/null
fi

echo -e "${RED}=== [2/4] 핵심 인프라 매니페스트 제거 ===${NC}"
# 매니페스트 기반 삭제 (설정파일에 정의된 모든 자원 포함)
kubectl delete -f robot-detector/manifests/detector-daemonset.yaml --ignore-not-found
kubectl delete -f central-hub/manifests/central-hub.yaml --ignore-not-found
kubectl delete -f central-hub/manifests/rbac.yaml --ignore-not-found

echo -e "${YELLOW}=== [3/4] Redis 데이터 초기화 및 서비스 제거 ===${NC}"
REDIS_POD=$(kubectl get pods -l app=redis -o jsonpath="{.items[0].metadata.name}" 2>/dev/null)
if [ ! -z "$REDIS_POD" ]; then
    echo "Cleaning up Redis data..."
    kubectl exec $REDIS_POD -- redis-cli FLUSHALL 2>/dev/null
fi
kubectl delete -f central-hub/manifests/redis-deployment.yaml --ignore-not-found

echo -e "${RED}=== [4/4] 최종 정리 (Finalizers 제거) ===${NC}"
# Terminating 상태에서 안 넘어가는 파드들을 위해 finalizers를 강제로 비웁니다.
STUCK_PODS=$(kubectl get pods | grep Terminating | awk '{print $1}')
for pod in $STUCK_PODS; do
    kubectl patch pod $pod -p '{"metadata":{"finalizers":null}}' --type merge 2>/dev/null
done

echo -e "${GREEN}--- [완료] 클러스터가 완전히 깨끗해졌습니다. ---${NC}"
kubectl get pods