import redis
import json
import os
import time

class NavWorker:
    def __init__(self):
        # Redis 연결 설정
        self.r = redis.Redis(
            host=os.getenv("REDIS_HOST", "redis-service"), 
            port=6379, 
            decode_responses=True
        )
        self.robot_id = os.getenv("ROBOT_SN", "ep01")
        self.task_id = os.getenv("TASK_ID", "nav_task_01")
        
    def run(self):
        print(f"[*] Navigation Worker ({self.task_id}) started for {self.robot_id}")
        
        # 1. LLM이 설정한 Task 정보 읽기 (예: {"target_x": 1.5, "target_y": 0.5})
        task_config = json.loads(self.r.get(f"task_config:{self.task_id}") or "{}")
        tx = task_config.get("target_x", 0)
        ty = task_config.get("target_y", 0)

        # 2. Link Proxy에게 이동 명령 발행 (명령 큐에 삽입)
        move_cmd = {
            "target": "chassis",
            "action": "MOVE",
            "params": {"x": tx, "y": ty, "z": 0, "spd": 0.5}
        }
        self.r.rpush(f"robot:{self.robot_id}:commands", json.dumps(move_cmd))
        print(f"[>] Move command sent: x={tx}, y={ty}")

        # 3. 도착 여부 모니터링 (Feedback Loop)
        while True:
            # Link Proxy가 업데이트하는 현재 좌표 읽기
            pos_data = self.r.hget(f"robot:{self.robot_id}:status", "position")
            if pos_data:
                pos = json.loads(pos_data)
                curr_x, curr_y = pos.get('x', 0), pos.get('y', 0)
                
                # 거리 계산 (간단한 유클리드 거리)
                distance = ((tx - curr_x)**2 + (ty - curr_y)**2)**0.5
                
                if distance < 0.1: # 10cm 이내 도착 시
                    print(f"[!] Target Reached: ({curr_x}, {curr_y})")
                    # LLM 오케스트레이터에게 완료 이벤트 알림
                    self.r.publish(f"event:{self.robot_id}:logic", json.dumps({
                        "event": "nav_completed",
                        "task_id": self.task_id,
                        "final_pos": [curr_x, curr_y]
                    }))
                    break
            
            time.sleep(0.5)

if __name__ == "__main__":
    NavWorker().run()