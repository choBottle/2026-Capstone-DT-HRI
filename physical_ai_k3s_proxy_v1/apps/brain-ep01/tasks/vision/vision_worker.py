import redis
import json
import os
import time

class VisionWorker:
    def __init__(self):
        self.r = redis.Redis(host=os.getenv("REDIS_HOST", "redis-service"), port=6379, decode_responses=True)
        self.robot_id = os.getenv("ROBOT_SN", "ep01")
        self.task_id = os.getenv("TASK_ID", "default_vision")
        
    def run(self):
        print(f"[*] Vision Worker ({self.task_id}) active for {self.robot_id}")
        
        # 1. LLM이 정의한 파라미터 가져오기 (예: "target": "person")
        task_config = json.loads(self.r.get(f"task_config:{self.task_id}") or "{}")
        target_object = task_config.get("target", "person")

        while True:
            # Link Proxy가 Redis Hash에 쓴 'raw_vision' 데이터를 가져옴
            raw_img = self.r.hget(f"robot:{self.robot_id}:status", "raw_vision")
            
            if raw_img:
                # [AI 로직] target_object가 감지되었는지 판별
                detected = self.detect_logic(raw_img, target_object)
                
                if detected:
                    # 이벤트 발생 알림 (LLM이 설정한 다음 단계 트리거)
                    event_data = {"event": "object_detected", "target": target_object, "time": time.time()}
                    self.r.publish(f"event:{self.robot_id}:logic", json.dumps(event_data))
                    print(f"[!] {target_object} Detected! Event published.")
            
            time.sleep(0.2)

    def detect_logic(self, img, target):
        # 실제로는 OpenCV/YOLO 모델이 들어가는 자리
        return True # 시뮬레이션을 위해 항상 감지된 것으로 가정

if __name__ == "__main__":
    VisionWorker().run()