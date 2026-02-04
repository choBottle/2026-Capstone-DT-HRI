import os, time, logging, sys, json, redis
from threading import Thread
from robomaster import robot

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("robot-link-proxy")

# 환경 변수
TARGET_IP = os.getenv("ROBOT_IP")
ROBOT_ID = os.getenv("ROBOT_SN", "ep01")
REDIS_HOST = os.getenv("REDIS_HOST", "redis-service")

# Redis 연결
try:
    r = redis.Redis(host=REDIS_HOST, port=6379, db=0, decode_responses=True)
except Exception as e:
    logger.error(f"❌ Redis 연결 실패: {e}")
    sys.exit(1)

class RobotLinkProxy:
    def __init__(self, ep_robot):
        self.ep_robot = ep_robot
        self.status_key = f"robot:{ROBOT_ID}:status"
        self.cmd_key = f"robot:{ROBOT_ID}:commands"

    def _update_redis(self, tag, data):
        try:
            r.hset(self.status_key, tag, json.dumps(data))
            r.publish(f"stream:{ROBOT_ID}:{tag}", json.dumps(data))
        except Exception as e:
            logger.debug(f"Redis 업데이트 실패: {e}")

    def sub_battery_info(self, info): 
        self._update_redis("battery", {"soc": info})

    def sub_position_info(self, pos_info):
        x, y, z = pos_info
        self._update_redis("position", {"x": round(x, 3), "y": round(y, 3), "yaw": round(z, 2)})

    def command_loop(self):
        logger.info(f"📥 명령 수신 루프 시작: {self.cmd_key}")
        while True:
            try:
                # Redis 리스트에서 명령 대기 (Timeout 5초)
                res = r.blpop(self.cmd_key, timeout=5)
                if res:
                    _, raw_cmd = res
                    cmd = json.loads(raw_cmd)
                    logger.info(f"📩 명령 수신: {cmd}")
                    self.execute_command(cmd)
            except Exception as e:
                logger.error(f"⚠️ 명령 루프 에러: {e}")
                time.sleep(1)

    def execute_command(self, cmd):
        try:
            target = cmd.get("target", "chassis")
            action = cmd.get("action")
            p = cmd.get("params", {})

            if target == "chassis":
                if action == "MOVE":
                    self.ep_robot.chassis.move(
                        x=p.get('x', 0), 
                        y=p.get('y', 0), 
                        z=p.get('yaw', 0), 
                        xy_speed=p.get('speed', 0.5)
                    ).wait_for_completed()
            elif target == "led":
                self.ep_robot.led.set_led(
                    comp="all", 
                    r=p.get('r', 255), 
                    g=p.get('g', 0), 
                    b=p.get('b', 0)
                )
            logger.info(f"✅ 실행 완료: {target}-{action}")
        except Exception as e:
            logger.error(f"❌ 명령 실행 중 오류: {e}")

def start_proxy():
    ep_robot = robot.Robot()
    try:
        logger.info(f"📡 로봇 연결 시도 중... (IP: {TARGET_IP}, SN: {ROBOT_ID})")
        
        # proto_type="tcp"를 쓰면 더 안정적일 수 있으나 기본 udp 유지
        res = ep_robot.initialize(conn_type="sta")
        
        if res:
            logger.info(f"✅ 로봇 연결 성공")
            proxy = RobotLinkProxy(ep_robot)
            
            # 데이터 구독
            ep_robot.battery.sub_battery_info(freq=1, callback=proxy.sub_battery_info)
            ep_robot.chassis.sub_position(freq=5, callback=proxy.sub_position_info)

            # 명령 수신 스레드 실행
            cmd_thread = Thread(target=proxy.command_loop, daemon=True)
            cmd_thread.start()

            logger.info(f"🚀 Link Proxy 가동 완료")
            
            # 메인 스레드 유지
            while True:
                time.sleep(1)
        else:
            logger.error("❌ 로봇 초기화 실패: IP를 확인하거나 로봇 전원을 리셋하세요.")
            
    except Exception as e:
        logger.error(f"💥 시스템 예외: {e}")
    finally:
        try:
            ep_robot.close()
            logger.info("🔌 로봇 연결 세션 종료")
        except:
            pass

if __name__ == "__main__":
    start_proxy()