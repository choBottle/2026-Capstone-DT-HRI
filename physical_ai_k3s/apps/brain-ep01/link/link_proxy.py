import os, time, logging, sys, json, redis, math
from threading import Thread
from robomaster import robot

# 로그 설정 (성공 원본 유지)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger("robot-link-proxy")

print("--- [START] link_proxy.py process is running ---", flush=True)

# 환경 변수 로드
ROBOT_ID = os.getenv("ROBOT_SN", "ep01")
REDIS_HOST = os.getenv("REDIS_HOST", "redis-service")

try:
    r = redis.Redis(host=REDIS_HOST, port=6379, db=0, decode_responses=True)
    r.ping() 
    logger.info("✅ Redis Connection Successful")
except Exception as e:
    logger.error(f"❌ Redis 연결 실패: {e}")
    sys.exit(1)

class RobotLinkProxy:
    def __init__(self, ep_robot):
        self.ep_robot = ep_robot
        self.status_key = f"robot:{ROBOT_ID}:status"
        self.cmd_key = f"robot:{ROBOT_ID}:commands"

    def _update_redis(self, tag, data):
        """데이터 업데이트 및 발행 (성공 원본 유지)"""
        try:
            r.hset(self.status_key, tag, json.dumps(data))
            r.publish(f"stream:{ROBOT_ID}:{tag}", json.dumps(data))
        except Exception as e:
            logger.error(f"❌ Redis Hash Update Error: {e}")

    def execute_command(self, cmd):
        target = cmd.get("target")
        action = cmd.get("action")
        p = cmd.get("params", {})

        try:
            # 1. 센서 데이터 구독 (샌드박스 기능을 성공 원본 패턴에 이식)
            if target == "sensor":
                # 개별 호출 방식으로 유지하여 SDK 충돌 방지
                if action == "SUB_BATTERY": 
                    self.ep_robot.battery.sub_battery_info(freq=p.get('f', 1), callback=lambda x: self._update_redis("battery", {"soc": x}))
                elif action == "SUB_POS": 
                    self.ep_robot.chassis.sub_position(freq=p.get('f', 5), callback=lambda x: self._update_redis("position", {"x":round(x[0],2), "y":round(x[1],2)}))
                elif action == "SUB_ARMOR": 
                    # [샌드박스 검증] sub_hit_event 방식으로 교체
                    self.ep_robot.armor.sub_hit_event(callback=lambda x: self._update_redis("armor_hit", {"id": x[0], "type": x[1]}))
                elif action == "SUB_SPEED": # [추가] 샌드박스 속도 로직
                    self.ep_robot.chassis.sub_velocity(freq=5, callback=lambda x: self._update_redis("velocity", {"speed":round(math.sqrt(x[0]**2+x[1]**2),2)}))
                elif action == "SUB_IMU": # [추가] 샌드박스 IMU 로직
                    self.ep_robot.chassis.sub_imu(freq=2, callback=lambda x: self._update_redis("imu", {"acc": x[0:3]}))
                elif action == "UNSUB":
                    self.ep_robot.battery.unsub_battery_info()
                    self.ep_robot.chassis.unsub_position()
                    self.ep_robot.armor.unsub_hit_event()
                    self.ep_robot.chassis.unsub_velocity()
                    self.ep_robot.chassis.unsub_imu()

            # 2. 이동/액추에이터/비전/LED (성공 원본 100% 동일)
            elif target == "chassis":
                if action == "MOVE": 
                    self.ep_robot.chassis.move(x=p.get('x',0), y=p.get('y',0), z=p.get('z',0), xy_speed=p.get('speed',0.5)).wait_for_completed()
                elif action == "ROTATE": 
                    self.ep_robot.chassis.move(z=p.get('yaw',0), z_speed=p.get('v_speed',45)).wait_for_completed()
            elif target == "actuator":
                if action == "ARM_MOVE": 
                    self.ep_robot.robotic_arm.move(x=p.get('arm_x',0), y=p.get('arm_y',0)).wait_for_completed()
                elif action == "GRIPPER":
                    if p.get('grip') == "open": self.ep_robot.gripper.open(power=p.get('grip_p', 50))
                    else: self.ep_robot.gripper.close(power=p.get('grip_p', 50))
            elif target == "vision":
                if action == "START":
                    self.ep_robot.camera.start_video_stream(display=False)
                    self.ep_robot.vision.sub_detect_info(name=p.get('type', 'marker'), callback=lambda x: self._update_redis(p.get('type'), x))
                elif action == "STOP":
                    self.ep_robot.vision.unsub_detect_info(name=p.get('type', 'marker'))
                    self.ep_robot.camera.stop_video_stream()
            elif target == "led":
                self.ep_robot.led.set_led(comp="all", r=p.get('r',255), g=p.get('g',255), b=p.get('b',255), effect=p.get('eff', 'on'))

            logger.info(f"✅ Success: {target} -> {action}")
        except Exception as e: 
            logger.error(f"❌ Execution Fail ({target}:{action}): {e}")

    def command_loop(self):
        logger.info(f"📥 Listener Active: Listening on {self.cmd_key}")
        while True:
            try:
                res = r.blpop(self.cmd_key, timeout=2)
                if res:
                    _, payload = res
                    cmd_data = json.loads(payload)
                    logger.info(f"📩 Command Received: {cmd_data}")
                    self.execute_command(cmd_data)
            except Exception as e:
                logger.error(f"⚠️ Command Loop Error: {e}")
                time.sleep(1)

def start_proxy():
    logger.info("🤖 RoboMaster SDK Initializing...")
    ep_robot = robot.Robot()
    if ep_robot.initialize(conn_type="sta"):
        ep_robot.set_robot_mode(mode="free")
        logger.info("📡 [Connected] Robot connection established!")
        proxy = RobotLinkProxy(ep_robot)
        worker = Thread(target=proxy.command_loop, daemon=True)
        worker.start()
        logger.info(f"🚀 [READY] Link Proxy is Running.")
        try:
            while True: time.sleep(10)
        except KeyboardInterrupt:
            ep_robot.close()
    else:
        logger.error("❌ [Failed] Could not initialize robot.")
        sys.exit(1)

if __name__ == "__main__":
    start_proxy()