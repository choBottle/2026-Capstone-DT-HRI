FROM ros:foxy-ros-base

WORKDIR /app

# 시스템 라이브러리 업데이트 및 pip 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# RoboMaster SDK 및 필수 의존성(OpenCV) 설치
# opencv-python-headless는 GUI 기능 없이 라이브러리만 설치하므로 컨테이너 환경에 최적입니다.
RUN pip3 install --no-cache-dir \
    robomaster \
    opencv-python-headless

# 소스 복사
COPY control.py .

# ROS 환경 로드 후 실행
CMD ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash && python3 -u control.py"]