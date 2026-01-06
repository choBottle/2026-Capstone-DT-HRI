FROM osrf/ros:foxy-desktop

# 필수 패키지 설치
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*

# RoboMaster SDK 설치 (Python 3.8 환경)
RUN pip3 install --upgrade pip
RUN pip3 install robomaster setuptools==58.2.0

# 작업 디렉토리 설정
WORKDIR /root/robomaster_project

# 환경 설정 자동 소싱
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc