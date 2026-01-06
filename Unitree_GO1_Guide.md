## 0) 전제(가장 안전한 방식)

* **PC(ubuntu18.04)** ↔ **GO1** 은 보통 **유선 LAN(권장)** 또는 같은 공유기/Wi-Fi 망에서 통신
* 처음엔 ROS 붙이지 말고 **SDK 예제 실행으로 “통신/구동”을 먼저 확인**하는 게 제일 안정적

---

## 1) Ubuntu 18.04 기본 세팅

```bash
sudo apt update
sudo apt install -y build-essential cmake git net-tools curl
ifconfig
```

### 빌드에서 자주 터지는 의존성만 추가 설치

대표 오류:

* `boost/shared_ptr.hpp: No such file or directory`

이거는 거의 100% Boost 개발 패키지 누락이라 아래로 끝내는 게 제일 깔끔함.

```bash
sudo apt install -y libboost-all-dev
```
추가로 C++ 예제 빌드에서 필요할 때가 많아서 아래도 같이 넣어두면 안정적.

```bash
sudo apt install -y libpthread-stubs0-dev
```

---

## 2) 네트워크 확인

### 2-1) 내 PC IP 확인

```bash
ifconfig
```

* 보통 `eth0`, `enpXsY` 같은 인터페이스에 `inet 192.168.x.x` 형태로 보임

### 2-2) GO1로 핑 확인

(너가 이미 핑 확인했다고 했으니, 여기선 “확인만” 체크포인트로 둠)

```bash
ping <GO1_IP>
```

* 핑이 안 되면 SDK/ROS 이전에 **망부터** 해결해야 함(공유기/유선/서브넷)

---

## 3) SDK 설치(클론)

### 3-1) repo 준비

(이미 받아놨다면 이 단계는 스킵)

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git
cd unitree_legged_sdk
```

---

## 4) 빌드(가장 흔한 실수 방지 포함)

```bash
cd ~/unitree_legged_sdk
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```
---

## 5) 예제 실행

```bash
cd ~/unitree_legged_sdk/build
./example_velocity
```

실행이 안 되면 보통 권한 문제인데:

```bash
chmod +x example_velocity
./example_velocity
```

---

## 6) “Boost 헤더 오류”

```bash
sudo apt install -y libboost-all-dev
```

그래도 안 되면(진짜 드문 케이스):

```bash
sudo apt install -y libboost-dev
```

---

## 7)다음 단계

### 루트 A) “ROS 없이” 내가 짠 C++ 코드 실행

* 네 코드가 SDK 기반 C++이면
  `unitree_legged_sdk` 안에 소스 추가 → `CMakeLists.txt`에 타겟 추가 → `build`에서 빌드/실행
  이게 가장 빠르고 오류도 적음.

### 루트 B) ROS로 제어(좀 더 표준)

* **ROS는 “PC에 설치”하는 게 일반적으로 편함**
  (라즈베리파이에 ROS 올려도 되지만, 처음부터 그러면 디버깅 난이도가 확 올라감)
* GO1는 보통 **PC에서 ROS 노드 실행 → 네트워크로 GO1에 명령** 흐름이 깔끔함.

---

## 8)체크리스트

1. `./example_velocity` 같은 예제가 실행
2. PC ↔ GO1 ping 
3. `boost/shared_ptr.hpp` 오류 없음
