# 📘 RoboMaster 초저지연 직통 스트리밍 (Direct Web Stream) 가이드 (ROS 미사용)

이 가이드는 라즈베리파이의 제한된 성능 문제를 해결하기 위해, ROS 2의 이미지 통신 과정을 생략하고 **웹 서버를 통해 유니티로 직접 영상을 쏘는 방식**을 설명합니다.

## 1. 개요 및 장단점 분석

이 방식은 `RoboMaster SDK`에서 받은 영상을 ROS 메시지로 변환하지 않고, Python `Flask` 서버를 통해 즉시 송출합니다.

### ✅ 장점 (Pros)

1. **초저지연 (Low Latency):** ROS 2의 무거운 직렬화/역직렬화(Serialization) 과정이 없어 속도가 2배 이상 빠릅니다.
2. **화면 깨짐 해결:** 유니티가 웹에서 "완성된 사진 한 장"씩을 요청해서 가져오므로, 데이터 전송 중 화면이 찢어지는 현상(Tearing)이 원천적으로 차단됩니다.
3. **CPU 부하 감소:** 라즈베리파이 발열과 렉이 현저히 줄어듭니다.

### ⚠️ 단점 및 한계 (Cons)

1. **ROS 2 노드 간 데이터 공유 불가:**
* 영상을 웹으로만 쏘기 때문에, 기존 매뉴얼에 있던 `video_recorder.py` 처럼 **'camera/raw' 토픽을 구독하는 다른 ROS 노드들이 작동하지 않습니다**.
* 영상을 저장하거나, 별도의 AI 노드에서 분석하려면 코드를 `direct_webcam.py` 안에 직접 통합해야 합니다.

2. **Rviz 시각화 불가:**
* ROS 네트워크에 영상 데이터가 없으므로, Rviz 같은 ROS 시각화 도구에서는 카메라 화면을 볼 수 없습니다.


3. **센서 퓨전(Sensor Fusion)의 어려움:**
* 나중에 라이다(LiDAR)나 IMU 센서와 영상을 합쳐서 지도(SLAM)를 그릴 때, 정확한 시간 동기화(Timestamping)를 맞추기가 ROS 2 표준 방식보다 어렵습니다.

---

## 2. 사전 준비 (라즈베리파이)

도커 컨테이너 내부에서 가벼운 웹 서버 라이브러리인 `Flask`를 설치해야 합니다.

```bash
# 라즈베리파이 터미널 (도커 내부)
pip3 install flask

```

---

## 3. 라즈베리파이 설정 (서버 코드)

기존의 `camera_driver.py` 등 다른 카메라 관련 파일은 모두 종료하고, 아래 파일 하나만 실행합니다.

### 📄 파일 생성: `direct_webcam.py`

경로: `~/robomaster_project/ros2_ws/src/my_robomaster/my_robomaster/direct_webcam.py`

```python
import time
import threading
import cv2
from flask import Flask, Response, make_response
from robomaster import robot

# [설정] 웹 서버 포트
PORT = 5000
app = Flask(__name__)

# 스레드 간 데이터 공유를 위한 변수와 자물쇠
shared_frame = None
lock = threading.Lock()

def robot_camera_thread():
    global shared_frame
    print("🤖 로봇 연결 시도 (STA 모드)...")
    
    ep_robot = robot.Robot()
    try:
        ep_robot.initialize(conn_type="sta")
        print(f"✅ 로봇 연결 성공! IP: {ep_robot.ip}")
    except Exception as e:
        print(f"❌ 연결 실패: {e}")
        return

    # [최적화] 360p 해상도로 CPU 부하 최소화 (필요 시 '720p' 변경 가능)
    ep_robot.camera.start_video_stream(display=False, resolution='360p')
    print("📷 카메라 스트림 시작 (360p)")

    while True:
        try:
            # 로봇에서 최신 이미지 가져오기
            img = ep_robot.camera.read_cv2_image(strategy="newest")
            
            if img is not None:
                # 1. 전송 속도를 위해 이미지 크기 최적화 (320x180)
                img_small = cv2.resize(img, (320, 180))
                
                # 2. JPEG 압축 (품질 50%)
                _, buffer = cv2.imencode('.jpg', img_small, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
                byte_data = buffer.tobytes()

                # 3. 데이터 업데이트 (자물쇠로 보호)
                with lock:
                    shared_frame = byte_data
            
            # 과열 방지를 위한 미세 딜레이
            time.sleep(0.03)
            
        except Exception as e:
            print(f"에러: {e}")
            time.sleep(1)

# [유니티용] 요청할 때마다 완성된 사진 한 장을 반환 (깨짐 방지 핵심)
@app.route('/snapshot')
def snapshot():
    with lock:
        if shared_frame is None:
            return "No Frame", 503
        data = shared_frame
    
    response = make_response(data)
    response.headers.set('Content-Type', 'image/jpeg')
    return response

# [브라우저용] 크롬 등에서 테스트하기 위한 스트리밍 주소
@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            with lock:
                if shared_frame is None:
                    time.sleep(0.05)
                    continue
                data = shared_frame
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + data + b'\r\n')
            time.sleep(0.05)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # 카메라 스레드 백그라운드 실행
    t = threading.Thread(target=robot_camera_thread)
    t.daemon = True
    t.start()

    # 웹 서버 실행
    print(f"🚀 웹 서버 실행 중... http://0.0.0.0:{PORT}")
    app.run(host='0.0.0.0', port=PORT, debug=False, threaded=True)

```

---

## 4. 유니티 설정 (클라이언트)

### ⚙️ 필수 설정: HTTP 허용 (중요)

유니티 보안 정책상 `http` 접속을 허용해줘야 합니다.

1. **Edit** → **Project Settings** → **Player** 메뉴 진입.
2. **Other Settings** 탭 → **Configuration** 섹션 찾기.
3. **Allow downloads over HTTP** 항목을 **`Always Allowed`**로 변경.

### 📄 스크립트 생성: `WebCamStream.cs`

RawImage 오브젝트에 연결할 스크립트입니다.

```csharp
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Networking;
using System.Collections;

public class WebCamStream : MonoBehaviour
{
    [Header("Settings")]
    // [주의] 라즈베리파이 IP 확인. 끝에 /snapshot 필수!
    public string snapshotUrl = "http://192.168.50.99:5000/snapshot"; 
    
    [Header("Display")]
    public RawImage displayImage;

    private Texture2D texture;

    void Start()
    {
        // 텍스처 메모리 초기화
        texture = new Texture2D(2, 2);
        displayImage.texture = texture;
        
        // 스트리밍 코루틴 시작
        StartCoroutine(FetchFrames());
    }

    IEnumerator FetchFrames()
    {
        while (true)
        {
            // 서버에 "사진 한 장만 줘" 라고 요청 (스냅샷 방식)
            using (UnityWebRequest request = UnityWebRequestTexture.GetTexture(snapshotUrl))
            {
                request.timeout = 1; // 1초 타임아웃
                yield return request.SendWebRequest();

                if (request.result == UnityWebRequest.Result.Success)
                {
                    // 받은 이미지를 텍스처로 변환
                    DownloadHandlerTexture handler = request.downloadHandler as DownloadHandlerTexture;
                    
                    // 이전 텍스처 삭제 (메모리 누수 방지)
                    if (displayImage.texture != null && displayImage.texture != texture)
                        Destroy(displayImage.texture);

                    // 화면에 적용
                    displayImage.texture = handler.texture;
                }
            }

            // 너무 빠르면 부하가 걸리므로 0.05초 대기 (약 20FPS)
            yield return new WaitForSeconds(0.05f);
        }
    }
}

```

---

## 5. 실행 순서 및 테스트

1. **[라즈베리파이]** 코드 실행
```bash
python3 direct_webcam.py

```


*주의: 기존에 실행 중이던 ROS 노드는 모두 종료하세요.*
2. **[PC]** 웹 브라우저 테스트 (1차 확인)
* 크롬 주소창에 `http://[라즈베리파이IP]:5000/video_feed` 입력.
* 영상이 끊김 없이 나오는지 확인합니다.


3. **[유니티]** 프로젝트 실행 (최종 확인)
* `RawImage`에 `WebCamStream.cs`가 연결되었는지 확인.
* **Play 버튼**을 누르면 깨끗한 영상이 송출됩니다.

---