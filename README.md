# siyi_a8mini / a8mini_msgs

[English](#english) | [한국어](#한국어)

---

## <a name="english"></a>English

### Overview

This repository contains two ROS2 packages based on the `mzahana/siyi_sdk`:

* `a8mini_msgs`: Custom ROS messages (`SystemInfo.msg`) used by the A8Mini stack.
* `siyi_a8mini`: Python ROS2 nodes and SDK utilities. This package splits functionality into **two separate nodes** to prevent C++ library conflicts (Segfaults):
    * `gimbal_control_node`: Handles all gimbal control, telemetry, and services using the `siyi_sdk`. (No OpenCV/GStreamer).
    * `video_stream_node`: Handles only RTSP video streaming using `mzahana/stream.py` (OpenCV + FFMPEG backend). It carefully manages import order to avoid conflicts with `rclpy`.

### Features

* **Crash Prevention:** Solves `exit code -6` / `-11` (Segfault) errors by separating gimbal/video nodes and ensuring `rclpy.init()` is called *before* `import cv2` in the video node.
* **Low-Latency Video Streaming:**
    * Uses `mzahana/stream.py` (OpenCV + FFMPEG backend) for video capture.
    * Injects `ffplay`-equivalent low-latency flags (`fflags;nobuffer|flags;low_delay`) via environment variables for improved performance.
    * Publishes `sensor_msgs/Image` and `sensor_msgs/CameraInfo` topics.
* **Gimbal Control Node:** A dedicated node for all `siyi_sdk` communication (Attitude, Services) without OpenCV/GStreamer dependencies.
* **Full Gimbal Control:** Full control over gimbal yaw/pitch (velocity and position), mode switching (Follow, FPV, Lock), and camera functions (Photo, Video) via ROS2 services and topics.
* **Telemetry:** Publishes gimbal firmware, hardware ID, and real-time attitude (10Hz) as ROS2 topics.

### Architecture: Why 2 Nodes?

This package **intentionally separates** gimbal control and video processing into two nodes. This is **not optional**—it is required to prevent critical memory crashes.

* **The Problem:** Importing `rclpy` (ROS2) in the same Python script as `import cv2` (OpenCV) or `import gi` (GStreamer) often causes a C++ library conflict, leading to an immediate **`exit code -6` (Abort)** or **`exit code -11` (Segmentation Fault)**. This is a known issue related to how these complex libraries initialize their C++ backends. (See: [rclpy/issues/1149](https://github.com/ros2/rclpy/issues/1149))
* **The Solution:**
    1.  **`gimbal_control_node`:** Imports `rclpy` and `siyi_sdk`. (No `cv2` or `gi`). This node is stable.
    2.  **`video_stream_node`:** Calls `rclpy.init()` **first**, *then* imports `cv2`, `cv_bridge`, and `stream.py`. This specific order prevents the C++ conflict and avoids the segfault.

### Performance (Video Latency)

Video latency is a critical factor. By default, `cv2.VideoCapture` with an RTSP stream (using the FFMPEG backend) can introduce significant buffering, leading to delays of 600-900ms compared to an optimized `ffplay` command.

This package minimizes this latency by setting FFMPEG flags (`fflags;nobuffer|flags;low_delay`) via the `OPENCV_FFMPEG_CAPTURE_OPTIONS` environment variable *before* importing `cv2`.

**Latency Comparison (Default OpenCV vs. FFplay):**

`[이미지 삽입: ffplay (우측)와 이 ROS 노드(좌측)의 지연 시간 비교. 'image_b25d42.jpg' 사용]`
*(이미지 캡션: 기준 시계(하단). 기본 cv2.VideoCapture (좌측, ~900ms 지연)가 최적화된 ffplay (우측, ~300ms 지연)보다 현저히 느림. 이 노드는 ffplay의 플래그를 주입하여 저지연을 달성합니다.)*

### Repository Structure

```

ros2\_siyi\_ws/
└── src/
├── a8mini\_msgs/
│   ├── msg/
│   │   └── SystemInfo.msg
│   ├── CMakeLists.txt
│   └── package.xml
│
└── siyi\_a8mini/
├── launch/
│   └── a8mini.launch.py  (2개의 노드를 모두 실행)
├── resource/
│   └── siyi\_a8mini     (ROS2 패키지 마커)
├── siyi\_a8mini/        (Python 소스 루트)
│   ├── **init**.py
│   ├── gimbal\_control\_node.py (짐벌 제어)
│   ├── video\_stream\_node.py   (영상 스트리밍)
│   │
│   ├── siyi\_sdk.py            (mzahana SDK 원본)
│   ├── siyi\_message.py        (mzahana SDK 원본)
│   ├── stream.py              (mzahana SDK 원본 - OpenCV/FFMPEG)
│   ├── cameras.py           (mzahana SDK 원본)
│   ├── crc16\_python.py        (mzahana SDK 원본)
│   └── utils.py               (mzahana SDK 원본)
│
├── test/
│   └── a8mini\_test.py
├── package.xml
└── setup.py

````

### Environment setup

Prerequisites:

* Ubuntu 22.04 + ROS2 Humble (Recommended)
* Python 3.10+

**1. Install ROS2 Dependencies**
Install `cv_bridge` (for video) and the core build tools.

```bash
# Setup ROS2 (if not done)
sudo apt update && sudo apt install ros-humble-desktop

# Install core dependencies for this package
sudo apt install python3-colcon-common-extensions ros-humble-cv-bridge
````

**2. CRITICAL: Solve OpenCV Version Conflict (The Segfault Cause)**
ROS2 Humble (`ros-humble-cv-bridge`) *requires* the `apt` version of OpenCV (e.g., `4.5.4`). If you have `opencv-python` installed via `pip` (e.g., 4.11.x), it **will** cause a **Segmentation Fault (`exit code -6`)**. You MUST remove the `pip` version.

```bash
# 1. REMOVE the pip version
pip3 uninstall opencv-python opencv-python-headless

# 2. INSTALL the system version that ROS uses
sudo apt install python3-opencv
```

**3. Install Python Dependencies (mzahana/siyi\_sdk)**
The original `stream.py` requires `imutils`.

```bash
pip3 install imutils
```

### Installation

```bash
# (From workspace root: ros2_siyi)
cd ~/project/ros2_siyi

# 1. Source ROS environment
source /opt/ros/humble/setup.bash  # Change 'humble' to your distribution

# 2. (Recommended) Clean previous builds
rm -rf build install log

# 3. Build (both packages)
colcon build

# 4. Source the workspace
source install/setup.bash
```

### Usage

**1. Launch Nodes**
Start the full stack (Gimbal Control Node + Video Stream Node):

```bash
# from workspace root
source install/setup.bash
ros2 launch siyi_a8mini a8mini.launch.py
```

**2. View Video Stream**
Use RVIZ2 to view the published image topic.

```bash
# In a new terminal
source install/setup.bash
rviz2
```

  * In RVIZ2, click `[Add]` -\> `[By topic]`.
  * Select the `/A8mini/camera/image_raw` topic.
  * **Important:** If you see "No Image", you must change the QoS settings.

`[이미지 삽입: RVIZ2에서 Image 토픽의 QoS 설정을 'Best Effort'로 변경. 'image_32310c.jpg' 사용]`
*(이미지 캡션: RVIZ2에서 'Reliability Policy'를 'Best Effort'로 변경하여 QoS를 맞춥니다.)*

  * Expand the "Image" display plugin settings.
  * Change the **Reliability Policy** from `Reliable` to `Best Effort`. The image will appear.

-----

### API Reference

#### Published Topics

| Topic Name | Message Type | Description |
| --- | --- | --- |
| `/A8mini/info` | `a8mini_msgs/SystemInfo` | (Latched) Gimbal FW version, HW ID, and modes. |
| `/A8mini/Gimbal/Info/attitude` | `geometry_msgs/Vector3Stamped` | Real-time gimbal attitude (Roll, Pitch, Yaw) at 10Hz. |
| `/A8mini/camera/image_raw` | `sensor_msgs/Image` | Raw video frames from the camera. |
| `/A8mini/camera/camera_info`| `sensor_msgs/CameraInfo` | (Latched) Camera calibration and info (based on 81° HFOV). |
| `/A8mini/camera/image_rect`| `sensor_msgs/Image` | (Currently same as `image_raw`) |

#### Subscribed Topics

| Topic Name | Message Type | Description |
| --- | --- | --- |
| `/A8mini/Gimbal/set_attitude` | `geometry_msgs/Point` | Set gimbal target attitude (`Point.x`=Yaw, `Point.y`=Pitch). |
| `/A8mini/Gimbal/set_rotation` | `geometry_msgs/Vector3` | Set gimbal rotation speed (`Vector3.x`=YawSpd, `Vector3.y`=PitchSpd). |

#### Services

| Service Name | Service Type | Description |
| --- | --- | --- |
| `/A8mini/soft_reboot` | `std_srvs/srv/Trigger` | Reboots the gimbal and camera. |
| `/A8mini/set_osd` | `std_srvs/srv/SetBool` | Set HDMI OSD On/Off. |
| `/A8mini/Gimbal/mode/take_photo` | `std_srvs/srv/Trigger` | Triggers photo capture. |
| `/A8mini/Gimbal/mode/toggle_video` | `std_srvs/srv/Trigger` | Toggles video recording (Start/Stop). |
| `/A8mini/Gimbal/mode/set_lock_mode` | `std_srvs/srv/Trigger` | Set gimbal to Lock Mode. |
| `/A8mini/Gimbal/mode/set_follow_mode` | `std_srvs/srv/Trigger` | Set gimbal to Follow Mode. |
| `/A8mini/Gimbal/mode/set_fpv_mode` | `std_srvs/srv/Trigger` | Set gimbal to FPV Mode. |
| `/A8mini/Gimbal/mode/set_hdmi_output` | `std_srvs/srv/SetBool` | Enable/Disable HDMI output. |

### Troubleshooting

**1. `exit code -6` or `-11` (Segmentation Fault) on startup**

  * **Symptom:** `video_stream_node` dies immediately with `exit code -6 (Abort)` or `-11 (Segfault)`. `gimbal_control_node` runs fine.
  * **Cause 1 (Most Likely): OpenCV Version Conflict.** You have a `pip` version of `opencv-python` (e.g., 4.11.x) installed, which conflicts with the `apt` version (e.g., 4.5.x) required by `ros-humble-cv-bridge`.
  * **Solution 1:** **You MUST remove the `pip` version.** See [Environment setup](https://www.google.com/search?q=%23environment-setup).
  * **Cause 2: Import Order.** `import cv2` was called *before* `rclpy.init()`.
  * **Solution 2:** This repository's `video_stream_node.py` already solves this by calling `rclpy.init()` in `main()` *before* importing `cv2`.

**2. `404 Stream Not Found` in `video_stream_node` logs**

  * **Symptom:** The gimbal node works, but the video node logs `Failed to open RTSP stream`.
  * **Cause:** The RTSP URL is incorrect. `main.264` (H.264) and `video1` (H.265) are common, but vary by camera.
  * **Solution:** Edit `a8mini.launch.py` and change the `rtsp_url` parameter default. First, test the URL directly with `ffplay`:
    ```bash
    # Test H.264 stream
    ffplay -flags low_delay rtsp://192.168.144.25:8554/main.264
    # Test H.265 stream (main)
    ffplay -flags low_delay rtsp://192.168.144.25:8554/video1
    # Test H.265 stream (sub)
    ffplay -flags low_delay rtsp://192.168.144.25:8554/video2
    ```
    Use the URL that works in `ffplay` in your `a8mini.launch.py`.

**3. RVIZ2 shows "No Image" (QoS Conflict)**

  * **Symptom:** `video_stream_node` is running, but RVIZ2 shows "No Image" and logs `[WARN] ... offering incompatible QoS. ... Last incompatible policy: RELIABILITY_QOS_POLICY`.
  * **Cause:** The `video_stream_node` publishes with `BEST_EFFORT` (for low latency), but RVIZ2's "Image" display defaults to `RELIABLE`.
  * **Solution:** In RVIZ2, expand the "Image" display plugin settings. Change the **Reliability Policy** from `Reliable` to `Best Effort`. The image will appear.

**4. Video stream is laggy or corrupted (Grey Blocks)**

  * **Symptom:** The video feed in RVIZ2 or the debug window has a \>500ms delay or shows grey blocks.
  * **Cause:** `cv2.VideoCapture` is not using low-latency FFMPEG options, or you are using TCP (`use_udp=False`) on a lossy network.
  * **Solution:**
    1.  Ensure `video_stream_node.py` includes the `os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "flags;low_delay|fflags;nobuffer"` line *before* `import cv2`.
    2.  Ensure `video_stream_node.py` is constructing `SIYIRTSP` with `use_udp=True` (this is the default in the current code).

-----

## <a name="한국어"></a>한국어

### 개요

이 저장소에는 `mzahana/siyi_sdk`를 기반으로 하는 두 개의 ROS2 패키지가 포함되어 있습니다:

  * `a8mini_msgs`: A8Mini 스택에서 사용하는 커스텀 ROS 메시지(`SystemInfo.msg`)입니다.
  * `siyi_a8mini`: Python으로 작성된 ROS2 노드, SDK 및 유틸리티입니다. 이 패키지는 C++ 라이브러리 충돌(Segfault)을 방지하기 위해 기능을 **두 개의 독립된 노드**로 분리합니다:
      * `gimbal_control_node`: `siyi_sdk`를 사용하여 모든 짐벌 제어, 텔레메트리, 서비스를 처리합니다. (OpenCV/GStreamer 의존성 없음).
      * `video_stream_node`: `rclpy`와의 충돌을 피하기 위해 임포트 순서를 제어하며, `mzahana/stream.py` (OpenCV + FFMPEG 백엔드)를 사용해 RTSP 비디오 스트리밍만 처리합니다.

### 주요 기능

  * **충돌 방지:** `video_stream_node`에서 `rclpy.init()`을 `import cv2`보다 먼저 호출하고, 짐벌/비디오 노드를 분리하여 `exit code -6` / `-11` (Segfault) 에러를 해결합니다.
  * **저지연 비디오 스트리밍:**
      * `mzahana/stream.py` (OpenCV + FFMPEG 백엔드)를 사용하여 비디오를 캡처합니다.
      * `ffplay`와 동등한 저지연 플래그(`fflags;nobuffer|flags;low_delay`)를 환경 변수로 주입하여 성능을 향상시킵니다.
      * `sensor_msgs/Image` 및 `sensor_msgs/CameraInfo` 토픽을 발행합니다.
  * **짐벌 제어 노드:** OpenCV/GStreamer 의존성이 없는, `siyi_sdk` 통신(자세, 서비스) 전용 노드입니다.
  * **전체 짐벌 제어:** ROS2 서비스 및 토픽을 통해 짐벌 Yaw/Pitch(속도 및 위치) 제어, 모드 변경(Follow, FPV, Lock), 카메라 기능(사진, 비디오)을 완벽하게 지원합니다.
  * **텔레메트리:** 짐벌 펌웨어, 하드웨어 ID, 실시간 자세(10Hz) 정보를 ROS2 토픽으로 발행합니다.

### 아키텍처: 왜 2개의 노드인가?

이 패키지는 짐벌 제어와 비디오 처리를 **의도적으로** 두 개의 노드로 분리합니다. 이는 **선택 사항이 아니며**, 치명적인 메모리 충돌을 방지하기 위해 필수적입니다.

  * **문제점:** `rclpy` (ROS2)를 `import cv2` (OpenCV) 또는 `import gi` (GStreamer)와 동일한 파이썬 스크립트에서 임포트하면 C++ 라이브러리 충돌이 발생하여, 즉시 **`exit code -6` (Abort)** 또는 \*\*`exit code -11` (Segfault)\*\*로 노드가 강제 종료됩니다. 이는 ROS2와 다른 C++ 백엔드 라이브러리의 초기화 순서 문제와 관련이 있습니다. (참고: [rclpy/issues/1149](https://github.com/ros2/rclpy/issues/1149))
  * **해결책:**
    1.  **`gimbal_control_node`:** `rclpy`와 `siyi_sdk`만 임포트합니다. (`cv2` 또는 `gi` 없음). 이 노드는 항상 안정적입니다.
    2.  **`video_stream_node`:** `rclpy.init()`을 **먼저** 호출한 다음, `cv2`, `cv_bridge`, `stream.py`를 임포트합니다. 이 특정 순서는 C++ 충돌을 방지하고 세그폴트를 회피합니다.

### 성능 (비디오 지연 시간)

비디오 지연 시간은 매우 중요합니다. 기본적으로 `cv2.VideoCapture`는 RTSP 스트림(FFMPEG 백엔드) 사용 시 과도한 버퍼링을 유발하여, 최적화된 `ffplay` 명령어 대비 600\~900ms의 지연이 발생할 수 있습니다.

이 패키지는 `cv2` 모듈을 임포트하기 *전에* `OPENCV_FFMPEG_CAPTURE_OPTIONS` 환경 변수를 설정하여 FFMPEG 플래그(`fflags;nobuffer|flags;low_delay`)를 주입함으로써 이 지연 시간을 최소화합니다.

**지연 시간 비교 (기본 OpenCV vs FFplay):**

`[이미지 삽입: ffplay (우측)와 이 ROS 노드(좌측)의 지연 시간 비교. 'image_b25d42.jpg' 사용]`
*(이미지 캡션: 기준 시계(하단). 기본 cv2.VideoCapture (좌측, \~900ms 지연)가 최적화된 ffplay (우측, \~300ms 지연)보다 현저히 느림. 이 노드는 ffplay의 플래그를 주입하여 저지연을 달성합니다.)*

### 저장소 구조

```
ros2_siyi_ws/
└── src/
    ├── a8mini_msgs/
    │   ├── msg/
    │   │   └── SystemInfo.msg
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── siyi_a8mini/
        ├── launch/
        │   └── a8mini.launch.py  (2개의 노드를 모두 실행)
        ├── resource/
        │   └── siyi_a8mini     (ROS2 패키지 마커)
        ├── siyi_a8mini/        (Python 소스 루트)
        │   ├── __init__.py
        │   ├── gimbal_control_node.py (짐벌 제어)
        │   ├── video_stream_node.py   (영상 스트리밍)
        │   │
        │   ├── siyi_sdk.py            (mzahana SDK 원본)
        │   ├── siyi_message.py        (mzahana SDK 원본)
        │   ├── stream.py              (mzahana SDK 원본 - OpenCV/FFMPEG)
        │   ├── cameras.py           (mzahana SDK 원본)
        │   ├── crc16_python.py        (mzahana SDK 원본)
        │   └── utils.py               (mzahana SDK 원본)
        │
        ├── test/
        │   └── a8mini_test.py
        ├── package.xml
        └── setup.py
```

### 환경 구성

사전 조건:

  * Ubuntu 22.04 + ROS2 Humble (권장)
  * Python 3.10+

**1. ROS2 의존성 설치**
비디오 처리를 위한 `cv_bridge`와 빌드 도구를 설치합니다.

```bash
# ROS2 설치 (Humble 예시, 이미 설치했다면 건너뜀)
sudo apt update && sudo apt install ros-humble-desktop

# 이 패키지의 핵심 의존성 설치
sudo apt install python3-colcon-common-extensions ros-humble-cv-bridge
```

**2. [매우 중요] OpenCV 버전 충돌 해결 (Segfault 원인)**
ROS2 Humble (`ros-humble-cv-bridge`)은 `apt`로 설치된 OpenCV (예: `4.S.4`)에 의존합니다. 만약 `pip`를 통해 `opencv-python` (예: `4.11.x`)을 설치했다면, `rclpy`와 `cv2`가 함께 임포트될 때 \*\*메모리 충돌(Segmentation Fault, `exit code -6` 또는 `-11`)\*\*이 발생합니다.

반드시 `pip` 버전을 **제거**하고 `apt` 버전을 사용해야 합니다.

```bash
# 1. pip 버전 제거
pip3 uninstall opencv-python opencv-python-headless

# 2. ROS가 사용하는 시스템(apt) 버전 설치
sudo apt install python3-opencv
```

**3. Python 의존성 설치 (mzahana/siyi\_sdk)**
`stream.py`는 `imutils`를 필요로 합니다.

```bash
pip3 install imutils
```

### 설치 방법

```bash
# (워크스페이스 루트: ros2_siyi)
cd ~/project/ros2_siyi

# 1. ROS 환경 설정
source /opt/ros/humble/setup.bash  # 'humble'을 본인의 배포판으로 변경

# 2. (권장) 이전 빌드 캐시 삭제
rm -rf build install log

# 3. 빌드 (두 패키지 모두)
colcon build

# 4. 워크스페이스 환경 설정
source install/setup.bash
```

### 사용법

**1. 노드 실행**
짐벌 제어 노드와 비디오 스트림 노드를 모두 실행합니다:

```bash
# from workspace root
source install/setup.bash
ros2 launch siyi_a8mini a8mini.launch.py
```

**2. 비디오 스트림 확인**
RVIZ2를 사용하여 발행된 이미지 토픽을 확인합니다.

```bash
# 새 터미널에서 실행
source install/setup.bash
rviz2
```

  * RVIZ2에서 `[Add]` -\> `[By topic]`을 클릭합니다.
  * `/A8mini/camera/image_raw` 토픽을 선택합니다.
  * **중요:** "No Image"가 표시되면 QoS 설정을 변경해야 합니다.

`[이미지 삽입: RVIZ2에서 Image 토픽의 QoS 설정을 'Best Effort'로 변경. 'image_32310c.jpg' 사용]`
*(이미지 캡션: RVIZ2에서 'Reliability Policy'를 'Best Effort'로 변경하여 QoS를 맞춥니다.)*

  * "Image" 디스플레이 플러그인 설정을 펼칩니다.
  * **Reliability Policy**를 `Reliable`에서 `Best Effort`로 변경합니다. 영상이 즉시 나타납니다.

-----

### API 레퍼런스

#### 발행 토픽 (Published Topics)

| 토픽 이름 | 메시지 타입 | 설명 |
| --- | --- | --- |
| `/A8mini/info` | `a8mini_msgs/SystemInfo` | (Latched) 짐벌 펌웨어, 하드웨어 ID, 모드 정보 |
| `/A8mini/Gimbal/Info/attitude` | `geometry_msgs/Vector3Stamped` | 실시간 짐벌 자세 (Roll, Pitch, Yaw), 10Hz |
| `/A8mini/camera/image_raw` | `sensor_msgs/Image` | 카메라의 원본 비디오 프레임 |
| `/A8mini/camera/camera_info`| `sensor_msgs/CameraInfo` | (Latched) 카메라 보정 정보 (81° HFOV 기준) |
| `/A8mini/camera/image_rect`| `sensor_msgs/Image` | (현재 `image_raw`와 동일) |

#### 구독 토픽 (Subscribed Topics)

| 토픽 이름 | 메시지 타입 | 설명 |
| --- | --- | --- |
| `/A8mini/Gimbal/set_attitude` | `geometry_msgs/Point` | 짐벌 목표 자세 설정 (`Point.x`=Yaw, `Point.y`=Pitch) |
| `/A8mini/Gimbal/set_rotation` | `geometry_msgs/Vector3` | 짐벌 회전 속도 설정 (`Vector3.x`=Yaw속도, `Vector3.y`=Pitch속도) |

#### 서비스 (Services)

| 서비스 이름 | 서비스 타입 | 설명 |
| --- | --- | --- |
| `/A8mini/soft_reboot` | `std_srvs/srv/Trigger` | 짐벌과 카메라를 재부팅합니다. |
| `/A8mini/set_osd` | `std_srvs/srv/SetBool` | HDMI OSD를 켜거나 끕니다. |
| `/A8mini/Gimbal/mode/take_photo` | `std_srvs/srv/Trigger` | 사진을 촬영합니다. |
| `/A8mini/Gimbal/mode/toggle_video` | `std_srvs/srv/Trigger` | 비디오 녹화를 시작/중지합니다. |
| `/A8mini/Gimbal/mode/set_lock_mode` | `std_srvs/srv/Trigger` | 짐벌을 Lock 모드로 설정합니다. |
| `/A8mini/Gimbal/mode/set_follow_mode` | `std_srvs/srv/Trigger` | 짐벌을 Follow 모드로 설정합니다. |
| `/A8mini/Gimbal/mode/set_fpv_mode` | `std_srvs/srv/Trigger` | 짐벌을 FPV 모드로 설정합니다. |
| `/A8mini/Gimbal/mode/set_hdmi_output` | `std_srvs/srv/SetBool` | HDMI 출력을 켜거나 끕니다. |

### Troubleshooting / 문제 해결

**1. `exit code -6` or `-11` (Segmentation Fault) on startup**

  * **증상:** `video_stream_node`가 `exit code -6 (Abort)` 또는 `-11 (Segfault)`로 즉시 종료됩니다. `gimbal_control_node`는 정상 작동합니다.
  * **원인 1 (주요 원인): OpenCV 버전 충돌.** `pip`로 설치된 `opencv-python` (예: 4.11.x)과 `ros-humble-cv-bridge`가 요구하는 `apt` 버전 (예: 4.5.x)이 충돌합니다.
  * **해결책 1:** **반드시 `pip` 버전을 제거해야 합니다.** [Environment setup / 환경 구성](https://www.google.com/search?q=%23environment-setup--%ED%99%98%EA%B2%BD-%EA%B5%AC%EC%84%B1) 섹션을 참고하세요.
  * **원인 2: 임포트 순서 오류.** `import cv2`가 `rclpy.init()`보다 *먼저* 호출되었습니다.
  * **해결책 2:** 이 저장소의 `video_stream_node.py`는 `main()` 함수에서 `rclpy.init()`을 먼저 호출한 뒤 `cv2`를 임포트하여 이 문제를 해결했습니다.

**2. `404 Stream Not Found` in `video_stream_node` logs**

  * **증상:** 짐벌 노드는 작동하지만, 비디오 노드 로그에 `Failed to open RTSP stream`이 표시됩니다.
  * **원인:** RTSP URL이 카메라 모델과 맞지 않습니다. `main.264` (H.264)와 `video1` (H.265) 등이 있으며, 카메라마다 다릅니다.
  * **해결책:** `a8mini.launch.py` 파일을 열어 `rtsp_url` 파라미터의 기본값을 수정합니다. 먼저 `ffplay`로 작동하는 주소를 찾으세요:
    ```bash
    # H.264 스트림 테스트
    ffplay -flags low_delay rtsp://192.168.144.25:8554/main.264
    # H.265 스트림 (메인) 테스트
    ffplay -flags low_delay rtsp://192.168.144.25:8554/video1
    # H.265 스트림 (서브) 테스트
    ffplay -flags low_delay rtsp://192.168.144.25:8554/video2
    ```
    `ffplay`에서 작동하는 주소를 `a8mini.launch.py`에 적용합니다.

**3. RVIZ2 shows "No Image" (QoS Conflict)**

  * **증상:** `video_stream_node`는 실행 중이지만, RVIZ2에 "No Image"가 표시되고 터미널에 `[WARN] ... offering incompatible QoS. ... Last incompatible policy: RELIABILITY_QOS_POLICY` 로그가 나타납니다.
  * **원인:** `video_stream_node`는 저지연을 위해 `BEST_EFFORT`로 토픽을 발행하지만, RVIZ2의 "Image" 플러그인은 기본값으로 `RELIABLE`을 요구합니다.
  * **해결책:** RVIZ2의 "Image" 디스플레이 플러그인 설정에서 **Reliability Policy**를 `Reliable`에서 `Best Effort`로 변경합니다. 영상이 즉시 나타납니다.

**4. Video stream is laggy or corrupted (Grey Blocks)**

  * **증상:** RVIZ2 또는 디버그 창의 영상이 500ms 이상 지연되거나 회색 블록으로 깨집니다.
  * **원인:** `cv2.VideoCapture`가 FFMPEG 저지연 옵션을 사용하지 않고 있거나, 손실이 많은 네트워크에서 TCP(`use_udp=False`)를 사용 중입니다.
  * **해결책:**
    1.  `video_stream_node.py`의 `main()` 함수가 `rclpy.init()` *이후*, `import cv2` *이전*에 `os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "flags;low_delay|fflags;nobuffer"` 라인을 포함하고 있는지 확인합니다.
    2.  `video_stream_node.py`가 `SIYIRTSP` 객체 생성 시 `use_udp=True` (현재 코드의 기본값)로 설정되어 있는지 확인합니다.

### License / 라이선스

This project is licensed under the MIT License. (라이선스를 여기에 명시)

```
```