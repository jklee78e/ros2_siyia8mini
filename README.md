# siyi_a8mini / a8mini_msgs

English | 한국어
---|---
ROS2 driver & SDK for the A8Mini platform (video, gimbal, telemetry messages) | A8Mini 플랫폼용 ROS2 드라이버 및 SDK (비디오, 짐벌, 텔레메트리 메시지)

## Table of Contents / 목차

- [Overview / 개요](#overview--개요)
- [Features / 주요 기능](#features--주요-기능)
- [Repository Structure / 저장소 구조](#repository-structure--저장소-구조)
- [Environment setup / 환경 구성](#environment-setup--환경-구성)
- [Installation / 설치 방법](#installation--설치-방법)
  - [From source / 소스에서 설치](#from-source--소스에서-설치)
  - [Python packages / Python 패키지 설치](#python-packages--python-패키지-설치)
- [Usage / 사용법](#usage--사용법)
  - [Launching nodes / 노드 실행](#launching-nodes--노드-실행)
  - [Running individual nodes / 개별 노드 실행](#running-individual-nodes--개별-노드-실행)
  - [Inspecting topics / 토픽 확인하기](#inspecting-topics--토픽-확인하기)
  - [Examples / 예제](#examples--예제)
- [Packages / 패키지 안내](#packages--패키지-안내)
- [API Reference / API 레퍼런스](#api-reference--api-레퍼런스)
- [Testing / 테스트](#testing--테스트)
- [Development / 개발 가이드](#development--개발-가이드)
- [Contributing / 기여 방법](#contributing--기여-방법)
- [License / 라이선스](#license--라이선스)

## Overview / 개요

English

This repository contains two ROS2 packages:

- `a8mini_msgs`: custom ROS messages used by the A8Mini stack.
- `siyi_a8mini`: Python ROS2 nodes, SDK and utilities (camera streaming, gimbal control, message handling).

한국어

이 저장소에는 다음의 ROS2 패키지가 포함되어 있습니다:

- `a8mini_msgs`: A8Mini 스택에서 사용하는 커스텀 ROS 메시지입니다.
- `siyi_a8mini`: Python으로 작성된 ROS2 노드, SDK 및 유틸리티(카메라 스트리밍, 짐벌 제어, 메시지 처리).

## Features / 주요 기능

- Video streaming node
- Gimbal control node
- SDK utilities for message encoding/decoding and CRC
- Test suite for smoke checks

## Repository Structure / 저장소 구조

Top-level `src` folder contains:

- `a8mini_msgs/` — message definitions and headers
- `siyi_a8mini/` — Python package with nodes and SDK
- `test/` — pytest-based tests

## Environment setup / 환경 구성

English

Prerequisites:

- Ubuntu 20.04 or 22.04 (recommended)
- ROS2 (Foxy/Galactic/Humble/Rolling) — use the distribution you target
- Python 3.8+ (system Python provided by ROS2; virtualenv/venv recommended for development)

Install system dependencies (example for Ubuntu + ROS2 Humble):

```bash
# Setup ROS2 sources (if not already done)
sudo apt update
sudo apt install -y curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update

# Install ROS2 (Humble example)
sudo apt install -y ros-humble-desktop

# Install common build tools
sudo apt install -y python3-colcon-common-extensions python3-pip python3-venv
```

Python packages used by this repo (install in a virtualenv or via pip):

```bash
# create and activate venv (optional but recommended)
python3 -m venv .venv
source .venv/bin/activate

# Install project dependencies (pin versions via requirements.txt)
pip install -U pip setuptools wheel
pip install -r siyi_a8mini/requirements.txt

# If you need system-level GStreamer/FFmpeg on Ubuntu:
sudo apt install -y gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav ffmpeg
```

Notes:

- If you use a different ROS2 distribution, replace package names (e.g., `ros-galactic-desktop`).
- Some packages (GStreamer, ffmpeg) may be required for video streaming depending on your node implementations.

한국어

사전 조건:

- 권장 OS: Ubuntu 20.04 또는 22.04
- ROS2 (목표로 하는 배포판 사용: Foxy/Galactic/Humble 등)
- Python 3.8 이상 (개발 시 가상환경 권장)

시스템 패키지 설치 예시 (Ubuntu + ROS2 Humble):

```bash
# ROS2 소스 추가 (이미 설정되어 있다면 건너뜀)
sudo apt update
sudo apt install -y curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update

# ROS2 설치 (Humble 예시)
sudo apt install -y ros-humble-desktop

# 빌드 도구 설치
sudo apt install -y python3-colcon-common-extensions python3-pip python3-venv
```

Python 패키지 설치:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip setuptools wheel
pip install -r siyi_a8mini/requirements.txt 2>/dev/null || pip install pytest opencv-python numpy
```

주의사항:

- ROS2 배포판이 다르면 패키지 이름을 바꿔 설치하세요 (예: `ros-galactic-desktop`).
- 비디오 스트리밍은 노드 구현에 따라 GStreamer 또는 ffmpeg 같은 추가 라이브러리가 필요할 수 있습니다.

## Installation / 설치 방법

### From source / 소스에서 설치

```bash
# in the workspace root that contains src/
source /opt/ros/humble/setup.bash  # change 'humble' to your distribution
colcon build --packages-select a8mini_msgs siyi_a8mini
source install/setup.bash
```

### Python packages / Python 패키지 설치

```bash
# editable install (optional)
pip install -e siyi_a8mini
```

## Usage / 사용법

### Launching nodes / 노드 실행

Start the full stack using the provided launch file:

```bash
# from workspace root
source install/setup.bash
ros2 launch siyi_a8mini a8mini.launch.py
```

If your ROS2 distribution is different, source the appropriate setup script (e.g., `source /opt/ros/galactic/setup.bash`).

### Running individual nodes / 개별 노드 실행

You can run a node directly with `ros2 run` (after sourcing `install/setup.bash`):

```bash
ros2 run siyi_a8mini gimbal_control_node
ros2 run siyi_a8mini video_stream_node
```

If you prefer running the module directly for debugging, run the Python entry point:

```bash
python3 -m siyi_a8mini.video_stream_node
python3 -m siyi_a8mini.gimbal_control_node
```

### Inspecting topics / 토픽 확인하기

List topics:

```bash
ros2 topic list
```

Echo a topic (example: gimbal attitude):

```bash
ros2 topic echo /A8mini/Gimbal/Info/attitude
```

View available services:

```bash
ros2 service list
```

Call a service (example: soft reboot):

```bash
ros2 service call /A8mini/soft_reboot std_srvs/srv/Trigger
```

### Examples / 예제

1) Start video stream node and display frames (local machine):

```bash
# Start node
ros2 run siyi_a8mini video_stream_node

# On another terminal, view topics
ros2 topic list
ros2 topic echo /A8mini/camera/image_raw
```

2) Publish a set-attitude command (example using `ros2 topic pub`):

```bash
# Set yaw,x pitch,y in degrees using Point message
ros2 topic pub /A8mini/Gimbal/set_attitude geometry_msgs/Point "{x: 10.0, y: -5.0, z: 0.0}"
```

3) Toggle recording via service call:

```bash
ros2 service call /A8mini/Gimbal/mode/toggle_video std_srvs/srv/Trigger
```

### Troubleshooting / 문제 해결

- If `cv2` import fails, ensure `opencv-python` is installed in your venv. On Ubuntu you can also install system OpenCV:

```bash
sudo apt install -y python3-opencv
```

- If RTSP stream fails with high latency, try installing system `ffmpeg` and GStreamer plugins and prefer UDP transport by adjusting the `rtsp_url` parameter.

- If nodes crash with segfaults when importing `rclpy` + `cv2`, ensure `rclpy.init()` is called after importing OpenCV or avoid importing GUI-backed OpenCV modules before ROS2 initialization (some drivers/platforms have conflicts).

## Packages / 패키지 안내

- `a8mini_msgs` — message definitions located in `a8mini_msgs/msg/`
- `siyi_a8mini` — nodes and SDK in `siyi_a8mini/`

## Testing / 테스트

Run pytest from the repository root:

```bash
source .venv/bin/activate
pytest -q
```

## Contributing / 기여 방법

Please open issues and pull requests. Follow the existing code style. Run tests locally before submitting PRs.

## License / 라이선스

Specify your license here (e.g., MIT or Apache-2.0). If there's a `LICENSE` file, mention it.

---

If you'd like, I can:

- Create separate `README.en.md` and `README.ko.md` files instead of a combined bilingual file.
- Add badges (CI, coverage, license).
- Generate a minimal `requirements.txt` in `siyi_a8mini/` if missing.

Let me know which extras you want and I'll add them.
