#!/usr/bin/env python3
import os
import cv2
print(f"OpenCV version: {cv2.__version__}")

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "protocol_whitelist;tcp,rtp,udp"

cap = cv2.VideoCapture("rtsp://192.168.144.25:8554/main.264", cv2.CAP_FFMPEG)
# cap = cv2.VideoCapture("rtsp://192.168.144.25:8554/main.264"1)
# cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다. 종료합니다.")
        break

    cv2.imshow('RTSP 스트림', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

