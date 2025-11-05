"""
SIYI A8 Mini ROS2 드라이버 - 비디오 스트림 노드

[최종 수정본 V15]
- Segfault 해결 (rclpy/issues/1149): rclpy.init()을 cv2, cv_bridge, stream.py 임포트 *전에* 호출
- mzahana/siyi_sdk의 stream.py (SIYIRTSP)를 직접 사용
- GStreamer/v4l2loopback 의존성 제거
- ffplay 저지연 옵션(nobuffer, low_delay)을 FFMPEG 백엔드에 적용
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import math
import threading
import numpy as np
import sys
import os # 저지연 옵션을 위해 os.environ 사용

# --- [핵심 수정] ---
# cv2, cv_bridge, stream.py는 rclpy.init()이 호출된 *이후*에
# main() 함수 또는 Node 생성자 내에서 임포트해야 Segfault(-6)를 피할 수 있습니다.
# --------------------

# --- 메시지 및 서비스 타입 임포트 ---
from sensor_msgs.msg import Image, CameraInfo
print("[DEBUG][Video] 기본 모듈 임포트 성공.")

# cv2, CvBridge, SIYIRTSP를 전역 변수로 선언 (None으로 초기화)
cv2 = None
CvBridge = None
SIYIRTSP = None

class SiyiVideoStreamNode(Node):
    def __init__(self):
        super().__init__('siyi_video_stream_node')
        self.get_logger().info("[Video Node] __init__ 시작 (stream.py 사용).")
        
        # --- [핵심 수정] cv2, cv_bridge, stream.py 임포트 ---
        global cv2, CvBridge, SIYIRTSP
        if cv2 is None:
            try:
                # [저지연 설정] ffplay 옵션을 OpenCV FFMPEG 백엔드에 적용
                # import cv2 전에 환경 변수를 설정해야 함
                os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|flags;low_delay"
                
                import cv2
                from cv_bridge import CvBridge
                from .stream import SIYIRTSP # 로컬 stream.py 임포트
                self.get_logger().info(f"[DEBUG][Video] OpenCV/cv_bridge/stream.py 임포트 성공. OpenCV 버전: {cv2.__version__}")
            except ImportError as e:
                self.get_logger().error(f"[DEBUG][Video] CRITICAL: OpenCV, cv_bridge 또는 stream.py 임포트 실패: {e}")
                sys.exit(1)
            except Exception as e:
                self.get_logger().error(f"[DEBUG][Video] CRITICAL: 임포트 중 알 수 없는 충돌: {e}")
                sys.exit(1)
        # ----------------------------------------
        
        # --- 1. 파라미터 ---
        self.declare_parameter('prefix', 'A8mini')
        self.declare_parameter('rtsp_url', 'rtsp://192.168.144.25:8554/main.264')
        self.declare_parameter('use_udp', True)
        
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value
        rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
        use_udp = self.get_parameter('use_udp').get_parameter_value().string_value

        # --- 3. QoS ---
        latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 이미지 QoS (Reliable) - RVIZ2 기본값과 맞춤
        image_qos = QoSProfile(
            depth=2, # Reliable QoS는 Best Effort(1)보다 버퍼가 더 필요
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- 5. 퍼블리셔 (Camera Stream) ---
        self.get_logger().info(f"[DEBUG] 카메라 스트림 퍼블리셔 설정 중 ({rtsp_url})...")
        self.bridge = CvBridge()
        
        self.image_raw_pub = self.create_publisher(Image, f'/{self.prefix}/camera/image_raw', image_qos)
        self.image_rect_pub = self.create_publisher(Image, f'/{self.prefix}/camera/image_rect', image_qos)
        self.cam_info_pub = self.create_publisher(CameraInfo, f'/{self.prefix}/camera/camera_info', latched_qos)
        self.camera_info_msg = None 
        
        # --- 9. [수정됨] SIYIRTSP 스레드 시작 ---
        self.get_logger().info(f"RTSP 스트림 연결 시도: {rtsp_url}")
        
        # stream.py의 SIYIRTSP 객체 생성 (UDP 사용, 디버그 창 켜기)
        self.rtsp = SIYIRTSP(rtsp_url=rtsp_url, cam_name=self.prefix, use_udp=use_udp, debug=False)
        self.rtsp.setShowWindow(True) # 요청하신 디버그 창 켜기
        
        self._stop_streaming = False
        self.stream_thread = threading.Thread(target=self.stream_loop)
        self.stream_thread.start()
        self.get_logger().info("영상 스트리밍 스레드 시작됨.")

    def stream_loop(self):
        """ stream.py의 getFrame()을 호출하여 프레임을 읽고 ROS 토픽으로 발행 """
        while rclpy.ok() and not self._stop_streaming:
            try:
                # stream.py의 getFrame() 함수 사용
                frame = self.rtsp.getFrame()
                
                if frame is None:
                    # stream.py가 내부적으로 재연결을 시도하므로 기다림
                    time.sleep(0.01) # 100Hz
                    continue

                now = self.get_clock().now().to_msg()
                
                if self.camera_info_msg is None:
                    height, width, _ = frame.shape
                    self.get_logger().info(f"영상 스트림 수신 시작 (해상도: {width}x{height})")
                    self.build_camera_info(width, height)
                
                self.camera_info_msg.header.stamp = now
                self.cam_info_pub.publish(self.camera_info_msg)

                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = now
                img_msg.header.frame_id = f"{self.prefix}_camera_optical_frame"

                self.image_raw_pub.publish(img_msg)
                self.image_rect_pub.publish(img_msg)
                
                # stream.py의 loop() 함수에 이미 sleep(0.001)이 있으므로 여기서는 sleep 없음
                
            except Exception as e:
                self.get_logger().error(f"스트리밍 콜백 에러: {e}")
                time.sleep(1.0)

    def build_camera_info(self, width, height):
        # (이전과 동일)
        self.get_logger().info("매뉴얼 스펙 기반 CameraInfo 생성 중...")
        msg = CameraInfo()
        msg.header.frame_id = f"{self.prefix}_camera_optical_frame"
        msg.height = height
        msg.width = width
        fov_h_rad = 81.0 * math.pi / 180.0
        fx = float(width) / (2.0 * math.tan(fov_h_rad / 2.0))
        fy = fx 
        cx = float(width) / 2.0
        cy = float(height) / 2.0
        msg.k = [fx,  0.0, cx, 0.0, fy,  cy, 0.0, 0.0, 1.0]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.distortion_model = "plumb_bob"
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx,  0.0, cx,  0.0, 0.0, fy,  cy,  0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_msg = msg
        self.get_logger().info(f"CameraInfo 생성 완료: K={msg.k}")

    def destroy_node(self):
        self.get_logger().info("Shutting down...")
        self._stop_streaming = True
        
        if hasattr(self, 'rtsp'):
            self.rtsp.close() # stream.py의 close() 함수 호출
            
        if hasattr(self, 'stream_thread'):
            self.stream_thread.join(timeout=2.0)
            
        self.get_logger().info("VideoCapture 연결 해제됨.")
        super().destroy_node()

def main(args=None):
    # [핵심 수정] rclpy.init()을 모든 임포트(특히 cv2/stream.py)보다 먼저 호출
    rclpy.init(args=args)
    
    print("[DEBUG][Video] rclpy.init() 완료. SiyiVideoStreamNode 객체 생성 시도...")
    node = SiyiVideoStreamNode()
    print("[DEBUG][Video] SiyiVideoStreamNode 객체 생성 완료. rclpy.spin() 진입...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[DEBUG][Video] KeyboardInterrupt 수신. 종료 프로세스 시작...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[DEBUG][Video] 노드 종료 및 셧다운 완료.")

if __name__ == '__main__':
    main()