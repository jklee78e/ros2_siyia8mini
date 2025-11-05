"""
SIYI A8 Mini ROS2 드라이버 (mzahana/siyi_sdk 사용)

[최종 수정본 V5]
- GStreamer 파이프라인(PyGObject)을 노드에 직접 통합
- 'stream.py' 의존성 완전 제거
- GStreamer의 'appsink'를 사용하여 스레드 충돌 없이 프레임을 ROS 토픽으로 발행
- GLib.MainLoop를 별도 스레드에서 실행
- 디버그 창(cv2.imshow) 기능 제거 (RVIZ2로 확인)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import math
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge

# --- GStreamer 임포트 ---
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib

# --- 로컬 SDK 파일 임포트 ---
from .siyi_sdk import SIYISDK
# [삭제] from .stream import SIYIRTSP

# --- 메시지 및 서비스 타입 임포트 ---
from a8mini_msgs.msg import SystemInfo
from geometry_msgs.msg import Vector3Stamped, Point, Vector3
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo


def euler_from_quaternion(q):
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z # radians


class SiyiA8MiniNode(Node):
    def __init__(self):
        super().__init__('siyi_a8mini_node')
        
        # GStreamer 초기화
        Gst.init(None)
        
        # --- 1. 파라미터 ---
        self.declare_parameter('prefix', 'A8mini')
        self.declare_parameter('server_ip', '192.168.144.25')
        self.declare_parameter('port', 37260)
        # H.264 (main.264) 파이프라인 사용
        self.declare_parameter('gst_pipeline',
            # "rtspsrc location=rtsp://192.168.144.25:8554/main.264 "
            # "latency=41 udp-reconnect=1 timeout=0 do-retransmission=false ! "
            # "decodebin3 ! "
            # "queue max-size-buffers=1 leaky=2 ! "
            # "videoconvert ! video/x-raw,format=BGR ! "
            # "appsink name=ros_sink drop=1"
            ""
        )

        #         rtspsrc location=rtsp://192.168.144.25:8554/main.264 
        # latency=0 udp-reconnect=1 timeout=0 do-retransmission=false ! 
        # decodebin3 ! 
        # queue max-size-buffers=1 leaky=2 ! 
        # videoconvert ! 
        # autovideosink
        
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value
        server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        gst_pipeline_str = self.get_parameter('gst_pipeline').get_parameter_value().string_value

        self.get_logger().info(f"Node prefix: {self.prefix}")
        self.get_logger().info(f"Connecting to SIYI SDK at {server_ip}:{port}")
        
        # --- 2. SDK (Gimbal Control) 연결 ---
        self.sdk = SIYISDK(server_ip=server_ip, port=port)
        if not self.sdk.connect():
            self.get_logger().error("SIYI Gimbal SDK 연결 실패!")
            rclpy.shutdown()
            return
        self.get_logger().info("SIYI Gimbal SDK 연결 성공.")

        # --- 3. QoS ---
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        # 이미지 QoS (Best Effort)
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # --- 4. 퍼블리셔 (Gimbal) ---
        self.info_pub = self.create_publisher(
            SystemInfo, f'/{self.prefix}/info', latched_qos)
        self.attitude_pub = self.create_publisher(
            Vector3Stamped, f'/{self.prefix}/Gimbal/Info/attitude', 10)
        
        # --- 5. 퍼블리셔 (Camera Stream) ---
        self.get_logger().info("카메라 스트림 퍼블리셔 설정 중 (GStreamer 통합)...")
        self.bridge = CvBridge()
        
        self.image_raw_pub = self.create_publisher(
            Image, f'/{self.prefix}/camera/image_raw', image_qos)
        self.image_rect_pub = self.create_publisher(
            Image, f'/{self.prefix}/camera/image_rect', image_qos)
            
        self.cam_info_pub = self.create_publisher(
            CameraInfo, f'/{self.prefix}/camera/camera_info', latched_qos)
        self.camera_info_msg = None 
        
        # --- 6. 구독자 (Subscribers) ---
        self.set_attitude_sub = self.create_subscription(
            Point, f'/{self.prefix}/Gimbal/set_attitude', self.set_attitude_callback, 10)
        self.set_rotation_sub = self.create_subscription(
            Vector3, f'/{self.prefix}/Gimbal/set_rotation', self.set_rotation_callback, 10)
        self.get_logger().warn(f"Topic '{self.prefix}/Vehicle/send_attitude' is not implemented.")

        # --- 7. 서비스 (Services) ---
        # (이전과 동일, mzahana SDK 호환)
        self.reboot_srv = self.create_service(
            Trigger, f'/{self.prefix}/soft_reboot', self.reboot_callback)
        self.osd_srv = self.create_service(
            SetBool, f'/{self.prefix}/set_osd', self.set_osd_callback)
        self.photo_srv = self.create_service(
            Trigger, f'/{self.prefix}/Gimbal/mode/take_photo', self.take_photo_callback)
        self.video_srv = self.create_service(
            Trigger, f'/{self.prefix}/Gimbal/mode/toggle_video', self.toggle_video_callback)
        self.lock_mode_srv = self.create_service(
            Trigger, f'/{self.prefix}/Gimbal/mode/set_lock_mode', self.set_lock_mode_callback)
        self.follow_mode_srv = self.create_service(
            Trigger, f'/{self.prefix}/Gimbal/mode/set_follow_mode', self.set_follow_mode_callback)
        self.fpv_mode_srv = self.create_service(
            Trigger, f'/{self.prefix}/Gimbal/mode/set_fpv_mode', self.set_fpv_mode_callback)
        self.hdmi_srv = self.create_service(
            SetBool, f'/{self.prefix}/Gimbal/mode/set_hdmi_output', self.set_hdmi_output_callback)
        
        # --- 8. 타이머 (Timers) ---
        self.attitude_timer = self.create_timer(0.1, self.publish_attitude)
        self.initial_info_timer = self.create_timer(1.0, self.publish_initial_info)

        # --- 9. [수정됨] GStreamer 파이프라인 시작 ---
        self.get_logger().info(f"GStreamer 파이프라인 시작 중...\n{gst_pipeline_str}")
        self.pipeline = None
        self.gstreamer_loop = None
        try:
            self.pipeline = Gst.parse_launch(gst_pipeline_str)
            
            # 파이프라인에서 'ros_sink'라는 이름의 appsink 엘리먼트를 찾음
            appsink = self.pipeline.get_by_name("ros_sink")
            if not appsink:
                self.get_logger().error("파이프라인에서 'ros_sink'라는 이름의 appsink를 찾을 수 없습니다.")
                raise Exception("Appsink not found")

            # appsink 설정: 새 샘플(프레임)이 도착하면 시그널을 보내도록 함
            appsink.set_property("emit-signals", True)
            appsink.set_property("max-buffers", 1) # 버퍼를 1로 유지하여 지연 최소화
            
            # 'new-sample' 시그널을 self.on_new_frame 함수에 연결
            appsink.connect("new-sample", self.on_new_frame_callback)
            
            # GStreamer 메시지 처리를 위한 GLib.MainLoop 생성
            self.gstreamer_loop = GLib.MainLoop()
            
            # GStreamer 루프를 별도 스레드에서 실행
            self.gstreamer_thread = threading.Thread(target=self.gstreamer_loop.run)
            self.gstreamer_thread.start()
            
            # 파이프라인 실행 시작
            self.pipeline.set_state(Gst.State.PLAYING)
            self.get_logger().info("GStreamer 파이프라인 스레드 시작됨.")
            
        except Exception as e:
            self.get_logger().error(f"GStreamer 파이프라인 생성 실패: {e}")
            rclpy.shutdown()
            return

    def on_new_frame_callback(self, sink):
        """ GStreamer appsink에서 새 프레임이 도착할 때마다 호출되는 콜백 (GStreamer 스레드) """
        try:
            sample = sink.pull_sample()
            if sample is None:
                return Gst.FlowReturn.OK

            buf = sample.get_buffer()
            caps = sample.get_caps()
            struct = caps.get_structure(0)
            width = struct.get_value("width")
            height = struct.get_value("height")

            # GStreamer 버퍼를 OpenCV가 읽을 수 있는 numpy 배열로 변환
            ret, map_info = buf.map(Gst.MapFlags.READ)
            if ret:
                frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape(height, width, 3)
                
                now = self.get_clock().now().to_msg()
                
                # 1. CameraInfo 메시지 빌드 (첫 프레임에서 한 번만)
                if self.camera_info_msg is None:
                    self.get_logger().info(f"영상 스트림 수신 시작 (해상도: {width}x{height})")
                    self.build_camera_info(width, height)
                
                # 2. CameraInfo 발행
                self.camera_info_msg.header.stamp = now
                self.cam_info_pub.publish(self.camera_info_msg)

                # 3. OpenCV 이미지를 ROS Image 메시지로 변환
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = now
                img_msg.header.frame_id = f"{self.prefix}_camera_optical_frame"

                # 4. image_raw 및 image_rect 토픽 발행
                self.image_raw_pub.publish(img_msg)
                self.image_rect_pub.publish(img_msg)
                
                # 5. 메모리 매핑 해제
                buf.unmap(map_info)

            return Gst.FlowReturn.OK
        
        except Exception as e:
            self.get_logger().error(f"스트리밍 콜백 에러: {e}")
            return Gst.FlowReturn.ERROR


    def build_camera_info(self, width, height):
        """ A8 Mini 매뉴얼 스펙(81도 HFOV) 기반 CameraInfo 메시지 생성 """
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

        msg.k = [fx,  0.0, cx,
                 0.0, fy,  cy,
                 0.0, 0.0, 1.0]
        
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.distortion_model = "plumb_bob"

        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        
        msg.p = [fx,  0.0, cx,  0.0,
                 0.0, fy,  cy,  0.0,
                 0.0, 0.0, 1.0, 0.0]

        self.camera_info_msg = msg
        self.get_logger().info(f"CameraInfo 생성 완료: K={msg.k}")

    # (이하 짐벌 제어/서비스 콜백 함수들은 이전과 동일)
    
    def publish_initial_info(self):
        self.get_logger().info("Requesting initial gimbal info...")
        try:
            self.sdk.requestFirmwareVersion()
            self.sdk.requestHardwareID()
            self.sdk.requestGimbalInfo()
            time.sleep(0.5) 

            msg = SystemInfo()
            fw_ver = self.sdk.getFirmwareVersion() 
            msg.gimbal_fw_version = fw_ver if fw_ver else "N/A"
            msg.camera_fw_version = "N/A (SDK 미지원)"
            msg.hardware_id = self.sdk.getHardwareID() if self.sdk.getHardwareID() else "N/A"
            
            motion_mode = self.sdk.getMotionMode()
            if motion_mode is not None:
                msg.motion_mode = motion_mode
            
            mounting_dir = self.sdk.getMountingDirection()
            if mounting_dir is not None:
                msg.mounting_dir = mounting_dir

            self.get_logger().info(f"Publishing Info: {msg}")
            self.info_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed to publish initial info: {e}")
        finally:
            self.get_logger().info("Initial info timer canceling.")
            self.initial_info_timer.cancel()

    def publish_attitude(self):
        yaw_val, pitch_val, roll_val = self.sdk.getAttitude()
        
        if yaw_val is not None and pitch_val is not None and roll_val is not None:
            msg = Vector3Stamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"{self.prefix}_gimbal_base"
            msg.vector.x = float(roll_val)
            msg.vector.y = float(pitch_val)
            msg.vector.z = float(yaw_val)
            self.attitude_pub.publish(msg)
        else:
            self.get_logger().warn("Attitude data is still None.", throttle_duration_sec=5.0)

    def set_attitude_callback(self, msg: Point):
        self.get_logger().info(f"Setting attitude: Yaw={msg.x} deg, Pitch={msg.y} deg")
        self.sdk.requestSetAngles(yaw_deg=msg.x, pitch_deg=msg.y)

    def set_rotation_callback(self, msg: Vector3):
        yaw_speed = int(max(min(msg.x, 100.0), -100.0))
        pitch_speed = int(max(min(msg.y, 100.0), -100.0))
        self.get_logger().info(f"Setting rotation speed: Yaw={yaw_speed}, Pitch={pitch_speed}")
        self.sdk.requestGimbalSpeed(yaw_speed=yaw_speed, pitch_speed=pitch_speed)

    def reboot_callback(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self.get_logger().info("Rebooting gimbal and camera...")
            payload_hex = "0101" 
            cmd_id_hex = "80" 
            msg_hex = self.sdk._out_msg.encodeMsg(payload_hex, cmd_id_hex)
            self.sdk.sendMsg(msg_hex)
            res.success = True
            res.message = "Reboot command (0x80) sent."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def set_osd_callback(self, req: SetBool.Request, res: SetBool.Response):
        try:
            self.get_logger().info(f"Setting OSD to: {req.data}")
            payload_hex = "01" if req.data else "00"
            cmd_id_hex = "4C" 
            msg_hex = self.sdk._out_msg.encodeMsg(payload_hex, cmd_id_hex)
            self.sdk.sendMsg(msg_hex)
            res.success = True
            res.message = "OSD command (0x4C) sent."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def take_photo_callback(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self.get_logger().info("Taking photo...")
            self.sdk.requestPhoto()
            res.success = True
            res.message = "Photo requested."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def toggle_video_callback(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self.get_logger().info("Toggling video recording...")
            self.sdk.requestRecording()
            res.success = True
            res.message = "Video toggle requested."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def set_lock_mode_callback(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self.get_logger().info("Setting Lock Mode...")
            self.sdk.requestLockMode()
            res.success = True
            res.message = "Lock mode set."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def set_follow_mode_callback(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self.get_logger().info("Setting Follow Mode...")
            self.sdk.requestFollowMode()
            res.success = True
            res.message = "Follow mode set."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def set_fpv_mode_callback(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self.get_logger().info("Setting FPV Mode...")
            self.sdk.requestFPVMode()
            res.success = True
            res.message = "FPV mode set."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res
    
    def set_hdmi_output_callback(self, req: SetBool.Request, res: SetBool.Response):
        try:
            self.get_logger().info(f"Setting HDMI Output to: {req.data}")
            if req.data: 
                payload_hex = "06"
            else: 
                payload_hex = "08"
            
            cmd_id_hex = self.sdk._out_msg.COMMAND.PHOTO_VIDEO_HDR
            msg_hex = self.sdk._out_msg.encodeMsg(payload_hex, cmd_id_hex)
            
            self.sdk.sendMsg(msg_hex)
            res.success = True
            res.message = f"HDMI command ({payload_hex}) sent."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def destroy_node(self):
        """ 노드 종료 시 GStreamer와 SDK 연결을 안전하게 해제합니다. """
        self.get_logger().info("Shutting down...")
        
        # GStreamer 스레드 중지
        if hasattr(self, 'gstreamer_loop') and self.gstreamer_loop.is_running():
            self.get_logger().info("GStreamer 루프 종료 중...")
            self.gstreamer_loop.quit()
        if hasattr(self, 'pipeline'):
            self.pipeline.set_state(Gst.State.NULL)
        if hasattr(self, 'gstreamer_thread'):
            self.gstreamer_thread.join(timeout=2.0)
        
        # SDK 연결 해제
        if hasattr(self, 'sdk'):
            self.sdk.disconnect()
            
        self.get_logger().info("SDK 연결 해제됨.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SiyiA8MiniNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()