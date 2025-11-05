"""
SIYI A8 Mini ROS2 드라이버 - 짐벌 제어 노드
(siyi_sdk만 사용. OpenCV/GStreamer 의존성 없음)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import sys

# --- 로컬 SDK 파일 임포트 ---
from .siyi_sdk import SIYISDK

# --- 메시지 및 서비스 타입 임포트 ---
from a8mini_msgs.msg import SystemInfo
from geometry_msgs.msg import Vector3Stamped, Point, Vector3
from std_srvs.srv import Trigger, SetBool

class SiyiGimbalControlNode(Node):
    def __init__(self):
        super().__init__('siyi_gimbal_control_node')
        self.get_logger().info("[Gimbal Node] __init__ 시작.")
        
        # --- 1. 파라미터 ---
        self.declare_parameter('prefix', 'A8mini')
        self.declare_parameter('server_ip', '192.168.144.25')
        self.declare_parameter('port', 37260)
        
        self.prefix = self.get_parameter('prefix').get_parameter_value().string_value
        server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

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

        # --- 4. 퍼블리셔 (Gimbal) ---
        self.info_pub = self.create_publisher(
            SystemInfo, f'/{self.prefix}/info', latched_qos)
        self.attitude_pub = self.create_publisher(
            Vector3Stamped, f'/{self.prefix}/Gimbal/Info/attitude', 10)
        
        # --- 6. 구독자 (Subscribers) ---
        self.set_attitude_sub = self.create_subscription(
            Point, f'/{self.prefix}/Gimbal/set_attitude', self.set_attitude_callback, 10)
        self.set_rotation_sub = self.create_subscription(
            Vector3, f'/{self.prefix}/Gimbal/set_rotation', self.set_rotation_callback, 10)
        self.get_logger().warn(f"Topic '{self.prefix}/Vehicle/send_attitude' is not implemented.")

        # --- 7. 서비스 (Services) ---
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
        self.get_logger().info("[Gimbal Node] __init__ 모든 과정 성공적으로 완료.")

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
        self.sdk.requestSetAngles(yaw_deg=msg.x, pitch_deg=msg.y)

    def set_rotation_callback(self, msg: Vector3):
        yaw_speed = int(max(min(msg.x, 100.0), -100.0))
        pitch_speed = int(max(min(msg.y, 100.0), -100.0))
        self.sdk.requestGimbalSpeed(yaw_speed=yaw_speed, pitch_speed=pitch_speed)

    def reboot_callback(self, req: Trigger.Request, res: Trigger.Response):
        try:
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
            self.sdk.requestPhoto()
            res.success = True
            res.message = "Photo requested."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def toggle_video_callback(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self.sdk.requestRecording()
            res.success = True
            res.message = "Video toggle requested."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def set_lock_mode_callback(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self.sdk.requestLockMode()
            res.success = True
            res.message = "Lock mode set."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def set_follow_mode_callback(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self.sdk.requestFollowMode()
            res.success = True
            res.message = "Follow mode set."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def set_fpv_mode_callback(self, req: Trigger.Request, res: Trigger.Response):
        try:
            self.sdk.requestFPVMode()
            res.success = True
            res.message = "FPV mode set."
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res
    
    def set_hdmi_output_callback(self, req: SetBool.Request, res: SetBool.Response):
        try:
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
        self.get_logger().info("Shutting down...")
        if hasattr(self, 'sdk'):
            self.sdk.disconnect()
        self.get_logger().info("SDK 연결 해제됨.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    print("[DEBUG][Gimbal] rclpy.init() 완료. SiyiGimbalControlNode 객체 생성 시도...")
    node = SiyiGimbalControlNode()
    print("[DEBUG][Gimbal] SiyiGimbalControlNode 객체 생성 완료. rclpy.spin() 진입...")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[DEBUG][Gimbal] KeyboardInterrupt 수신. 종료 프로세스 시작...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[DEBUG][Gimbal] 노드 종료 및 셧다운 완료.")

if __name__ == '__main__':
    main()