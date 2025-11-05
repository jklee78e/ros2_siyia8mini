#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
# --- QoS 프로파일 임포트 추가 ---
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# --- 사용할 메시지 및 서비스 타입 임포트 ---
from a8mini_msgs.msg import SystemInfo
from geometry_msgs.msg import Vector3Stamped, Point, Vector3
from std_srvs.srv import Trigger, SetBool

# 테스트할 노드의 고정된 Prefix
NODE_PREFIX = "A8mini"

class A8MiniTester(Node):
    def __init__(self):
        super().__init__('a8mini_tester_node')
        self.get_logger().info("SIYI A8 Mini 테스트 노드 시작...")

        self.info_received = False
        self.attitude_received = False

        # QoS 프로파일 (Latched 토픽 구독용)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- 토픽 구독자 (노드에서 오는 데이터 확인용) ---
        self.info_sub = self.create_subscription(
            SystemInfo,
            f'/{NODE_PREFIX}/info',
            self.info_callback,
            latched_qos) # <-- QoS 프로파일 적용
        
        self.att_sub = self.create_subscription(
            Vector3Stamped,
            f'/{NODE_PREFIX}/Gimbal/Info/attitude',
            self.attitude_callback,
            10)

        # --- 토픽 퍼블리셔 (노드로 명령 전송용) ---
        self.pos_pub = self.create_publisher(Point, f'/{NODE_PREFIX}/Gimbal/set_attitude', 10)
        self.rot_pub = self.create_publisher(Vector3, f'/{NODE_PREFIX}/Gimbal/set_rotation', 10)

        # --- 서비스 클라이언트 (노드의 기능 호출용) ---
        self.photo_cli = self.create_client(Trigger, f'/{NODE_PREFIX}/Gimbal/mode/take_photo')
        self.video_cli = self.create_client(Trigger, f'/{NODE_PREFIX}/Gimbal/mode/toggle_video')
        self.lock_cli = self.create_client(Trigger, f'/{NODE_PREFIX}/Gimbal/mode/set_lock_mode')
        self.follow_cli = self.create_client(Trigger, f'/{NODE_PREFIX}/Gimbal/mode/set_follow_mode')
        self.fpv_cli = self.create_client(Trigger, f'/{NODE_PREFIX}/Gimbal/mode/set_fpv_mode')
        self.osd_cli = self.create_client(SetBool, f'/{NODE_PREFIX}/set_osd')
        self.hdmi_cli = self.create_client(SetBool, f'/{NODE_PREFIX}/Gimbal/mode/set_hdmi_output')
        self.reboot_cli = self.create_client(Trigger, f'/{NODE_PREFIX}/soft_reboot')

        self.all_clients = [
            self.photo_cli, self.video_cli, self.lock_cli, self.follow_cli, 
            self.fpv_cli, self.osd_cli, self.hdmi_cli, self.reboot_cli
        ]
        self.get_logger().info("서비스 서버를 기다리는 중...")

    # --- 콜백 함수 (토픽 수신 확인용) ---
    def info_callback(self, msg):
        if not self.info_received:
            self.get_logger().info(f"✅ [테스트 1] '/info' 토픽 수신 성공:\n{msg}")
            self.info_received = True

    def attitude_callback(self, msg):
        if not self.attitude_received:
            self.get_logger().info(f"✅ [테스트 1] '/Gimbal/Info/attitude' 토픽 수신 성공: (Yaw={msg.vector.z:.2f})")
            self.attitude_received = True

    # --- 서비스 호출 헬퍼 함수 ---
    def call_trigger_service(self, client: rclpy.client.Client, service_name: str):
        self.get_logger().info(f"⚡ [테스트 3] 서비스 호출: {service_name}")
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"🚨 서비스 '{service_name}'를 찾을 수 없습니다.")
            return

        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result():
            self.get_logger().info(f"✅ 서비스 응답: {future.result().message}")
        else:
            self.get_logger().error(f"🚨 서비스 호출 실패: {service_name}")

    def call_setbool_service(self, client: rclpy.client.Client, service_name: str, data: bool):
        self.get_logger().info(f"⚡ [테스트 3] 서비스 호출: {service_name} (Data: {data})")
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"🚨 서비스 '{service_name}'를 찾을 수 없습니다.")
            return

        req = SetBool.Request()
        req.data = data
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result():
            self.get_logger().info(f"✅ 서비스 응답: {future.result().message}")
        else:
            self.get_logger().error(f"🚨 서비스 호출 실패: {service_name}")

    # --- 메인 테스트 실행 함수 ---
    def run_all_tests(self):
        # 1. 서비스가 모두 켜질 때까지 대기
        for client in self.all_clients:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"🚨 치명적: 서비스 '{client.srv_name}'를 사용할 수 없습니다. 메인 노드가 실행 중인지 확인하세요.")
                return

        self.get_logger().info("✅ 모든 서비스가 활성화되었습니다. 3초 후 테스트를 시작합니다.")
        time.sleep(3.0)

        # --- [수정됨] 테스트 1: 토픽 수신 확인 ---
        self.get_logger().info("--- 1. 토픽 수신 테스트 (Info, Attitude) ---")
        self.get_logger().info("Latched /info 토픽을 즉시 확인합니다...")
        
        # Latched 토픽을 즉시 받을 수 있도록 spin_once를 먼저 호출
        # info_callback이 QoS 설정으로 인해 즉시 호출될 것입니다.
        rclpy.spin_once(self, timeout_sec=0.5) 
        
        start_time = self.get_clock().now()
        while rclpy.ok() and (not self.info_received or not self.attitude_received):
            rclpy.spin_once(self, timeout_sec=0.1)
            # 타임아웃을 10초로 늘림
            if (self.get_clock().now() - start_time).nanoseconds > 10e9: 
                if not self.info_received:
                    self.get_logger().error("🚨 테스트 1 실패: /info 토픽을 수신하지 못했습니다.")
                if not self.attitude_received:
                     self.get_logger().error("🚨 테스트 1 실패: /Gimbal/Info/attitude 토픽을 수신하지 못했습니다.")
                break
        
        time.sleep(1.0)

        # --- 테스트 2: 짐벌 제어 (토픽 발행) ---
        self.get_logger().info("--- 2. 짐벌 제어 테스트 (Topic Publish) ---")
        self.get_logger().info("짐벌을 보세요! 2초간 Yaw(x) 50 속도로 회전합니다...")
        self.rot_pub.publish(Vector3(x=50.0, y=0.0, z=0.0))
        time.sleep(2.0)
        
        self.get_logger().info("짐벌 정지...")
        self.rot_pub.publish(Vector3(x=0.0, y=0.0, z=0.0))
        time.sleep(1.0)

        self.get_logger().info("짐벌을 보세요! Pitch(y) -30도 위치로 이동합니다...")
        self.pos_pub.publish(Point(x=0.0, y=-30.0, z=0.0))
        time.sleep(2.0)
        
        self.get_logger().info("짐벌 중앙 (0, 0) 위치로 복귀합니다...")
        self.pos_pub.publish(Point(x=0.0, y=0.0, z=0.0))
        time.sleep(2.0)

        # --- 테스트 3: 기능 호출 (서비스) ---
        self.get_logger().info("--- 3. 기능 테스트 (Service Call) ---")
        
        self.call_trigger_service(self.follow_cli, "Follow 모드 설정")
        time.sleep(1.0)
        
        self.get_logger().info("짐벌을 보세요! 사진을 촬영합니다...")
        self.call_trigger_service(self.photo_cli, "사진 촬영")
        time.sleep(1.0)

        self.get_logger().info("비디오 녹화를 시작합니다...")
        self.call_trigger_service(self.video_cli, "비디오 녹화 (시작)")
        time.sleep(2.0)
        self.get_logger().info("비디오 녹화를 중지합니다...")
        self.call_trigger_service(self.video_cli, "비디오 녹화 (중지)")
        time.sleep(1.0)

        self.call_setbool_service(self.osd_cli, "OSD 켜기", True)
        time.sleep(1.0)
        
        self.call_setbool_service(self.osd_cli, "OSD 끄기", False)
        time.sleep(1.0)

        self.call_setbool_service(self.hdmi_cli, "HDMI 출력 켜기", True)
        time.sleep(1.0)

        self.call_trigger_service(self.lock_cli, "Lock 모드 설정")
        time.sleep(1.0)
        
        self.call_trigger_service(self.fpv_cli, "FPV 모드 설정")
        time.sleep(1.0)

        self.get_logger().info("========================================")
        self.get_logger().info("✅ 모든 자동 테스트가 완료되었습니다.")
        self.get_logger().info("🚨 'reboot' 테스트는 위험하므로 수동으로 실행하세요.")
        self.get_logger().info("테스트 노드를 3초 후 종료합니다.")
        time.sleep(3.0)


def main(args=None):
    rclpy.init(args=args)
    tester = A8MiniTester()
    
    # 테스트 실행
    tester.run_all_tests()
    
    # 테스트 완료 후 노드 종료
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()