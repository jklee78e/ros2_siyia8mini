from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # 파라미터 정의
    prefix = LaunchConfiguration('prefix', default='A8mini')
    server_ip = LaunchConfiguration('server_ip', default='192.168.144.25')
    port = LaunchConfiguration('port', default=37260)
    
    # FFplay에서 확인된 H.264 주소
    rtsp_url = LaunchConfiguration('rtsp_url', default='rtsp://192.168.144.25:8554/main.264')
    use_udp = LaunchConfiguration('use_udp', default=True)

    return LaunchDescription([
        
        # 1. 짐벌 제어 노드
        Node(
            package='siyi_a8mini',
            executable='gimbal_control_node',
            name='siyi_gimbal_control_node',
            output='screen',
            parameters=[
                {'prefix': prefix},
                {'server_ip': server_ip},
                {'port': port}
            ]
        ),
        
        # 2. 비디오 스트림 노드 (stream.py 사용)
        Node(
            package='siyi_a8mini',
            executable='video_stream_node',
            name='siyi_video_stream_node',
            output='screen',
            parameters=[
                {'prefix': prefix},
                {'rtsp_url': rtsp_url},
                {'use_udp': use_udp}
            ]
        )
    ])