from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'siyi_a8mini'

setup(
    name=package_name,
    version='0.0.0',
    
    # [핵심 수정]
    # 'src' 디렉토리를 사용하지 않고, setup.py와 동일한 위치에 있는
    # 모든 파이썬 패키지(siyi_a8mini 폴더)를 찾도록 수정합니다.
    packages=find_packages(exclude=['test']),
    # [핵심 수정] package_dir={'': 'src'} 라인을 완전히 삭제합니다.

    data_files=[
        # 리소스 마커 파일 (패키지를 찾는 데 필수)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # 런치 파일
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    # siyi-sdk는 .py 파일로 직접 복사했으므로 install_requires에서 제외
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kiie',
    maintainer_email='kiie@todo.todo',
    description='ROS2 Driver for SIYI A8 Mini',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'gimbal_control_node = siyi_a8mini.gimbal_control_node:main',
            'video_stream_node = siyi_a8mini.video_stream_node:main',
        ],
    },
)