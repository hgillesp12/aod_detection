from setuptools import setup
from glob import glob
import os

package_name = 'drone_video_feed'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge', 'torch', 'opencv-python', 'ultralytics'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A ROS 2 package for drone video feed and machine learning processing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_feed_node = drone_video_feed.video_feed_node:main',
            'quadcopter_controller = drone_video_feed.quadcopter_controller:main',
        ],
    },
)
