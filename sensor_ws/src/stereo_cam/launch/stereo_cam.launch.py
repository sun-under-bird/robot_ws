#!/usr/bin/env python3
"""启动拼接双目相机节点。改参数直接在这里改。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stereo_cam',
            executable='stereo_cam_node',
            name='stereo_cam_node',
            output='screen',
            parameters=[{
                'device': '/dev/video0',
                'width': 1280,   
                'height': 480,
                'fps': 15,            
                'pixel_format': 'MJPG',
                'frame_id': 'camera',
                'encoding': 'mono8', 
                'time_offset': 0.0,   # Kalibr标定后填曝光->打戳的固定延迟(秒)
            }],
        ),
    ])
