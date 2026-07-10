#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('stereo_camera_pkg_py'), 'config'
    )
    bag_path = LaunchConfiguration('bag_path')
    play_rate = LaunchConfiguration('play_rate')

    # 标定文件路径（传给 stereo_info.py 的 ROS 参数）
    left_yaml = LaunchConfiguration('left_yaml')
    right_yaml = LaunchConfiguration('right_yaml')

    # 1) 播放 bag（假设 bag 里有 /image_raw/compressed）
    play_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            bag_path,
            '--rate', play_rate,
            # '--loop',   # 需要循环就取消注释
        ],
        output='screen'
    )

    # 2) 解码 compressed -> raw
    # 输入：/image_raw/compressed  (sensor_msgs/CompressedImage)
    # 输出：/image_raw            (sensor_msgs/Image)
    decompress = Node(
        package='image_transport',
        executable='republish',
        name='republish_mjpeg_to_raw',
        output='screen',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in/compressed', '/image_raw/compressed'),
            ('out', '/image_raw'),
        ],
    )

    # 3) 运行已安装的双目分割节点，避免依赖源码目录。
    split_and_info = Node(
        package='stereo_camera_pkg_py',
        executable='stereo_info',
        name='stereo_info',
        output='screen',
        parameters=[{
            'left_yaml_path': left_yaml,
            'right_yaml_path': right_yaml,
        }]
    )

    # 延迟启动，确保 bag play 和 republish 先起来
    delayed_split = TimerAction(period=1.0, actions=[split_and_info])

    return LaunchDescription([
        DeclareLaunchArgument('bag_path'),
        DeclareLaunchArgument('play_rate', default_value='1.0'),

        DeclareLaunchArgument(
            'left_yaml',
            default_value=os.path.join(config_dir, 'left1.yaml')
        ),
        DeclareLaunchArgument(
            'right_yaml',
            default_value=os.path.join(config_dir, 'right1.yaml')
        ),

        play_bag,
        decompress,
        delayed_split,
    ])
