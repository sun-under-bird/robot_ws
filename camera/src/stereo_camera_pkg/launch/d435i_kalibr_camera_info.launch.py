from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stereo_camera_pkg',
            executable='d435i_kalibr_camera_info_node',
            name='d435i_kalibr_camera_info_node',
            output='screen',
        ),
    ])
