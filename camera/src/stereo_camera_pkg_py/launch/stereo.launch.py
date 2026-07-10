import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('stereo_camera_pkg_py'), 'config'
    )
    left_yaml = os.path.join(config_dir, 'left.yaml')
    right_yaml = os.path.join(config_dir, 'right.yaml')

    cmd_config_camera = [
        'v4l2-ctl', '-d', '/dev/video6', 
        '-c', 'auto_exposure=0',
        '-c', 'exposure_time_absolute=700', 
        '-c', 'white_balance_automatic=1',
#        '-c', 'white_balance_temperature=4500',
        '-c', 'brightness=10'
    ]
    config_action = ExecuteProcess(cmd=cmd_config_camera, output='screen')

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        parameters=[{
            'video_device': '/dev/video6',
            'pixel_format': 'mjpeg2rgb',
            'image_width': 640,
            'image_height': 240,
            'framerate': 30.0,
            'camera_name': 'default_cam',
        }]
    )

    stereo_driver = Node(
        package='stereo_camera_pkg_py',
        executable='stereo_info',
        name='stereo_info',
        output='screen',
        parameters=[{
            'left_yaml_path': left_yaml,
            'right_yaml_path': right_yaml,
        }]
    )

    return LaunchDescription([
        config_action,
        TimerAction(period=1.0, actions=[usb_cam_node]),
        TimerAction(period=2.0, actions=[stereo_driver]),
    ])
