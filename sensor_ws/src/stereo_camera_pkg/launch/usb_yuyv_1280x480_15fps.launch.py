"""启动 1280×480、15 FPS 的 YUYV 拼接双目相机。"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """创建 USB 相机、双目切分节点和静态 TF 的启动描述。"""

    # 相机输出左右目横向拼接的原始 YUYV 图像。
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video2',
            'image_width': 1280,
            'image_height': 480,
            'framerate': 15.0,
            'pixel_format': 'yuyv',
            'io_method': 'mmap',
            'image_qos': 'sensor_data',
            'gain': 0,
        }]
    )

    # 将拼接图像切分成左右目话题，供后续标定和建图使用。
    stereo_split_node = Node(
        package='stereo_camera_pkg',
        executable='stereo_split_node',
        name='stereo_split_node'
    )

    tf_cam_link_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_cam_link_left',
        arguments=['0.01', '0.025', '0.087', '0', '0', '0', '1', 'camera_link', 'left_camera']
    )

    tf_cam_link_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_cam_link_right',
        arguments=['0.01', '-0.025', '0.087', '0', '0', '0', '1', 'camera_link', 'right_camera']
    )

    tf_left_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_left_optical',
        arguments=['0', '0', '0', '-1.570796', '0', '-1.570796',
                   'left_camera', 'camera_left_frame']
    )

    tf_right_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_right_optical',
        arguments=['0', '0', '0', '-1.570796', '0', '-1.570796',
                   'right_camera', 'camera_right_frame']
    )

    return LaunchDescription([
        tf_cam_link_left,
        tf_cam_link_right,
        tf_left_optical,
        tf_right_optical,
        usb_cam_node,
        stereo_split_node
    ])
