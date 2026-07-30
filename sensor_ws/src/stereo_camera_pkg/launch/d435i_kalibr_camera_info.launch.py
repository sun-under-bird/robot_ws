from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """按 D435i 图像时间戳发布对应的 Kalibr CameraInfo。"""
    left_image_topic = LaunchConfiguration('left_image_topic')
    right_image_topic = LaunchConfiguration('right_image_topic')
    left_info_topic = LaunchConfiguration('left_info_topic')
    right_info_topic = LaunchConfiguration('right_info_topic')

    declared_arguments = [
        DeclareLaunchArgument(
            'left_image_topic',
            default_value='/camera/camera/infra1/image_rect_raw',
            description='用于触发左目 CameraInfo 的 D435i 图像话题。',
        ),
        DeclareLaunchArgument(
            'right_image_topic',
            default_value='/camera/camera/infra2/image_rect_raw',
            description='用于触发右目 CameraInfo 的 D435i 图像话题。',
        ),
        DeclareLaunchArgument(
            'left_info_topic',
            default_value='/camera/camera/infra1/camera_info_kalibr',
            description='左目 Kalibr CameraInfo 输出话题。',
        ),
        DeclareLaunchArgument(
            'right_info_topic',
            default_value='/camera/camera/infra2/camera_info_kalibr',
            description='右目 Kalibr CameraInfo 输出话题。',
        ),
    ]

    camera_info_node = Node(
        package='stereo_camera_pkg',
        executable='d435i_kalibr_camera_info_node',
        name='d435i_kalibr_camera_info_node',
        output='screen',
        parameters=[{
            # 四个话题由外层 D435i 启动文件统一配置，避免参数覆盖后断链。
            'left_image_topic': left_image_topic,
            'right_image_topic': right_image_topic,
            'left_info_topic': left_info_topic,
            'right_info_topic': right_info_topic,
        }],
    )

    return LaunchDescription(declared_arguments + [camera_info_node])
