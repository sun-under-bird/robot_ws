from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    base_frame = LaunchConfiguration('base_frame')
    use_viz = LaunchConfiguration('use_viz')
    approx_sync = LaunchConfiguration('approx_sync')

    rtabmap_slam_params = {
        'frame_id': base_frame,
        'subscribe_rgbd': False,
        'subscribe_stereo': True,
        'subscribe_odom_info': False,
        'subscribe_odom':False,
        'odom_frame_id':'odom',
        'use_sim_time': False,
        'approx_sync': approx_sync,
        'approx_sync_max_interval': 0.05,
        'sync_queue_size': 15,
        'topic_queue_size': 15,
        'wait_for_transform': 0.2,
        'tf_delay': 0.05,
        'Rtabmap/ImagesAlreadyRectified': 'true',
        'Rtabmap/DetectionRate': '1',
        'Reg/Force3DoF': 'false',
        'Reg/Strategy': '0',
        'Kp/MaxFeatures': '1000',
        'Kp/NndrRatio': '0.75',
        'GFTT/MinDistance': '5',
        'GFTT/QualityLevel': '0.00001',
        'GFTT/MaxCorners': '500',
        'Stereo/MaxDisparity': '128',
        'Grid/CellSize': '0.05',
        'Grid/3D': 'true',
        'Grid/GroundIsObstacle': 'false',
        'Grid/RangeMax': '3',
    }

    remaps_slam = [
        ('left/image_rect', '/stereo/left/camera/image_rect'),
        ('right/image_rect', '/stereo/right/camera/image_rect'),
        ('left/camera_info', '/stereo/left/camera/camera_info'),
        ('right/camera_info', '/stereo/right/camera/camera_info'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('use_viz', default_value='true'),
        DeclareLaunchArgument('approx_sync', default_value='true'),


        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_left',
            namespace='/stereo/left/camera',
            remappings=[
                ('image', '/stereo/left/camera/image_mono'),
                ('camera_info', '/stereo/left/camera/camera_info'),
                ('image_rect', '/stereo/left/camera/image_rect')
            ]
        ),

        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_right',
            namespace='/stereo/right/camera',
            remappings=[
                ('image', '/stereo/right/camera/image_mono'),
                ('camera_info', '/stereo/right/camera/camera_info'),
                ('image_rect', '/stereo/right/camera/image_rect'),
            ]
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_slam_params],
            remappings=remaps_slam,
            arguments=['-d']
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            condition=IfCondition(use_viz),
            parameters=[rtabmap_slam_params],
            remappings=remaps_slam
        ),
    ])
