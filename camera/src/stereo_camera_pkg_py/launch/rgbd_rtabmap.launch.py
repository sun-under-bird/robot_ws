from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_share = get_package_share_directory('stereo_camera_pkg')

    base_frame = LaunchConfiguration('base_frame')
    use_viz = LaunchConfiguration('use_viz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_config_file = LaunchConfiguration('ekf_config')

    rtabmap_odom_params = {
        'frame_id': base_frame,
        'subscribe_rgbd': True,
        'subscribe_stereo': False,
        'subscribe_odom_info': True,
        'use_sim_time': use_sim_time,
        'approx_sync': True,
        'topic_queue_size': 20,
        'wait_for_transform': 0.5,
        'Rtabmap/ImagesAlreadyRectified': 'true',
        'publish_tf': False,
        'Vis/FeatureType': '8',
        'Vis/EstimationType': '1',
        'Vis/MinInliers': '20',
        'Vis/MaxFeatures': '1500',
        'Vis/CorType': '0',
        'Odom/ResetCountdown': '1',
        'Odom/Strategy': '0',
        'OdomF2M/MaxSize': '1000',
        'GFTT/MinDistance': '5',
        'GFTT/QualityLevel': '0.00001',
        'Stereo/MaxDisparity': '128',
        'publish_null_when_lost':False,
    }

    rtabmap_slam_params = {
        'frame_id': base_frame,
        'subscribe_rgbd': True,
        'subscribe_stereo': False,
        'subscribe_odom_info': False,
        'subscribe_odom': False,
        'approx_sync_max_interval': 0.2,
        'odom_frame_id':'odom',
        'use_sim_time': use_sim_time,
        'approx_sync': True,
        'sync_queue_size': 10,
        'topic_queue_size': 10,
        'wait_for_transform': 0.5,
        'tf_delay': 0.05,
        'Rtabmap/ImagesAlreadyRectified': 'true',
        'Rtabmap/DetectionRate': '1',
        'Reg/Force3DoF': 'true',
        'Kp/MaxFeatures': '1000',
        'GFTT/MinDistance': '5',
        'GFTT/QualityLevel': '0.00001',
        'Stereo/MaxDisparity': '128',
        'Grid/RayTracing': 'true',
        'Grid/RangeMax': '3',
    }

    odom_remaps = [
        ('rgbd_image', '/stereo_camera/rgbd_image'),
        ('odom', '/vo'),
    ]

    slam_remaps = [
        ('rgbd_image', '/stereo_camera/rgbd_image'),
        ('odom', '/odom'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('use_viz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.join(package_share, 'config', 'ekf_stereo_wheel.yaml')
        ),

        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_left',
            namespace='/stereo/left/camera',
            parameters=[{'use_sim_time': use_sim_time}],
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
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('image', '/stereo/right/camera/image_mono'),
                ('camera_info', '/stereo/right/camera/camera_info'),
                ('image_rect', '/stereo/right/camera/image_rect')
            ]
        ),

        Node(
            package='rtabmap_sync',
            executable='stereo_sync',
            name='stereo_sync',
            output='screen',
            namespace='stereo_camera',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'approx_sync': True},
                {'approx_sync_max_interval': 0.05},
                {'sync_queue_size': 20},
                {'topic_queue_size': 20},
                {'Stereo/MaxDisparity': '128'},
            ],
            remappings=[
                ('left/image_rect', '/stereo/left/camera/image_rect'),
                ('right/image_rect', '/stereo/right/camera/image_rect'),
                ('left/camera_info', '/stereo/left/camera/camera_info'),
                ('right/camera_info', '/stereo/right/camera/camera_info'),
            ]
        ),


#        Node(
#            package='rtabmap_odom',
#            executable='stereo_odometry',
#            name='stereo_odometry',
#            output='screen',
#            parameters=[rtabmap_odom_params],
#            remappings=odom_remaps
#        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_slam_params],
            remappings=slam_remaps,
            arguments=['-d']
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            condition=IfCondition(use_viz),
            parameters=[rtabmap_slam_params],
            remappings=slam_remaps
        ),
    ])
