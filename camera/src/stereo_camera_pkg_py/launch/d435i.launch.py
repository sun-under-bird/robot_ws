import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='d435i_body_to_camera_link',
            arguments=[
                '--x', '-0.00552',
                '--y', '0.00510',
                '--z', '0.01174',
                '--qx', '0.5',
                '--qy', '-0.5',
                '--qz', '0.5',
                '--qw', '0.5',
                '--frame-id', 'body',
                '--child-frame-id', 'camera_link',
            ],
        ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'body',
                'subscribe_rgbd': False,
                'subscribe_stereo': True,
                'subscribe_odom_info': False,
                'use_sim_time': False,
                'approx_sync': True,
                'approx_sync_max_interval': 0.1,
                'sync_queue_size': 20,
                'topic_queue_size': 20,
                'wait_for_transform': 0.2,
                'tf_delay': 0.05,
                'Rtabmap/ImagesAlreadyRectified': 'true',
                'Rtabmap/DetectionRate': '1',
                'Reg/Force3DoF': 'false',
                'Kp/MaxFeatures': '400',
                'Kp/NndrRatio': '0.75',
                'GFTT/MinDistance': '10',
                'GFTT/QualityLevel': '0.0001',
                'GFTT/MaxCorners': '400',
                'Stereo/MaxDisparity':'128',
            }],
            remappings=[
                ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
                ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
		('left/camera_info', '/camera/camera/infra1/camera_info_kalibr'),
		('right/camera_info', '/camera/camera/infra2/camera_info_kalibr'),
                ('odom','/odometry')
            ],
            arguments=['--delete_db_on_start']
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'subscribe_stereo': True,
                'frame_id': 'body',
                'approx_sync': True,
                'approx_sync_max_interval': 0.1,
                'sync_queue_size': 20,
                'topic_queue_size': 20,
                'wait_for_transform': 0.5,
            }],
            remappings=[
                ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
                ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
		('left/camera_info', '/camera/camera/infra1/camera_info_kalibr'),
		('right/camera_info', '/camera/camera/infra2/camera_info_kalibr'),
                ('odom', '/odometry')
            ]
        ),
    ])
