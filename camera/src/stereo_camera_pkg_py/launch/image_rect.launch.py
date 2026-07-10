import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_rgbd': False,
                'subscribe_stereo': True,
                'subscribe_odom_info': True,
                'use_sim_time': False,
                'approx_sync': True,
                'approx_sync_max_interval': 0.01,
                'sync_queue_size': 10,
                'topic_queue_size': 10,
                'wait_for_transform': 0.2,
                'tf_delay': 0.05,
                'Rtabmap/ImagesAlreadyRectified': 'true',
            
                'Vis/EstimationType': '1', 
                'Vis/MinInliers': '15',
                'Vis/MaxFeatures': '700',
                'Vis/CorType': '0',

                'Odom/Strategy': '0', 
                'OdomF2M/MaxSize': '500',

                'GFTT/MinDistance': '10',
                'GFTT/QualityLevel': '0.00001',

                'Stereo/MaxDisparity':'128',
            }],
            remappings=[
                ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
                ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
                ('left/camera_info', '/camera/camera/infra1/camera_info'),
                ('right/camera_info', '/camera/camera/infra2/camera_info'),
            ]
        ),

        # 3. RTAB-Map ½¨Í¼Ö÷½Úµã
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_rgbd': False,
                'subscribe_stereo': True,
                'subscribe_odom_info': True,
                'use_sim_time': False,
                'approx_sync': True,
                'approx_sync_max_interval': 0.01,
                'sync_queue_size': 10,
                'topic_queue_size': 10,
                'wait_for_transform': 0.2,
                'tf_delay': 0.05,

                'Rtabmap/ImagesAlreadyRectified': 'true',
                'Rtabmap/DetectionRate': '1',
                'Reg/Force3DoF': 'false',

                'Kp/MaxFeatures': '400',
                'Kp/NndrRatio': '0.75',

                'GFTT/MinDistance': '10',
                'GFTT/QualityLevel': '0.00001',
                'GFTT/MaxCorners': '400',

                'Stereo/MaxDisparity':'128',
            }],
            remappings=[
                ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
                ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
                ('left/camera_info', '/camera/camera/infra1/camera_info'),
                ('right/camera_info', '/camera/camera/infra2/camera_info'),
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
                'frame_id': 'camera_link',
                'approx_sync': True,
            }],
            remappings=[
                ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
                ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
                ('left/camera_info', '/camera/camera/infra1/camera_info'),
                ('right/camera_info', '/camera/camera/infra2/camera_info'),
            ]
        ),
    ])
