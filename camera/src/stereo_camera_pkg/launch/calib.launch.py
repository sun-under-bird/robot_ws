from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    baseline = 0.08192  # m

    left_rectify = Node(
        package='image_proc',
        executable='rectify_node',
        name='left_rectify',
        output='screen',
        remappings=[
            # input
            ('image',       '/left_camera/image_raw'),
            ('camera_info', '/left_camera/camera_info_raw'),

            # output base
            ('image_rect',  '/left_camera/image_rect'),

            # IMPORTANT: also remap image_transport sub-topics
            ('image_rect/compressed',      '/left_camera/image_rect/compressed'),
            ('image_rect/compressedDepth', '/left_camera/image_rect/compressedDepth'),
            ('image_rect/theora',          '/left_camera/image_rect/theora'),
        ],
        parameters=[{
            # 一般保持默认即可；如果你遇到 QoS 警告，再加 qos_overrides（下面我也给了模板）
            'use_system_default_qos': False,
        }]
    )

    right_rectify = Node(
        package='image_proc',
        executable='rectify_node',
        name='right_rectify',
        output='screen',
        remappings=[
            ('image',       '/right_camera/image_raw'),
            ('camera_info', '/right_camera/camera_info_raw'),

            ('image_rect',  '/right_camera/image_rect'),

            ('image_rect/compressed',      '/right_camera/image_rect/compressed'),
            ('image_rect/compressedDepth', '/right_camera/image_rect/compressedDepth'),
            ('image_rect/theora',          '/right_camera/image_rect/theora'),
        ],
        parameters=[{
            'use_system_default_qos': False,
        }]
    )

    disparity = Node(
        package='stereo_image_proc',
        executable='disparity_node',
        name='disparity_node',
        output='screen',
        parameters=[{
            'approximate_sync': True,
            'queue_size': 30,

            # 关键：统一 QoS（全部 BEST_EFFORT）
            'qos_overrides./left_camera/image_rect.subscription.reliability': 'best_effort',
            'qos_overrides./right_camera/image_rect.subscription.reliability': 'best_effort',
            'qos_overrides./left_camera/camera_info.subscription.reliability': 'best_effort',
            'qos_overrides./right_camera/camera_info.subscription.reliability': 'best_effort',
            'qos_overrides./stereo/disparity.publisher.reliability': 'best_effort',
        }],
        remappings=[
            ('left/image_rect',   '/left_camera/image_rect'),
            ('left/camera_info',  '/left_camera/camera_info'),
            ('right/image_rect',  '/right_camera/image_rect'),
            ('right/camera_info', '/right_camera/camera_info'),
            ('disparity',         '/stereo/disparity'),
        ]
    )

    points2 = Node(
        package='stereo_image_proc',
        executable='point_cloud_node',
        name='point_cloud_node',
        output='screen',
        parameters=[{
            'approximate_sync': True,
            'queue_size': 30,

            'qos_overrides./stereo/disparity.subscription.reliability': 'best_effort',
            'qos_overrides./left_camera/camera_info.subscription.reliability': 'best_effort',
            'qos_overrides./right_camera/camera_info.subscription.reliability': 'best_effort',
            'qos_overrides./left/image_rect_color.subscription.reliability': 'best_effort',
            'qos_overrides./stereo/points2.publisher.reliability': 'best_effort',
        }],
        remappings=[
            ('left/image_rect_color', '/left_camera/image_rect'),
            ('left/camera_info',      '/left_camera/camera_info'),
            ('right/camera_info',     '/right_camera/camera_info'),
            ('disparity',             '/stereo/disparity'),
            ('points2',               '/stereo/points2'),
        ]
    )

    tf_cam_link_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_cam_link_left',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_link', 'left_camera']
    )

    tf_cam_link_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_cam_link_right',
        arguments=[str(baseline), '0', '0', '0', '0', '0', '1', 'camera_link', 'right_camera']
    )

    tf_left_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_left_optical',
        arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963', 'left_camera', 'left_camera_optical']
    )

    tf_right_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_right_optical',
        arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963', 'right_camera', 'right_camera_optical']
    )

    return LaunchDescription([
        tf_cam_link_left, tf_cam_link_right,
        tf_left_optical, tf_right_optical,
        left_rectify, right_rectify,
        disparity, points2,
    ])
