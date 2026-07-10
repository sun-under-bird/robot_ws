from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter, SetRemap

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_stereo_image_proc = get_package_share_directory('stereo_image_proc')
    stereo_image_proc_launch = PathJoinSubstitution(
        [pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py']
    )

    localization = LaunchConfiguration('localization')
    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_id = LaunchConfiguration('frame_id')

    # rtabmap_odom / rtabmap_slam 通用参数
    parameters = {
        'frame_id': frame_id,

        # ✅ 走 RGBD 订阅（来自 rtabmap_sync/stereo_sync 输出的 /stereo_camera/rgbd_image）
        'subscribe_rgbd': True,

        # 手持通常帧率/延迟会抖一点，建议 approx sync
        'approx_sync': True,
        'sync_queue_size': 50,
        'topic_queue_size': 20,

        # 你的需求里常用的一些
        'map_negative_poses_ignored': True,
        'subscribe_odom_info': True,

        # RTAB-Map 内部参数要用字符串
        'OdomF2M/MaxSize': '1000',
        'GFTT/MinDistance': '10',
        'GFTT/QualityLevel': '0.00001',

        # 手持双目常见点云飘散，适当限制生成深度/栅格的距离
        'Grid/RangeMax': '2.0',
        'Grid/NormalsSegmentation': 'True',

        'use_sim_time': use_sim_time,
    }

    # rtabmap 节点 remap：rgbd_image + odom
    remappings = [
        ('rgbd_image', '/stereo_camera/rgbd_image'),
        ('odom', '/vo'),
    ]

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'),
        'config',
        'demo_robot_mapping.rviz'
    )

    return LaunchDescription([
        # ---------------- Args ----------------
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Launch RTAB-Map UI.'),
        DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz2.'),
        DeclareLaunchArgument('localization', default_value='false', description='Localization mode.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz, description='RViz config path.'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock (rosbag).'),
        DeclareLaunchArgument('frame_id', default_value='camera_link', description='RTAB-Map base frame id.'),

        SetParameter(name='use_sim_time', value=use_sim_time),

        # ---------------- 1) stereo_image_proc：用 raw + camera_info_raw 做矫正 ----------------
        # 关键：把 stereo_image_proc 订阅的 camera_info remap 到 camera_info_raw
        # 这样它拿到的是 RAW 的 K+D（不带 baseline 的 P[3]）
        GroupAction(actions=[
            SetRemap(src='camera_info', dst='camera_info_raw'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([stereo_image_proc_launch]),
                launch_arguments=[
                    # 你的相机话题命名空间就是 /left_camera 和 /right_camera
                    ('left_namespace', 'left_camera'),
                    ('right_namespace', 'right_camera'),

                    # 视差范围可以按你双目的 baseline+fx 调，先用 128/256 试
                    ('disparity_range', '128'),

                    # 如果你输出是 mono8，更推荐 grayscale；如果你输出 bgr8 也能跑
                    # stereo_image_proc 会自动根据输入选择
                ]
            ),
        ]),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_cam_link_left',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'camera_link', 'left_camera']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_cam_link_right',
            arguments=['0.08192', '0', '0', '0', '0', '0', '1', 'camera_link', 'right_camera']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_left_frame_optical',
            arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963', 'left_camera', 'left_camera_optical']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_right_frame_optical',
            arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963', 'right_camera', 'right_camera_optical']
        ),

        # ---------------- 2) rtabmap_sync：把 rect + camera_info 合成 rgbd_image ----------------
        Node(
            package='rtabmap_sync',
            executable='stereo_sync',
            name='stereo_sync',
            output='screen',
            namespace='stereo_camera',
            remappings=[
                ('left/image_rect',   '/left_camera/image_rect'),
                ('right/image_rect',  '/right_camera/image_rect'),
                ('left/camera_info',  '/left_camera/camera_info_raw'),
                ('right/camera_info', '/right_camera/camera_info_raw'),
            ]
        ),

        # ---------------- 3) Visual odometry（输出 /vo） ----------------
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
        ),

        # ---------------- 4) RTAB-Map SLAM / Localization ----------------
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['--delete_db_on_start'],
        ),

        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[parameters, {
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True'
            }],
            remappings=remappings,
        ),

        # ---------------- 5) Visualization ----------------
        Node(
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[parameters, {"odometry_node_name": "stereo_odometry"}],
            remappings=remappings,
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_cfg')],
        ),
    ])
