from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_stereo_image_proc = get_package_share_directory('stereo_image_proc')
    stereo_image_proc_launch = PathJoinSubstitution(
        [pkg_stereo_image_proc, 'launch', 'stereo_image_proc.launch.py']
    )
    base_frame = LaunchConfiguration('base_frame')
    use_viz = LaunchConfiguration('use_viz')
    approx_sync = LaunchConfiguration('approx_sync')
    baseline = LaunchConfiguration('baseline')

    rtabmap_slam_params = {
	    'frame_id': base_frame,
	    'subscribe_rgbd': False,
	    'subscribe_stereo': True,
	    'subscribe_odom_info': True,
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

	    'Kp/MaxFeatures': '500',
	    'Kp/NndrRatio': '0.75',

	    'GFTT/MinDistance': '10',
	    'GFTT/QualityLevel': '0.0001',
	    'GFTT/MaxCorners': '500',

	    'Stereo/MaxDisparity':'128',
    

	    'Grid/CellSize':'0.05',
	    'Grid/3D':'true',
	    'Grid/GroundIsObstacle':'false',
            'Grid/MaxObstacleHeight':'1',

	}

    rtabmap_odom_params = {
	    'frame_id': base_frame,

	    # stereo mode
	    'subscribe_rgbd': False,
	    'subscribe_stereo': True,
	    'subscribe_odom_info': True,
	    'use_sim_time': False,
	    'approx_sync': approx_sync,
	    'approx_sync_max_interval': 0.05,
	    'sync_queue_size': 15,
	    'topic_queue_size': 15,
	    'wait_for_transform': 0.2,
	    'tf_delay': 0.05,
	    'Rtabmap/ImagesAlreadyRectified': 'true',
	   
	    'Vis/EstimationType': '1', 
	    'Vis/MinInliers': '8',
	    'Vis/MaxFeatures': '700',

	    'OdomF2M/MaxSize': '500',
	    'Odom/KeyFrameThr': '0.5',
	    'Odom/ScanKeyFrameThr': '0.5',

	    'GFTT/MinDistance': '5',
	    'GFTT/QualityLevel': '0.0001',
	    'GFTT/MaxCorners': '500',

	    'Stereo/MaxDisparity':'128'
	}

    
    remaps = [
        ('left/image_rect',   '/stereo/left/camera/image_rect'),
        ('right/image_rect',  '/stereo/right/camera/image_rect'),
        ('left/camera_info',  '/stereo/left/camera/camera_info'),
        ('right/camera_info', '/stereo/right/camera/camera_info'),
        ('odom',              '/vo'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='camera_link'),
        DeclareLaunchArgument('use_viz', default_value='true'),
        DeclareLaunchArgument('approx_sync', default_value='true'),
        DeclareLaunchArgument('baseline', default_value='-0.05'), 

        # TF
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
            arguments=['0', baseline, '0', '0', '0', '0', '1', 'camera_link', 'right_camera']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_left_optical',
            arguments=['0', '0', '0', '-1.570796', '0', '-1.570796',
                       'left_camera', 'camera_left_frame']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_right_optical',
            arguments=['0', '0', '0', '-1.570796', '0', '-1.570796',
                       'right_camera', 'camera_right_frame']
        ),
        
                        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stereo_image_proc_launch),
            launch_arguments=[
                ('left_namespace',  '/stereo/left/camera'),
                ('right_namespace', '/stereo/right/camera'),
                ('disparity_range', '128'),
                ('approximate_sync','True'),
            ]
        ),

        # stereo_odometry
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[rtabmap_odom_params],
            remappings=remaps
        ),

        # rtabmap
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{"odometry_node_name": 'stereo_odometry'},rtabmap_slam_params],
            remappings=remaps,
            arguments=['-d']
        ),

        # rtabmap_viz
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            condition=IfCondition(use_viz),
            parameters=[rtabmap_slam_params, {'odometry_node_name': 'stereo_odometry'}],
            remappings=remaps
        ),
        
 
    ])

