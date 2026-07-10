from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,
        'sync_queue_size': 5,
        'topic_queue_size': 5,
        'wait_for_transform': 0.2,
        'tf_delay': 0.05,

        'Rtabmap/ImagesAlreadyRectified': 'true',
        'Rtabmap/DetectionRate': '1',
        'Reg/Force3DoF': 'true',

        'Kp/MaxFeatures': '1000',
        'Kp/NndrRatio': '0.75',

        'GFTT/MinDistance': '10',
        'GFTT/QualityLevel': '0.00001',
        'GFTT/MaxCorners': '400',

        'Stereo/MaxDisparity':'128',
        'qos_image':2,
        'Grid/RangeMax':'4',
        'Grid/NoiseFilteringRadius': '0.15',
'Grid/NoiseFilteringMinNeighbors': '8',
    'Grid/NoiseFilteringRadius': '0.1',
    'Grid/NoiseFilteringMinNeighbors': '5',

	}

    rtabmap_odom_params = {
	'frame_id': base_frame,

        'subscribe_rgbd': False,
        'subscribe_stereo': True,
        'subscribe_odom_info': True,
        'use_sim_time': False,
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,
        'sync_queue_size': 5,
        'topic_queue_size': 5,
        'wait_for_transform': 0.2,
        'tf_delay': 0.05,
        'Rtabmap/ImagesAlreadyRectified': 'true',
        
        'Vis/FeatureType': '8',
        'Vis/EstimationType': '1', 
        'Vis/MinInliers': '10',
        'Vis/MaxFeatures': '1000',
        'Vis/CorType': '0',

        'Odom/Strategy': '0', 
        'OdomF2M/MaxSize': '1000',

        'GFTT/MinDistance': '10',
        'GFTT/QualityLevel': '0.00001',
         'Grid/RayTracing': 'true',
        'Kp/NndrRatio':'0.75',
        'Stereo/MaxDisparity':'128',
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
        DeclareLaunchArgument('baseline', default_value='-0.0518'), 

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
        
                        
        Node(
	   package='image_proc',
	   executable='rectify_node',
	   name='rectify_left',
	   namespace='/stereo/left/camera',
	   remappings=[
	      ('image', '/stereo/left/camera/image_mono'),
	      ('camera_info', '/stereo/left/camera/camera_info'),
	      ('image_rect','/stereo/left/camera/image_rect')]
        ),  


        Node(
           package='image_proc',
           executable='rectify_node',
           name='rectify_right',
           namespace='/stereo/right/camera',
           remappings=[
            ('image', '/stereo/right/camera/image_mono'),
            ('camera_info', '/stereo/right/camera/camera_info'),
            ('image_rect','/stereo/right/camera/image_rect'),]
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

