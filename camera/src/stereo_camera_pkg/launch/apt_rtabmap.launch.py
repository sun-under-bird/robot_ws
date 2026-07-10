from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 
def generate_launch_description():
 
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    rtabmap_common_params = {
        'frame_id': 'left_camera', 
        'odom_frame_id': 'left_camera', 
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_stereo': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,  
        'wait_for_transform': 0.2,        
        'use_sim_time': use_sim_time
        }
 
    rtabmap_remaps = [
        # 双目左目图像
        ('left/image_rect', '/left_camera/image_rect'),
        ('left/camera_info', '/left_camera/camera_info'),
        # 双目右目图像
        ('right/image_rect', '/right_camera/image_rect'),
        ('right/camera_info', '/right_camera/camera_info'),
    ]
 
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_left_camera_tf',
        #     arguments=[
        #         '0', '0', '0',  # x/y/z 平移（相机在base_link原点）
        #         '0', '0', '0',  # roll/pitch/yaw 旋转（无旋转）
        #         'base_link',     # 父坐标系
        #         'left_camera'    # 子坐标系（左目相机，和话题对应）
        #     ]
        # ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_right_camera_tf',
            arguments=[
                '-0.0828', '0', '0', # x平移（双目基线，根据你的相机实际间距改，比如6cm）
                '0', '0', '0',    # 旋转
                'left_camera',      # 父坐标系
                'right_camera'    # 子坐标系（右目相机）
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'left_camera']
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_right_camera_tf',
        #     arguments=[
        #         '0', '0', '0', # x平移（双目基线，根据你的相机实际间距改，比如6cm）
        #         '0', '0', '0',    # 旋转
        #         'odom',      # 父坐标系
        #         'base_link'    # 子坐标系（右目相机）
        #     ]
        # ),

        # Node(
        #     package='rtabmap_odom',
        #     executable='stereo_odometry',
        #     name='stereo_odometry',
        #     parameters=[rtabmap_common_params],
        #     remappings=rtabmap_remaps,
        #     output='screen',
        # ),

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[rtabmap_common_params],
            remappings=rtabmap_remaps,
            output='screen',
        ),

        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            parameters=[rtabmap_common_params],
            remappings=rtabmap_remaps,
            output='screen',
        )
    ])