# from launch import LaunchDescription
# from launch_ros.actions import Node
# 封装终端指令相关-------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-------
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# 文件包含相关-------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关-------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关-------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-----
# from ament_index_python.packages import get_package_share_directory


import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数（和节点中的参数对应）
    left_cam_idx = DeclareLaunchArgument(
        'left_cam_idx',
        default_value='0',
        description='左摄像头设备索引（/dev/videoX）'
    )
    right_cam_idx = DeclareLaunchArgument(
        'right_cam_idx',
        default_value='2',
        description='右摄像头设备索引（/dev/videoX）'
    )
    width = DeclareLaunchArgument(
        'width',
        default_value='320',
        description='摄像头分辨率宽度'
    )
    height = DeclareLaunchArgument(
        'height',
        default_value='240',
        description='摄像头分辨率高度'
    )
    left_calib_path = DeclareLaunchArgument(
        'left_calib_path',
        default_value=os.path.join(
            get_package_share_directory('stereo_camera_pkg'),  # 你的包名
            'config',  # 推荐在包下建config目录存内参文件
            'left.yaml'
        ),
        description='左相机内参YAML文件路径'
    )
    right_calib_path = DeclareLaunchArgument(
        'right_calib_path',
        default_value=os.path.join(
            get_package_share_directory('stereo_camera_pkg'),
            'config',
            'right.yaml'
        ),
        description='右相机内参YAML文件路径'
    )

    # timer_ms = DeclareLaunchArgument(
    #     'timer_ms',
    #     default_value='100',
    #     description='定时器周期（毫秒），100=10FPS'
    # )

    # 启动双目摄像头节点
    stereo_camera_node = Node(
        package='stereo_camera_pkg',        # 你的包名
        executable='stereo_camera_node',    # 可执行文件名称
        name='stereo_camera_node',          # 节点名称
        output='screen',                    # 日志输出到终端
        namespace='stereo_camera',
        emulate_tty=True,
        parameters=[{
            'left_cam_idx': LaunchConfiguration('left_cam_idx'),
            'right_cam_idx': LaunchConfiguration('right_cam_idx'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'left_calib_path': LaunchConfiguration('left_calib_path'),
            'right_calib_path': LaunchConfiguration('right_calib_path'),
        }]
    )

    # 组装启动描述
    return LaunchDescription([
        left_cam_idx,
        right_cam_idx,
        width,
        height,
        left_calib_path,
        right_calib_path,
        stereo_camera_node
    ])