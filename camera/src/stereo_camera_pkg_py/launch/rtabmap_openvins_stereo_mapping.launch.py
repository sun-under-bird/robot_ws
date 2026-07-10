import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


THIS_DIR = os.path.dirname(__file__)
DEFAULT_CONFIG_DIR = os.path.normpath(os.path.join(THIS_DIR, "..", "config"))


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    openvins_config_path = LaunchConfiguration("openvins_config_path")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    frame_id = LaunchConfiguration("frame_id")
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    map_frame_id = LaunchConfiguration("map_frame_id")

    left_image_topic = LaunchConfiguration("left_image_topic")
    right_image_topic = LaunchConfiguration("right_image_topic")
    left_info_topic = LaunchConfiguration("left_info_topic")
    right_info_topic = LaunchConfiguration("right_info_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_info_topic = LaunchConfiguration("odom_info_topic")

    database_path = LaunchConfiguration("database_path")
    delete_db_on_start = LaunchConfiguration("delete_db_on_start")
    launch_viz = LaunchConfiguration("launch_viz")

    declared_arguments = [
        DeclareLaunchArgument("namespace", default_value="", description="ROS2 namespace。"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="是否使用仿真时间。"),
        DeclareLaunchArgument("log_level", default_value="info", description="节点日志等级。"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(DEFAULT_CONFIG_DIR, "rtabmap_openvins_mapping_params.yaml"),
            description="RTAB-Map + OpenVINS 参数文件。",
        ),
        DeclareLaunchArgument(
            "openvins_config_path",
            default_value="",
            description="可选：OpenVINS 原生 yaml。非空时会覆盖 OdomOpenVINS/* 中同名参数。",
        ),
        DeclareLaunchArgument("frame_id", default_value="camera_link", description="机器人基坐标系。"),
        DeclareLaunchArgument("odom_frame_id", default_value="odom", description="局部里程计坐标系。"),
        DeclareLaunchArgument("map_frame_id", default_value="map", description="全局地图坐标系。"),
        DeclareLaunchArgument(
            "left_image_topic",
            default_value="/camera/camera/infra1/image_rect_raw",
            description="左目校正图像话题。",
        ),
        DeclareLaunchArgument(
            "right_image_topic",
            default_value="/camera/camera/infra2/image_rect_raw",
            description="右目校正图像话题。",
        ),
        DeclareLaunchArgument(
            "left_info_topic",
            default_value="/camera/camera/infra1/camera_info_kalibr",
            description="左目 CameraInfo 话题。",
        ),
        DeclareLaunchArgument(
            "right_info_topic",
            default_value="/camera/camera/infra2/camera_info_kalibr",
            description="右目 CameraInfo 话题。",
        ),
        DeclareLaunchArgument("imu_topic", default_value="/camera/camera/imu", description="IMU 话题。"),
        DeclareLaunchArgument("odom_topic", default_value="/odom", description="OpenVINS 输出里程计话题。"),
        DeclareLaunchArgument(
            "odom_info_topic",
            default_value="/odom_info",
            description="OpenVINS odometry info 话题，RTAB-Map 用它读取特征/内点统计。",
        ),
        DeclareLaunchArgument(
            "database_path",
            default_value=os.path.expanduser("~/.ros/rtabmap_openvins_mapping.db"),
            description="RTAB-Map 数据库路径。",
        ),
        DeclareLaunchArgument("delete_db_on_start", default_value="true", description="启动时是否删除旧地图。"),
        DeclareLaunchArgument("launch_viz", default_value="true", description="是否启动 rtabmap_viz。"),
    ]

    # OpenVINS 双目惯性里程计节点
    openvins_odometry_node = Node(
        package="rtabmap_odom",
        executable="stereo_odometry",
        name="openvins_stereo_odometry",
        namespace=namespace,
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "frame_id": frame_id,
                "odom_frame_id": odom_frame_id,
                "OdomOpenVINS/ConfigPath": openvins_config_path,
            },
        ],
        remappings=[
            ("left/image_rect", left_image_topic),
            ("right/image_rect", right_image_topic),
            ("left/camera_info", left_info_topic),
            ("right/camera_info", right_info_topic),
            ("imu", imu_topic),
            ("odom", odom_topic),
            ("odom_info", odom_info_topic),
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # RTAB-Map 建图节点
    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace=namespace,
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "frame_id": frame_id,
                "odom_frame_id": odom_frame_id,
                "map_frame_id": map_frame_id,
                "database_path": database_path,
                "delete_db_on_start": delete_db_on_start,
            },
        ],
        remappings=[
            ("left/image_rect", left_image_topic),
            ("right/image_rect", right_image_topic),
            ("left/camera_info", left_info_topic),
            ("right/camera_info", right_info_topic),
            ("odom", odom_topic),
            ("odom_info", odom_info_topic),
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # 可视化节点
    rtabmap_viz_node = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        namespace=namespace,
        output="screen",
        condition=IfCondition(launch_viz),
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "frame_id": frame_id,
                "odom_frame_id": odom_frame_id,
                "map_frame_id": map_frame_id,
            },
        ],
        remappings=[
            ("left/image_rect", left_image_topic),
            ("right/image_rect", right_image_topic),
            ("left/camera_info", left_info_topic),
            ("right/camera_info", right_info_topic),
            ("odom", odom_topic),
            ("odom_info", odom_info_topic),
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        declared_arguments
        + [
            openvins_odometry_node,
            rtabmap_node,
            rtabmap_viz_node,
        ]
    )
