from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 将字符串形式的 launch 参数转换为 bool。
def _as_bool(value: str) -> bool:
    return str(value).lower() in ("true", "1", "yes", "on")


# 创建 v4l2 相机节点和双目图像拆分节点。
def _launch_setup(context, *args, **kwargs):
    image_width = int(LaunchConfiguration("image_width").perform(context))
    image_height = int(LaunchConfiguration("image_height").perform(context))

    params = {
        "video_device": LaunchConfiguration("video_device").perform(context),

        # 双目拼接总图：左 1280x720 + 右 1280x720 = 2560x720。
        "image_size": [image_width, image_height],

        # v4l2_camera 当前不支持 MJPG，这里必须使用 YUYV。
        "pixel_format": LaunchConfiguration("pixel_format").perform(context),

        # 输出灰度图，后续给 SLAM 使用。
        "output_encoding": LaunchConfiguration("output_encoding").perform(context),

        "camera_frame_id": LaunchConfiguration("camera_frame_id").perform(context),
        "camera_info_url": LaunchConfiguration("camera_info_url").perform(context),
        "use_sim_time": _as_bool(LaunchConfiguration("use_sim_time").perform(context)),

        # 相机控制参数。
        "brightness": int(LaunchConfiguration("brightness").perform(context)),
        "contrast": int(LaunchConfiguration("contrast").perform(context)),
        "saturation": int(LaunchConfiguration("saturation").perform(context)),
        "hue": int(LaunchConfiguration("hue").perform(context)),
        "gamma": int(LaunchConfiguration("gamma").perform(context)),
        "frame_rate": float(LaunchConfiguration("frame_rate").perform(context)),

        # SLAM 建议关闭自动白平衡，避免图像亮度/颜色漂移。
        "white_balance_automatic": _as_bool(
            LaunchConfiguration("white_balance_automatic").perform(context)
        ),
    }

    v4l2_cam_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="v4l2_camera",
        output="screen",
        parameters=[params],
    )

    stereo_split_node = Node(
        package="stereo_camera_pkg",
        executable="stereo_split_node",
        name="stereo_split_node",
        output="screen",
    )

    return [v4l2_cam_node, stereo_split_node]


# 生成 launch 描述，并声明所有可覆盖参数。
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),

        DeclareLaunchArgument("image_width", default_value="1280"),
        DeclareLaunchArgument("image_height", default_value="480"),

        DeclareLaunchArgument("pixel_format", default_value="YUYV"),
        DeclareLaunchArgument("frame_rate", default_value="100.0"),
        DeclareLaunchArgument("output_encoding", default_value="yuv422_yuy2"),

        DeclareLaunchArgument("camera_frame_id", default_value="stereo_camera"),
        DeclareLaunchArgument("camera_info_url", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        DeclareLaunchArgument("brightness", default_value="0"),
        DeclareLaunchArgument("contrast", default_value="32"),
        DeclareLaunchArgument("saturation", default_value="38"),
        DeclareLaunchArgument("hue", default_value="0"),
        DeclareLaunchArgument("gamma", default_value="150"),
        DeclareLaunchArgument("white_balance_automatic", default_value="false"),

        OpaqueFunction(function=_launch_setup),
    ])
